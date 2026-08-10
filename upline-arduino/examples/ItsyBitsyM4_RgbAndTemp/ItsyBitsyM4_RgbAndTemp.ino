// Upline on an Adafruit ItsyBitsy M4 Express (ATSAMD51G19A).
//
// Exposes two things over USB serial, with no board-specific host code:
//
//   rgb   int    writable  the onboard DotStar, as one 24-bit colour 0..0xFFFFFF
//   tempf fix2   readable  the SAMD51's own die temperature, in °F
//
// Open the port, wait for a heartbeat, send ^?^, and the board answers with a
// schema describing both. Then:
//
//   ^rgb~16711680^      full red      (0xFF0000)
//   ^rgb~32768^         mid green     (0x008000)
//   ^rgb~0^             off
//
// Each is echoed back immediately with just that key, so a host sees the colour
// take effect without waiting for the next tick — and sees the clamp when it
// overshoots:
//
//   ^rgb~16711680^   ->  ^rgb~16711680^
//   ^rgb~99999999^   ->  ^rgb~16777215^     clamped to the declared maximum
//
// and every two seconds the board sends something like:
//
//   ^tempf~78.35^rgb~16711680^
//
// Colour arrives as a plain integer rather than hex because `int` is a
// standard Upline type that every host already understands (spec §8.3).
//
// Licence: CC0 / public domain.

#include <upline.hpp>

// ── The device descriptor ────────────────────────────────────────────────────
// Lives in flash. min/max let a host build a slider and a colour picker without
// knowing anything about this board.
UPLINE_SCHEMA(itsyBitsySchema,
  // A readable local id, which spec §7 allows for one-offs. Every board
  // flashed with this example shares it — give each unit its own before you
  // ship anything, ideally a UUID v4 as unpadded base64url.
  "_i|itsybitsy-m4-demo"
  "~_n|ItsyBitsy M4"
  "~_d|Onboard RGB and die temperature"
  "~rgb|rw|int|0|0|16777215"              // 0x000000 .. 0xFFFFFF
  "~tempf|r|fix2||-40.00|200.00");

Upline upline(Serial, itsyBitsySchema);

// Calibration trim, in hundredths of a degree F. Hardcoded for testing: the
// factory data fixes the sensor's SLOPE, but bench test to set OFFSET.
static const int32_t DEGREE_F_OFFSET_FX2 = -1200;        // -12.00 F

// ── Onboard DotStar (APA102) ─────────────────────────────────────────────────
// Bit-banged in a dozen lines so this example needs no extra library. The
// variant header gives us the pins.
static void dotStarShiftOut(uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    digitalWrite(PIN_DOTSTAR_DATA, (value & 0x80) ? HIGH : LOW);
    digitalWrite(PIN_DOTSTAR_CLK, HIGH);
    digitalWrite(PIN_DOTSTAR_CLK, LOW);
    value <<= 1;
  }
}

/**
 * Write one 24-bit colour to the onboard DotStar.
 * @param colour 0x00RRGGBB
 */
static void dotStarSetColour(uint32_t colour) {
  const uint8_t red   = (uint8_t)(colour >> 16);
  const uint8_t green = (uint8_t)(colour >> 8);
  const uint8_t blue  = (uint8_t)(colour);
  for (uint8_t i = 0; i < 4; ++i) dotStarShiftOut(0x00);   // start frame
  dotStarShiftOut(0xFF);                                   // full brightness
  dotStarShiftOut(blue);                                   // APA102 wants B,G,R
  dotStarShiftOut(green);
  dotStarShiftOut(red);
  for (uint8_t i = 0; i < 4; ++i) dotStarShiftOut(0xFF);   // end frame
}

// ── SAMD51 die temperature ───────────────────────────────────────────────────
// Follows the SAMD51 datasheet, "Device Temperature Measurement": read the PTAT
// and CTAT sensors on ADC1, then interpolate using the two calibration points
// the factory wrote into the NVM Temperature Log row.
//
// The waits below are bounded rather than bare `while` loops. That is the one
// piece of belt-and-braces worth keeping: an unbounded spin here locks the chip
// inside setup(), and the board then enumerates over USB but never sends a
// single byte, which is a genuinely unpleasant thing to debug.

#define ADC_SPIN_LIMIT 200000u
static void temperatureBegin() {
  // Power up the temperature sensor inside the supply controller, and select
  // the 1.0 V bandgap — the reference the factory calibration was taken at.
  SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;
  SUPC->VREF.reg |= SUPC_VREF_ONDEMAND | SUPC_VREF_TSEN | SUPC_VREF_VREFOE;
  // ADC1 is untouched by the Arduino core, so it is ours to configure.
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC1;
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  ADC1->CTRLA.bit.SWRST = 1;
  for (uint32_t g = ADC_SPIN_LIMIT; ADC1->SYNCBUSY.bit.SWRST && g; --g) {}
  ADC1->CTRLA.reg    = ADC_CTRLA_PRESCALER_DIV32;
  ADC1->CTRLB.reg    = ADC_CTRLB_RESSEL_12BIT;
  for (uint32_t g = ADC_SPIN_LIMIT; ADC1->SYNCBUSY.bit.CTRLB && g; --g) {}
  ADC1->SAMPCTRL.reg = 63;                                 // slow sample: it is a slow signal
  // INTREF is the 1.0 V bandgap — NOT one of the INTVCC options, and beware the
  // naming: on SAMD51 INTVCC1 is the full VDDANA (3.3 V) and INTVCC0 is the
  // half-rail one. Choosing VDDANA scales every reading down by 3.3x, so PTAT
  // reads about 830 where the calibration says it should be near 2700 — a
  // plausible-looking number that is quietly a third of the truth. This was the
  // actual bug behind the wildly high temperature.
  ADC1->REFCTRL.reg  = ADC_REFCTRL_REFSEL_INTREF;
  ADC1->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
  ADC1->CTRLA.bit.ENABLE = 1;
  for (uint32_t g = ADC_SPIN_LIMIT; ADC1->SYNCBUSY.bit.ENABLE && g; --g) {}
}

// How many conversions to average per channel. Noise falls as the square root
// of this, so 4 halves it and 16 quarters it. Each conversion is about 50 us,
// so even 16 costs well under a millisecond once a second.
#define TEMP_SAMPLE_COUNT 4

static uint16_t adcConvertOnce() {
  ADC1->INTFLAG.reg = ADC_INTFLAG_RESRDY;          // no stale flag to trip over
  ADC1->SWTRIG.bit.START = 1;
  for (uint32_t g = ADC_SPIN_LIMIT; ADC1->INTFLAG.bit.RESRDY == 0 && g; --g) {}
  return (uint16_t)ADC1->RESULT.reg;
}

/**
 * Select a channel and return the mean of TEMP_SAMPLE_COUNT conversions.
 *
 * The reading immediately after a mux change is not valid, so one is thrown
 * away first — which is safe now that each conversion completes before the next
 * begins. The sensor is genuinely noisy; averaging is worth the microseconds.
 */
static uint16_t adcReadAveraged(uint8_t muxPositive) {
  ADC1->INPUTCTRL.bit.MUXPOS = muxPositive;
  for (uint32_t g = ADC_SPIN_LIMIT; ADC1->SYNCBUSY.bit.INPUTCTRL && g; --g) {}
  adcConvertOnce();                                // discard the settling sample
  uint32_t total = 0;
  for (uint8_t sample = 0; sample < TEMP_SAMPLE_COUNT; ++sample) {
    total += adcConvertOnce();
  }
  return (uint16_t)(total / TEMP_SAMPLE_COUNT);
}

/**
 * Read the die temperature in degrees Fahrenheit.
 *
 * The board hands us a float, which is the case spec §8.3 describes: keep the
 * float out of the wire format by converting to a scaled integer here, so the
 * device never links a floating-point text formatter.
 *
 * @return temperature in hundredths of a degree F, e.g. 7835 for 78.35 °F
 */
static int32_t readDieTemperatureFx2() {
  // Factory calibration, NVM Temperature Log row. The address is taken from the
  // datasheet rather than a CMSIS symbol: the NVMCTRL_TEMP_LOG_Wn defines are
  // spaced 0x10 apart and are not the three consecutive words we need here.
  const volatile uint32_t* temperatureLog = (const volatile uint32_t*)0x00800100UL;
  const uint32_t logWord0 = temperatureLog[0];   // TLI, TLD, THI, THD
  const uint32_t logWord1 = temperatureLog[1];   // room/hot PTAT
  const uint32_t logWord2 = temperatureLog[2];   // room/hot CTAT
  const float temperatureLow  = ((logWord0 >>  0) & 0xFF) + (((logWord0 >>  8) & 0x0F) / 10.0f);
  const float temperatureHigh = ((logWord0 >> 12) & 0xFF) + (((logWord0 >> 20) & 0x0F) / 10.0f);
  const float ptatLow   = (float)((logWord1 >>  8) & 0x0FFF);
  const float ptatHigh  = (float)((logWord1 >> 20) & 0x0FFF);
  const float ctatLow   = (float)((logWord2 >>  0) & 0x0FFF);
  const float ctatHigh  = (float)((logWord2 >> 12) & 0x0FFF);
  const float ptatMeasured = (float)adcReadAveraged(ADC_INPUTCTRL_MUXPOS_PTAT_Val);
  const float ctatMeasured = (float)adcReadAveraged(ADC_INPUTCTRL_MUXPOS_CTAT_Val);
  const float numerator =
      temperatureLow  * ptatHigh * ctatMeasured
    - temperatureHigh * ptatLow  * ctatMeasured
    - temperatureLow  * ctatHigh * ptatMeasured
    + temperatureHigh * ctatLow  * ptatMeasured;
  const float denominator =
      ptatHigh * ctatMeasured
    - ptatLow  * ctatMeasured
    - ctatHigh * ptatMeasured
    + ctatLow  * ptatMeasured;
  if (denominator == 0.0f) return 0;                       // calibration missing
  const float celsius    = numerator / denominator;
  const float fahrenheit = celsius * 1.8f + 32.0f;
  int32_t fahrenheitFx2 =
      (int32_t)(fahrenheit * 100.0f + (fahrenheit < 0 ? -0.5f : 0.5f));
  fahrenheitFx2 += DEGREE_F_OFFSET_FX2;
  // Never emit a value outside the range the schema promises.
  if (fahrenheitFx2 < -4000)  fahrenheitFx2 = -4000;
  if (fahrenheitFx2 > 20000)  fahrenheitFx2 = 20000;
  return fahrenheitFx2;
}

// ── Application state ────────────────────────────────────────────────────────

static const uint16_t REPORT_INTERVAL_MS = 2000;           // inside the 2.5 s heartbeat

static uint32_t currentColour = 0;
static uint32_t lastReportMillis = 0;
static bool     ackIsPending = false;                      // a command owes a reply

/**
 * Called once for every key/value pair the host sends.
 * Unknown keys are ignored, which is what keeps a device forward-compatible.
 */
void uplineOnEntry(const UplineEntry& entry) {
  if (entry.isRead()) {                                    // report without changing
    if (!strcmp(entry.key, "rgb")) ackIsPending = true;
    return;
  }
  if (!strcmp(entry.key, "rgb")) {
    int32_t requested;
    if (!uplineParseNumber(entry.value(0), 0, &requested)) return;   // refuse garbage
    if (requested < 0) requested = 0;                       // clamp to the declared range
    if (requested > 0xFFFFFF) requested = 0xFFFFFF;
    currentColour = (uint32_t)requested;
    dotStarSetColour(currentColour);
    ackIsPending = true;                                   // replied to from loop()
  }
}

void setup() {
  Serial.begin(115200);                                    // ignored on USB CDC, kept for clarity
  pinMode(PIN_DOTSTAR_DATA, OUTPUT);
  pinMode(PIN_DOTSTAR_CLK, OUTPUT);
  dotStarSetColour(0);                                     // start dark
  temperatureBegin();
  upline.onEntry(uplineOnEntry);
}

void loop() {
  // Reads commands, answers "?" with the schema, and heartbeats every 2.5 s.
  upline.poll();

  // Echo the colour as soon as it is set, rather than making a host wait up to
  // two seconds to see it. Flagged in the handler and sent from here, not sent
  // from the handler directly: that runs while the parser is still walking the
  // record, and a device stays easier to reason about when transmitting happens
  // from one place only. It also makes the clamp above visible — ask for
  // 0x1000000 and the reply says 16777215.
  if (ackIsPending) {
    ackIsPending = false;
    upline.beginRecord();
    upline.addInt("rgb", (int32_t)currentColour);
    upline.endRecord();
  }

  if (millis() - lastReportMillis >= REPORT_INTERVAL_MS) {
    lastReportMillis = millis();
    upline.beginRecord();
    upline.addFixed("tempf", readDieTemperatureFx2(), 2);
    upline.addInt("rgb", (int32_t)currentColour);
    upline.endRecord();
  }
}
