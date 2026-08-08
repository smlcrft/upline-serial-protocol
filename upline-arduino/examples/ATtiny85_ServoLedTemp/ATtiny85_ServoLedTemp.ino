// Upline on an ATtiny85 — 8 KB flash, 512 B RAM, no hardware UART.
// 
// Tested by Programming w/ Sparkfun AVR Tiny programmer (USBTinyISP Slow), Board = ATtiny25/45/85(No Bootloader), 16MHz (PLL), BOD Enabled @ 1.8V.
//
// Proof that a self-describing device fits on a part this small. Exposes:
//
//   servo  int   0..160          writable   servo angle, degrees, 80 at boot
//   led    bool  0/1             writable   an LED
//   tempf  fix2  -40.00..200.00  readable   uncalibrated die temperature, °F
//
// The servo is rate limited and then released, both to be kind to a USB 5 V
// rail. Commanding a big jump makes a servo draw close to stall current for the
// whole move, which can brown out the chip and reset it; easing there at a
// fixed degrees-per-second spreads that draw out, so 0 to 160 at the default
// 60 deg/s takes under three seconds instead of one violent lunge. It boots
// commanding 80, the midpoint, so it never drives to an end stop. Three seconds
// after it arrives the pulses stop and the servo goes limp — no holding current
// at all. `servo` reports the angle you last asked for; the eased position on
// its way there is internal, since a released servo can sag and would make any
// reported position a guess anyway.
//
// Open the port at 115200, wait for a heartbeat, send ^?^, and the chip answers
// with a schema describing all three. Then:
//
//   ^servo|90^          move toward 90 degrees, easing at SERVO_SLEW_PER_SECOND
//   ^led|1^             LED on
//   ^servo|0~led|0^     both at once
//
// Every command is echoed immediately with just the keys it touched, so a host
// never waits on the next telemetry tick to see its setting take effect:
//
//   ^servo|0~led|0^  ->  ^servo|0~led|0^
//   ^servo|200^      ->  ^servo|80^        refused, still at 80
//
// An out-of-range value is echoed with the setting still in force, which is how
// a host tells a rejected command from a lost one. Costs about 1 ms of airtime.
//
// and every two seconds it sends something like:
//
//   ^tempf|72.50~servo|90~led|0^
//
// ── WIRING ───────────────────────────────────────────────────────────────────
//
//         ATtiny85 (DIP-8)
//       ┌───────∪───────┐
//  PB5  │ 1           8 │  VCC
//  PB3  │ 2           7 │  PB2
//  PB4  │ 3           6 │  PB1
//  GND  │ 4           5 │  PB0
//       └───────────────┘
//
//   pin 4  GND
//   pin 5  PB0  serial TX  ->  host RX
//   pin 6  PB1  serial RX  <-  host TX
//   pin 2  PB3  LED -> resistor -> GND
//   pin 3  PB4  servo signal
//   pin 7  PB2  free
//   pin 1  PB5  RESET, unusable unless fuses are changed
//   pin 8  VCC
//
// GND, TX and RX land on pins 4-5-6, so the three wires a serial cable needs
// for signalling come off the chip together. VCC is on pin 8, diagonally
// opposite GND — that is the DIP-8 pinout, not a choice, so power and ground
// can never both sit beside the serial pair.
//
// The serial pins are not movable. ATTinyCore's Serial is bit-banged using the
// analog comparator, and RX is hardwired to its AIN1 input, which is PB1. A
// SoftwareSerial instance on other pins would work — this library is a template
// over the port type, so UplineDevice<SoftwareSerial> is a one-line change —
// but it costs more RAM and blocks interrupts for a whole byte while sending,
// which jitters the servo. The built-in Serial is the better trade here.
//
// Run the chip at 16 MHz from the internal PLL, at 115200 baud. Both halves of
// that matter: 115200 is unreachable at 8 MHz (the bit period would be 63
// cycles against an ideal 69.4, a 9.3% error), and 9600 is slow enough that
// sending the descriptor blanks interrupts for 183 ms — nine dead servo frames
// every time a host asks ^?^. At 115200 that is 15 ms. See the servo section.
//
// ── BURN THE FUSES FIRST ─────────────────────────────────────────────────────
//
// A factory ATtiny85 has CKDIV8 programmed, so it runs at 1 MHz — the 8 MHz
// oscillator divided by eight. The Clock menu is labelled "Only set on
// bootload" for a reason: it sets F_CPU for the compiler, but uploading a
// sketch never writes fuses. Get this wrong and everything runs 16x slow.
//
//   Board       ATtiny25/45/85 (No Bootloader)
//   Chip        ATtiny85
//   Clock       16 MHz (PLL)          <- low fuse 0xF1, PLL as system clock
//   B.O.D.      see below             <- depends on how the servo is powered
//   Programmer  USBtinyISP (slow)     <- "slow" is required at 1 MHz
//
// Then Tools -> Burn Bootloader, which writes the fuses and erases the chip,
// and re-upload the sketch. Plain "USBtinyISP" is fine once it is at 16 MHz.
//
// BOD is a real trade here, not a default to accept. The ATtiny85 is rated for
// 16 MHz only between 4.5 and 5.5 V; below that its ceiling is 10 MHz. So:
//
//   4.3 V  Correct for 16 MHz — a dip out of spec resets cleanly instead of
//          executing unpredictably. But a servo sharing the 5 V rail routinely
//          drags it under 4.3 V while travelling, and the chip then resets
//          mid-move: state returns to zero, and small commands appear to work
//          while large ones never arrive. Only usable with the servo on its own
//          supply, or with enough bulk capacitance to hold the rail up.
//
//   1.8 V  Rides through those dips, at the cost of the guarantee. Undervolted
//          at 16 MHz the failure mode is not a reset but arbitrary
//          misbehaviour. Fine on a bench, and the pragmatic choice when the
//          servo shares the rail — just know which protection you gave up.
//
// Either way, do not let the servo stall against a mechanical end stop: it
// draws near maximum current continuously, and that is the one load most likely
// to pull the rail down far enough to matter. See SERVO_PULSE_MIN_US.
//
// Licence: CC0 / public domain.

#define UPLINE_ENABLE_BASE64 0        // nothing here sends binary
#include <upline.hpp>

// ── Pins ─────────────────────────────────────────────────────────────────────
static const uint8_t LED_PIN   = 3;   // PB3
static const uint8_t SERVO_PIN = 4;   // PB4

// ── Oscillator trim ──────────────────────────────────────────────────────────
// Cancels the +2.80% bit-period error described in the header by running the
// clock that much slow. OSCCAL trims the 8 MHz RC oscillator, and the datasheet
// is explicit that the PLL is locked to it, so a step here moves the 16 MHz
// system clock with it.
//
// One step is roughly 1% near the factory value, so -3 is the right place to
// start. Hardcoded per chip, like the temperature offset below: this belongs to
// one physical part, and 0 leaves the chip untrimmed.
//
// Note what else moves with it. delayMicroseconds and millis() are both derived
// from F_CPU, which the compiler still believes is exactly 16000000, so a -3
// trim stretches every servo pulse by about 3% — near 45 us, or 5 degrees of
// travel — and makes millis() run about 3% fast. Neither matters much here
// (hobby servos vary more than that between units, and the heartbeat's 3x stale
// window swallows the clock drift), but trim the servo endpoints after the
// oscillator, never before.
static const int8_t OSCILLATOR_TRIM_STEPS = -3;

/** Apply OSCILLATOR_TRIM_STEPS to OSCCAL. Call before Serial.begin. */
static void applyOscillatorTrim() {
  // One step at a time: the datasheet warns that a large single jump in
  // frequency can upset the core, and asks for gradual changes.
  int8_t remaining = OSCILLATOR_TRIM_STEPS;
  while (remaining < 0) { if (OSCCAL > 0)   OSCCAL--; remaining++; }
  while (remaining > 0) { if (OSCCAL < 255) OSCCAL++; remaining--; }
}

// ── Descriptor ───────────────────────────────────────────────────────────────
// Lives in flash, so its length costs no RAM. min/max is what lets a host draw
// a 0-160 slider and a toggle without knowing anything about this board.
UPLINE_SCHEMA(tinySchema,
  // A readable local id, which spec §8.1 allows for one-offs. Give each unit
  // its own before shipping anything — ideally a UUID v4 as base64url.
  "uuid|attiny85-demo"
  "~name|ATtiny85 node"
  "~desc|Servo, LED, die temperature"
  "~ver|1"
  "~servo|int|rw|80|0|160"
  "~led|bool|rw|0"
  "~tempf|fix2|r||-40.00|200.00");

Upline upline(Serial, tinySchema);

// ── Application state ────────────────────────────────────────────────────────
static const uint16_t SERVO_FRAME_MS = 20;      // servos want a pulse this often

// How long to keep driving after arriving, before letting the servo go limp —
// long enough to settle, short enough that idle current drops to nothing.
static const uint16_t SERVO_HOLD_MS = 3000;

// How often to send an unsolicited telemetry record. Comfortably inside the
// 2500 ms heartbeat, so poll() never needs to send a bare heartbeat of its own.
static const uint16_t REPORT_INTERVAL_MS = 2000;

// Travel-rate limit. Fixed rather than exposed as a key: it is a property of
// what the servo is bolted to, not something a host should be free to raise —
// the whole point is that it bounds peak current draw.
static const uint16_t SERVO_SLEW_PER_SECOND = 60;

// Pulse widths at 0 and at 160 degrees, and the one pair worth measuring per
// servo. 1000-2000 us is the cautious legacy range and on most hobby servos
// covers only the middle of the mechanical travel. Full travel
// usually wants something nearer 500-2500 us.
//
// Widen these carefully and one end at a time. Commanding past the mechanical
// stop does not fail visibly: the servo simply stalls against it, drawing close
// to its maximum current for as long as it is driven, which is both a brownout
// risk and hard on the gears. Widen until the travel is right, then back off.
//
// Trim these after OSCILLATOR_TRIM_STEPS, never before — the oscillator trim
// scales every pulse by the same percentage it scales the clock.
static const uint16_t SERVO_PULSE_MIN_US = 700;    // command 0
static const uint16_t SERVO_PULSE_MAX_US = 2300;   // command 160

// Boot position, matching the schema's declared default so a host that has not
// yet talked to us still draws the slider where the servo actually is.
static const uint8_t SERVO_BOOT_DEGREES = 80;   // the midpoint of 0..160

// Position starts AT the target rather than at zero, which matters. Starting at
// zero would make the very first pulse 700 us and slam the horn into the 0
// degree end stop before the slew limiter eased it back to 80 — the one move
// this sketch exists to avoid. Beginning already parked at the midpoint means
// the first pulse is 1500 us, so the only uncontrolled travel is whatever it
// takes to get from wherever the horn was left to the middle, which is
// unavoidable on power-up since an unpowered servo cannot report its position.
static uint8_t  servoTargetDegrees = SERVO_BOOT_DEGREES;
static int16_t  servoPositionFx2   = (int16_t)SERVO_BOOT_DEGREES * 100;
static bool     servoHasArrived = false;        // position reached target
static bool     servoIsDriven   = true;        // position reached target
static uint32_t servoArrivedAtMillis = 0;       // and when that happened
static bool     ledIsOn = false;
static uint32_t lastServoPulseMillis = 0;
static uint32_t lastReportMillis = 0;

// Which keys a command touched and still owe an acknowledgement. A bitmask
// rather than a flag per key so that a multi-key record like ^servo|0~led|0^
// coalesces into one reply instead of two — one 18-byte record costs less
// airtime than two records of 12 and 8, and blanks interrupts once.
static const uint8_t ACK_SERVO = 0x01;
static const uint8_t ACK_LED   = 0x02;
static uint8_t pendingAckMask = 0;

// ── Servo ────────────────────────────────────────────────────────────────────

/**
 * Step the position toward the target, then emit one pulse — unless we have
 * been parked long enough to release the servo. Call every frame.
 *
 * Position is tracked in hundredths of a degree because a whole degree is too
 * coarse a step: at 60 deg/s and a 20 ms frame the servo should move 1.2°, and
 * rounding that to 1 or 2 would make the rate wrong by a fifth. Integer maths
 * throughout — no float on this chip.
 *
 * @param now millis() as read by the caller, so the hold is timed against the
 *            same instant that scheduled this frame
 */
static void servoAdvanceAndPulse(uint32_t now) {
  const int16_t targetFx2 = (int16_t)servoTargetDegrees * 100;
  // Degrees per second -> hundredths of a degree per frame.
  // 60 deg/s * 20 ms / 10 = 120 hundredths = 1.2° per frame.
  const int16_t stepFx2 =
      (int16_t)(((uint32_t)SERVO_SLEW_PER_SECOND * SERVO_FRAME_MS) / 10u);
  int16_t remaining = targetFx2 - servoPositionFx2;
  if (remaining >  stepFx2) remaining =  stepFx2;      // ease, do not lunge
  if (remaining < -stepFx2) remaining = -stepFx2;
  servoPositionFx2 += remaining;
  if (servoPositionFx2 == targetFx2) {
    // Arrived. Keep driving briefly so it can settle, then stop pulsing
    // entirely: with no pulse an analogue hobby servo releases and draws
    // nothing. A new target clears the flag below and driving resumes on the
    // next frame. (A digital servo latches its last command and holds torque
    // regardless — that is the servo's design, not something this can undo.)
    if (!servoHasArrived) {
      servoHasArrived = true;
      servoArrivedAtMillis = now;
    }
    if (now - servoArrivedAtMillis >= SERVO_HOLD_MS){
      servoIsDriven = false; // servo no longer being driven.
      return;
    }
  } else {
    servoIsDriven = true;
    servoHasArrived = false;
  }
  // Map 0.00-160.00° onto SERVO_PULSE_MIN_US..SERVO_PULSE_MAX_US. The multiply
  // is widened to 32 bits deliberately: 16000 * 1600 is over 25 million and
  // would wrap a uint16_t four hundred times over. Both endpoints come out
  // exact, so the pulse can never leave the declared range.
  const uint16_t pulseMicros = SERVO_PULSE_MIN_US +
      (uint16_t)(((uint32_t)servoPositionFx2 *
                  (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / 16000u);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseMicros);
  digitalWrite(SERVO_PIN, LOW);
}

// ── Die temperature ──────────────────────────────────────────────────────────
// The ATtiny85 has an on-chip sensor on ADC channel 15, read against the
// internal 1.1 V reference. It is uncalibrated: the datasheet gives only
// typical readings of 230 / 300 / 370 LSB at -40 / +25 / +85 °C, and warns the
// part-to-part spread is wide. Interpolating the -40 and +85 endpoints gives
// 125 °C across 140 LSB, which is the slope below.
//
#define TEMP_SAMPLE_COUNT 4

// Calibration trim, in hundredths of a degree F. Hardcoded rather than exposed
// as a key: it belongs to this one chip, and it is not something a host has any
// business setting. To find yours, let the board idle at a known room
// temperature and subtract the reported reading from it. Expect it to
// over-correct once the chip is working, because part of what you trim away is
// real die heating rather than sensor error.
static const int32_t DEGREE_F_OFFSET_FX2 = 3100;  // +31.00 F, see note below

/**
 * Read the die temperature.
 * @return hundredths of a degree Fahrenheit, e.g. 7250 for 72.50 °F
 */
static int32_t readDieTemperatureFx2() {
  analogReference(INTERNAL1V1);
  analogRead(ADC_TEMPERATURE);                  // discard: the reference just changed

  // 10-bit conversions cap at 1023, so four of them cannot overflow a uint16_t.
  uint16_t total = 0;
  for (uint8_t sample = 0; sample < TEMP_SAMPLE_COUNT; ++sample) {
    total += analogRead(ADC_TEMPERATURE);
  }
  const int32_t raw = total / TEMP_SAMPLE_COUNT;

  // Note the int32_t cast: on AVR a plain int is 16 bits and (raw-230)*12500
  // would overflow long before it got here.
  const int32_t celsiusFx2 = -4000 + ((raw - 230) * 12500) / 140;
  int32_t fahrenheitFx2 = (celsiusFx2 * 9) / 5 + 3200 + DEGREE_F_OFFSET_FX2;

  // Never emit a value outside the range the schema promises.
  if (fahrenheitFx2 < -4000)  fahrenheitFx2 = -4000;
  if (fahrenheitFx2 > 20000)  fahrenheitFx2 = 20000;
  return fahrenheitFx2;
}

// ── Upline ───────────────────────────────────────────────────────────────────

/**
 * Called once for every key/value pair the host sends.
 * Unknown keys are ignored, which is what keeps a device forward-compatible.
 */
void uplineOnKeyValPair(const char* key, const char* value, bool isFlag) {
  if (isFlag) return;                           // no flags defined here
  if (!strcmp(key, "servo")) {
    const long requested = atol(value);
    if (requested >= 0 && requested <= 160) {
      servoTargetDegrees = (uint8_t)requested;
    }
    // Acknowledge even when the value was refused. The reply carries the value
    // actually in force, so a host that asked for 200 and gets 80 back learns
    // its command was rejected rather than lost — silence could mean either.
    pendingAckMask |= ACK_SERVO;
  } else if (!strcmp(key, "led")) {
    ledIsOn = (value[0] == '1');
    digitalWrite(LED_PIN, ledIsOn ? HIGH : LOW);
    pendingAckMask |= ACK_LED;
  }
}

/**
 * Reply with just the keys a command touched, if any.
 *
 * Deliberately not sent from inside uplineOnKeyValPair: that runs while the
 * parser is still walking the record, and write() holds interrupts off for a
 * whole byte, so acknowledging there would go half-duplex mid-record and could
 * swallow bytes of whatever the host sent next. Called from loop() instead,
 * once the record is fully consumed.
 *
 * This also serves as the heartbeat, since endRecord() restarts that timer.
 */
static void sendPendingAck() {
  if (!pendingAckMask) return;
  upline.beginRecord();
  if (pendingAckMask & ACK_SERVO) upline.addInt("servo", servoTargetDegrees);
  if (pendingAckMask & ACK_LED)   upline.addBool("led", ledIsOn);
  upline.endRecord();
  pendingAckMask = 0;
}

void setup() {
  applyOscillatorTrim();                        // before begin() reads the clock
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(SERVO_PIN, LOW);
  upline.onPair(uplineOnKeyValPair);
}

void loop() {
  // Reads commands, answers "?" with the schema, heartbeats every 2.5 s.
  upline.poll();
  sendPendingAck();                             // echo anything just commanded
  const uint32_t now = millis();
  if (now - lastServoPulseMillis >= SERVO_FRAME_MS) {
    lastServoPulseMillis = now;
    servoAdvanceAndPulse(now);
  }
  if (now - lastReportMillis >= REPORT_INTERVAL_MS) {
    lastReportMillis = now;
    // `servo` echoes the angle last commanded, so a host always sees its own
    // setting reflected back. The eased position on the way there stays
    // internal — see the note in the header.
    upline.beginRecord();
    if(!servoIsDriven){ // don't measure while servo is being driven. Adds lots of analog noise.
      upline.addFixed("tempf", readDieTemperatureFx2(), 2);
    }
    upline.addInt("servo", servoTargetDegrees);
    upline.addBool("led", ledIsOn);
    upline.endRecord();
  }
}
