// Upline on any Arduino — the smallest complete device.
//
// Exposes two things over serial, with no board-specific host code:
//
//   count int   readable  a free-running counter, one per second
//   led   bool  writable  the onboard LED
//
// Open the port at 115200, wait for a heartbeat, send ^?^, and the board
// answers with a schema describing both. Then:
//
//   ^led|1^             LED on
//   ^led|0^             LED off
//
// A command is echoed back as soon as it lands, carrying just the key it
// touched, so a host sees its setting take effect without waiting for the next
// tick:
//
//   ^led|1^  ->  ^led|1^
//
// and once a second the board sends:
//
//   ^count|42~led|1^
//
// Everything a device has to do — announce itself, accept commands, report
// state on a heartbeat — is in the fifty lines below. Start here; the
// board-specific examples add sensors, ranges and clamping on top of it.
//
// Licence: CC0 / public domain.

#include <upline.hpp>

// ── Descriptor ───────────────────────────────────────────────────────────────
// Lives in flash, so its length costs no RAM. This is the whole contract: a
// host reads it and knows how to draw the device without a line of code that
// knows anything about this board.
UPLINE_SCHEMA(basicSchema,
  // A readable local id, which spec §8.1 allows for one-offs. Every board
  // flashed with this example shares it — give each unit its own before you
  // ship anything, ideally a UUID v4 as unpadded base64url.
  "uuid|upline-basic-0001"
  "~name|Basic demo"
  "~desc|Counter and LED"
  "~ver|1"
  "~count|int|r"                          // read-only; no min/max, it only counts up
  "~led|bool|rw|0"                        // settable, defaults off
  "~reset|cmd");                          // an action: zero the counter

Upline upline(Serial, basicSchema);

// ── Application state ────────────────────────────────────────────────────────
static const uint16_t REPORT_INTERVAL_MS = 1000;   // inside the 2.5 s heartbeat

static bool     ledIsOn = false;
static int32_t  counter = 0;
static uint32_t lastReportMillis = 0;
static bool     ackIsPending = false;   // a write landed and owes a reply
static bool     resetIsPending = false; // the "reset" command landed

// ── Upline ───────────────────────────────────────────────────────────────────

/**
 * Called once for every key/value pair the host sends.
 * Unknown keys are ignored, which is what keeps a device forward-compatible.
 */
void uplineOnKeyValPair(const char* key, const char* value, bool isFlag) {
  if (isFlag) {                           // a cmd carries no value (spec §8.3)
    if (!strcmp(key, "reset")) {
      counter = 0;
      resetIsPending = true;              // replied to from loop(), not here
    }
    return;
  }
  if (!strcmp(key, "led")) {
    ledIsOn = (value[0] == '1');
    digitalWrite(LED_BUILTIN, ledIsOn ? HIGH : LOW);
    ackIsPending = true;                  // replied to from loop(), not here
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);         // start in the schema's declared state
  upline.onPair(uplineOnKeyValPair);
}

void loop() {
  // Reads commands, answers "?" with the schema, and heartbeats every 2.5 s.
  upline.poll();

  // Echo whatever was just commanded. Flagged in the handler and sent here
  // rather than sent from the handler directly: that runs while the parser is
  // still walking the record, and a device stays much easier to reason about if
  // transmitting only ever happens from one place. On a half-duplex port it is
  // not merely tidier but necessary — see the ATtiny85 example.
  if (ackIsPending) {
    ackIsPending = false;
    upline.beginRecord();
    upline.addBool("led", ledIsOn);
    upline.endRecord();
  }

  // A command has no value of its own to echo, so confirm it with the key it
  // affected (spec §7.1) — here, the counter it just zeroed.
  if (resetIsPending) {
    resetIsPending = false;
    upline.beginRecord();
    upline.addInt("count", counter);
    upline.endRecord();
  }

  if (millis() - lastReportMillis >= REPORT_INTERVAL_MS) {
    lastReportMillis = millis();
    upline.beginRecord();
    upline.addInt("count", counter++);
    upline.addBool("led", ledIsOn);
    upline.endRecord();
  }
}
