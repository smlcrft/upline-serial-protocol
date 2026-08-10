// upline.hpp — Upline serial protocol, single-header C++ implementation.
//
//   ^temp~23.45^rh~48^fan~1^
//
// A newline-delimited ASCII key/value protocol for microcontrollers: publish
// state, accept commands, and describe yourself, over UART / USB / BT-Serial.
// The wire format is specified at github.com/smlcrft/upline-serial-protocol.
// Header-only, no allocation, no libc, no floating point. Drop this file next
// to your sketch and include it.
//
// ── QUICK START ──────────────────────────────────────────────────────────────
//
//   #include "upline.hpp"
//
//   // 1. Declare what this device is and what keys it has. Lives in flash,
//   //    not RAM. Write only your own values: the framing, _v and _r are
//   //    filled in for you, so the buffer size can never drift out of sync.
//   //    uuid / name / desc / ver are required; the rest are yours.
//   UPLINE_SCHEMA(mySchema,
//     "_i|n4o8IUt-TQWzph4vfI2Qqw"        // unique per device
//     "~_n|Greenhouse"
//     "~_d|North bed sensors"
//     "~temp|r|fix2||-40.00|125.00"       // 2-decimal sensor, range -40..125
//     "~fan|rw|bool|0");                  // on/off, settable, defaults off
//
//   Upline upline(Serial, mySchema);
//
//   // 2. Handle anything the host sends you. Optional.
//   void uplineOnEntry(const UplineEntry& entry) {
//     if (entry.isRead()) return;         // ^fan^ asks for the value, does not set it
//     if (!strcmp(entry.key, "fan")) digitalWrite(FAN_PIN, entry.value(0)[0] == '1');
//   }
//
//   void setup() {
//     Serial.begin(115200);               // 9600 if your clock is an internal RC
//     upline.onEntry(uplineOnEntry);
//   }
//
//   void loop() {
//     // 3. poll() does three jobs: reads incoming records and calls your
//     //    handler, answers the host's "?" with your schema automatically,
//     //    and emits a "^^" heartbeat every 2.5 s so the host knows you are
//     //    alive. Call it every time through loop() and never block for long
//     //    — the serial buffer holds only ~5 ms of data at 115200 baud.
//     upline.poll();
//
//     // 4. Send whatever you like, whenever you like. Sending also resets the
//     //    heartbeat timer, so a device already streaming never adds "^^".
//     if (timeForAReading()) {
//       upline.beginRecord();
//       upline.addFixed("temp", degreesTimes100(), 2);   // 2345 -> "23.45"
//       upline.addBool("fan", fanIsOn());
//       upline.endRecord();
//     }
//   }
//
// ── SIZE ─────────────────────────────────────────────────────────────────────
//
// Arduino builds with -ffunction-sections -Wl,--gc-sections, and everything
// here is inline, so anything you never call is never emitted. You do not have
// to disable features you are not using — the linker already did.
//
// Measured on the bundled examples, which are what you can reproduce:
//
//   Basic                    Uno / ATmega328P    4060 B / 358 B
//   ATtiny85_ServoLedTemp    ATtiny85            4426 B / 262 B
//   ItsyBitsyM4_RgbAndTemp   ItsyBitsy M4       13168 B
//
// The knobs that genuinely change the build are UPLINE_TRANSMIT_ONLY, which
// removes the parser and the receive buffer entirely, UPLINE_RX_BUFFER_SIZE,
// which is pure RAM, and UPLINE_MAX_VALUES / UPLINE_MAX_SUBVALUES, which bound
// how complex a single inbound entry may be. UPLINE_ENABLE_FIXED and
// UPLINE_ENABLE_BASE64 save nothing on a normal Arduino build; they exist as a
// guardrail, so a project with a hard size ceiling gets a compile error instead
// of silently linking a feature, and for toolchains that do not garbage-collect
// unused sections.
//
// ── PLATFORMS ────────────────────────────────────────────────────────────────
//
// No per-chip special cases are needed — the Arduino abstraction covers it.
// Compiled clean on every core below with this exact header:
//
//   AVR      Uno / ATmega328P, Duemilanove / ATmega168, ATtiny85, ATtiny1614
//   SAMD     MKRZero (M0+), Feather M4 (M4)
//   RP2040   Pico, Pico W                          (Cortex-M0+)
//   RP2350   Pico 2, Pico 2 W  — ARM M33 and RISC-V both
//   ESP32    ESP32 (Xtensa), ESP32-S3, ESP32-C3 (RISC-V)
//   Teensy   4.1 (Cortex-M7)
//
// Off Arduino, supply a port with available() / read() / write(uint8_t) and
// #define UPLINE_MILLIS() to your clock — see UplineDevice below.
//
// Licence: public domain / CC0. Use it however you like.

#ifndef UPLINE_HPP
#define UPLINE_HPP

#include <stdint.h>
#include <stddef.h>

// ─────────────────────────────────────────────────────────────────────────────
// Configuration — #define any of these BEFORE including this header.
// ─────────────────────────────────────────────────────────────────────────────

// Largest inbound record the device can receive, in bytes. This limits only
// what the device READS; records it sends may be any length (spec §10).
//
// 128 is the floor the spec requires, and it is deliberately above the 120-byte
// host-to-device recommendation so a conforming host can never overflow it.
// Lowering it below 128 makes the device non-conforming — do that only if you
// also control the host and know its commands are shorter.
/** Upline version this library speaks. Emitted as `_v` (spec §7). */
#define UPLINE_VERSION 1

#ifndef UPLINE_RX_BUFFER_SIZE
#define UPLINE_RX_BUFFER_SIZE 128
#endif

// Milliseconds between heartbeats. Any record the device sends resets the
// timer, so a device already streaming telemetry never emits a bare heartbeat.
#ifndef UPLINE_HEARTBEAT_MS
#define UPLINE_HEARTBEAT_MS 2500
#endif

// Set to 0 to omit fixed-point ("fixN") support. On a normal Arduino build this
// saves nothing — unused inline code is never emitted — so it is a guardrail
// rather than an optimisation: calling addFixed with this off is a compile
// error instead of a silent 200 bytes. Useful on a hard size ceiling, and on
// toolchains that do not garbage-collect unused sections.
#ifndef UPLINE_ENABLE_FIXED
#define UPLINE_ENABLE_FIXED 1
#endif

// Set to 0 to omit base64url ("b64") support. Same guardrail semantics as
// UPLINE_ENABLE_FIXED above — no saving unless you would otherwise have called
// addBase64 or uplineDecodeBase64.
#ifndef UPLINE_ENABLE_BASE64
#define UPLINE_ENABLE_BASE64 1
#endif

// Set to 1 for a device with no receive path — only a TX pin wired, a one-way
// link, or a sensor that must never be commandable. Drops the receive buffer,
// the line reader, and the parser.
//
// This is a conforming profile, not a crippled device (spec §9): since such a
// device can never answer "?", poll() broadcasts the "!" schema unprompted
// instead, so a host still discovers it. Everything else is unchanged.
#ifndef UPLINE_TRANSMIT_ONLY
#define UPLINE_TRANSMIT_ONLY 0
#endif

// How often a transmit-only device broadcasts its schema. Kept slower than the
// heartbeat because a large schema is real bandwidth: 200 bytes every 2.5 s is
// 8% of a 9600 baud link, a kilobyte over 40%. The spec asks for at most 5000.
#ifndef UPLINE_SCHEMA_BROADCAST_MS
#define UPLINE_SCHEMA_BROADCAST_MS 5000
#endif

// Internal: receive support is simply the inverse of the transmit-only profile.
#define UPLINE_ENABLE_RECEIVE (!UPLINE_TRANSMIT_ONLY)

// Turn a macro's *value* into a string literal, so the schema can carry numbers
// that are defined elsewhere without anyone retyping them.
#define UPLINE_STRINGIFY_(x) #x
#define UPLINE_STRINGIFY(x) UPLINE_STRINGIFY_(x)

// What `_r` advertises. A transmit-only build has no receive path at all, and
// spec §10 spells that as 0 — the value a host reads to learn it must never send.
#if UPLINE_TRANSMIT_ONLY
#define UPLINE_DECLARED_RX 0
#else
#define UPLINE_DECLARED_RX UPLINE_RX_BUFFER_SIZE
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Platform glue
// ─────────────────────────────────────────────────────────────────────────────

// Schema strings live in flash, not RAM.
//
// Every Arduino core provides PROGMEM and pgm_read_byte, so this needs no
// per-chip special cases. On AVR they do real work. On ESP8266 they are
// required for a different reason: PROGMEM data lands in IROM, which must be
// read 32-bit aligned, and without PROGMEM a const array would sit in RAM. On
// ESP32, SAMD, RP2040, Teensy and the rest, the core defines PROGMEM as empty
// and pgm_read_byte as a plain dereference, so using them costs nothing.
//
// Off Arduino entirely, fall back to a plain pointer read.
#if defined(ARDUINO)
#include <Arduino.h>
#define UPLINE_FLASH_STORAGE PROGMEM
#define UPLINE_READ_FLASH_BYTE(pointer) pgm_read_byte(pointer)
#else
#define UPLINE_FLASH_STORAGE
#define UPLINE_READ_FLASH_BYTE(pointer) (*(pointer))
#endif

// Millisecond clock, used only for the heartbeat and schema timers.
//
// Off Arduino you MUST define this, or the fallback below leaves the clock
// frozen at zero and poll() emits a heartbeat on every single call. The free
// parsing functions do not use it, so a project that only calls
// uplineParseLine can ignore this entirely.
#ifndef UPLINE_MILLIS
#if defined(ARDUINO)
#define UPLINE_MILLIS() millis()
#else
#define UPLINE_MILLIS() 0
#endif
#endif

/**
 * Declare a device descriptor, stored in flash rather than RAM.
 *
 * Supply only the key/value pairs — the leading `^!~` and trailing `^` are
 * added for you. Adjacent string literals concatenate, so a long schema can be
 * split across lines for readability at no cost.
 *
 * Supply `_i`, `_n`, and `_d` yourself (spec §7). `_v` and `_r` are added
 * automatically from UPLINE_VERSION and the receive buffer — a transmit-only
 * build declares `_r|0` on its own, so a host is never told to send to a device
 * that cannot listen.
 *
 * A data key declaration is `name|mode|type|default|min|max`, every sub-value
 * after the mode optional. Mode is `r` (the default, so a sensor may leave it
 * empty), `rw`, `w`, `x` for an executable, or `d` for a declaration-only
 * parameter; `min` and `max` are independent (spec §8.1). An `x` key names an
 * action the host invokes as `^key^`, and any sub-values after the mode are the
 * names of the keys it takes as arguments.
 *
 *   UPLINE_SCHEMA(mySchema,
 *     "uuid|n4o8IUt-TQWzph4vfI2Qqw"
 *     "~name|Greenhouse"
 *     "~desc|North bed sensors"
 *     "~ver|1"
 *     "~temp|fix2|r||-40.00|125.00"
 *     "~fan|bool|rw|0"
 *     "~beep|cmd");
 *
 * @param name      identifier to declare, later passed to the Upline constructor
 * @param ...       the descriptor body, as one or more string literals
 */
#define UPLINE_SCHEMA(name, ...) \
  static const char name[] UPLINE_FLASH_STORAGE =                              \
    "^?~_v|" UPLINE_STRINGIFY(UPLINE_VERSION)                                    \
    "~_r|"   UPLINE_STRINGIFY(UPLINE_DECLARED_RX)                                \
    "~" __VA_ARGS__ "^"

// ─────────────────────────────────────────────────────────────────────────────
// Parse results
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Outcome of parsing one line. Per-entry problems are not reported here: a
 * malformed entry is dropped on its own and the rest of the line still applies
 * (spec §3), so a line carrying one bad entry among good ones is UplineOk.
 */
enum UplineResult {
  UplineOk = 0,            ///< Line parsed; every fully bounded entry was dispatched.
  UplineNotUpline = -1,    ///< Line did not start with '^'. Ignore it, not an error.
  UplineTruncated = -2     ///< Line did not end in an unescaped '^'. The trailing
                           ///< fragment was dropped; entries before it WERE dispatched.
};

/**
 * Most values a single entry may carry, and most sub-values across all of them
 * (spec §5). An entry exceeding either is dropped rather than truncated, so this
 * bounds what the device can *receive*, exactly as UPLINE_RX_BUFFER_SIZE does.
 * Both live on the stack for the duration of one parse.
 */
#ifndef UPLINE_MAX_VALUES
#define UPLINE_MAX_VALUES 6
#endif
#ifndef UPLINE_MAX_SUBVALUES
#define UPLINE_MAX_SUBVALUES 10
#endif

/**
 * One decoded entry: a key and its values, each of which is one or more
 * sub-values (spec §5). Everything points into the caller's line buffer and
 * stays valid only until the next line is read.
 *
 *   ^temp^            key "temp", valueCount 0        -- a read, or an execute
 *   ^temp~23.45^      key "temp", value(0) "23.45"
 *   ^temp~^           key "temp", value(0) ""         -- a write of the empty string
 *   ^rgb~1|2|3^       key "rgb",  sub(0,0..2) "1","2","3"
 *   ^beep~440~0.5^    key "beep", value(0) "440", value(1) "0.5"
 */
struct UplineEntry {
  const char* key;         ///< Unescaped; never empty.
  uint8_t valueCount;      ///< 0 for `^key^`.

  /** True for `^key^` — no values at all. A read, or an execute with no arguments. */
  bool isRead() const { return valueCount == 0; }

  /** First sub-value of value `index`, or "" when there is none. The common
      case: a scalar value is simply a single sub-value. */
  const char* value(uint8_t index) const { return sub(index, 0); }

  /** Sub-value `subIndex` of value `index`, or "" when out of range. */
  const char* sub(uint8_t index, uint8_t subIndex) const {
    if (index >= valueCount || subIndex >= subCounts[index]) return "";
    uint8_t flat = subIndex;
    for (uint8_t v = 0; v < index; ++v) flat = (uint8_t)(flat + subCounts[v]);
    return subs[flat];
  }

  /** How many sub-values value `index` carries. One for a scalar. */
  uint8_t subCount(uint8_t index) const {
    return index < valueCount ? subCounts[index] : 0;
  }

  uint8_t subCounts[UPLINE_MAX_VALUES];
  char*   subs[UPLINE_MAX_SUBVALUES];   ///< flat, in wire order
  uint8_t subTotal;
};

/** Callback for each entry, as used by the UplineDevice class. */
typedef void (*UplineEntryHandler)(const UplineEntry& entry);

/**
 * Callback for each entry, as used by uplineParseLine directly. `context` is
 * whatever you passed in, so several parsers can run independently.
 */
typedef void (*UplineEntryHandlerWithContext)(void* context, const UplineEntry& entry);

// ─────────────────────────────────────────────────────────────────────────────
// Core codec — free functions, usable without the Upline class
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Decode backslash escapes in place. The result is always the same length or
 * shorter, so it safely overwrites its input.
 *
 * Recognises \\  \^  \|  \~  \n  \r  (spec §4). The trailing-backslash guard
 * keeps this safe even on text that has not been validated.
 *
 * @param text  NUL-terminated string, modified in place
 */
inline void uplineUnescapeInPlace(char* text) {
  char* write = text;
  for (char* read = text; *read; ++read) {
    if (*read == '\\' && read[1]) {
      switch (*++read) {
        case 'n': *write++ = '\n'; break;
        case 'r': *write++ = '\r'; break;
        default:  *write++ = *read; break;   // \\  \^  \|  \~
      }
    } else {
      *write++ = *read;
    }
  }
  *write = '\0';
}

/**
 * True for a '_'-prefixed key this specification does not define. Such a key is
 * ignored rather than dispatched, so a future version of the protocol can add a
 * field without breaking this parser (spec §7).
 *
 * The six defined ones are all two characters: _i uuid, _n name, _d description,
 * _v version, _r receive buffer size, _e error.
 *
 * Checking before unescaping is safe: a key cannot acquire a leading '_' that
 * way, because "\_" is an undefined escape and the scan rejects it first.
 */
inline bool uplineIsKeyUnknownReserved(const char* key) {
  if (*key != '_') return false;
  if (!key[1] || key[2]) return true;                  // "_" alone, or longer than two
  switch (key[1]) {
    case 'i': case 'n': case 'd': case 'v': case 'r': case 'e': return false;
    default: return true;
  }
}

/**
 * Parse one complete line and dispatch each entry.
 *
 * The line must already have its terminator removed — see UplineDevice for the
 * reader that does that. The buffer is modified in place: separators become NUL
 * terminators and escapes are decoded, so everything the entry points at stays
 * valid only until the next line is read.
 *
 * An entry is the text between two unescaped carets and **both must be seen**
 * before it is dispatched (spec §3): the leading caret proves it began where we
 * think it did, the trailing caret proves it arrived whole. So a line missing
 * its opening caret is ignored entirely, and a trailing fragment is dropped
 * without disturbing the complete entries ahead of it.
 *
 * Entries are dispatched as they are scanned, and each is self-contained — no
 * entry's meaning depends on another, so partial application is not a hazard the
 * way it was when a record's first pair scoped the rest.
 *
 * @param line     one line, terminator stripped; modified in place
 * @param length   bytes in `line`, not counting the NUL
 * @param handler  called once per complete, well-formed entry, in wire order
 * @param context  passed straight through to the handler; may be NULL
 */
inline UplineResult uplineParseLine(char* line, size_t length,
                                    UplineEntryHandlerWithContext handler,
                                    void* context) {
  // A line that does not open with '^' is not ours — foreign traffic or debug
  // output sharing the same port. Silently ignored, never an error (spec §3).
  if (length < 1 || *line != '^') return UplineNotUpline;

  UplineEntry entry;
  char* end = line + length;
  char* key = line + 1;
  bool valid = true;                 // current entry survived validation
  entry.valueCount = 0;
  entry.subTotal = 0;

  for (char* cursor = key; cursor < end; ++cursor) {
    // Skip over a valid escape so an escaped delimiter is not mistaken for a
    // real one. This is why a plain strtok cannot parse Upline.
    if (*cursor == '\\') {
      if (cursor + 1 >= end) break;                      // dangling: fragment, dropped
      switch (cursor[1]) {
        case '\\': case '^': case '|': case '~': case 'n': case 'r':
          ++cursor;
          continue;
        default:
          valid = false;                                 // undefined escape (spec §4)
          continue;
      }
    }

    if (*cursor == '~') {                                // next value
      *cursor = '\0';
      if (entry.valueCount < UPLINE_MAX_VALUES && entry.subTotal < UPLINE_MAX_SUBVALUES) {
        entry.subs[entry.subTotal++] = cursor + 1;
        entry.subCounts[entry.valueCount++] = 1;
      } else {
        valid = false;                                   // more than this build accepts
      }
      continue;
    }

    if (*cursor == '|') {                                // next sub-value
      *cursor = '\0';
      if (entry.valueCount && entry.subTotal < UPLINE_MAX_SUBVALUES) {
        entry.subs[entry.subTotal++] = cursor + 1;
        ++entry.subCounts[entry.valueCount - 1];
      } else {
        valid = false;                                   // a bar in the key, or too many
      }
      continue;
    }

    if (*cursor == '^') {                                // entry complete
      *cursor = '\0';
      if (valid && *key && !uplineIsKeyUnknownReserved(key)) {
        // Unescape after splitting, never before: only unescaped separators
        // divide, so `a\|b` must stay one sub-value (spec §5).
        uplineUnescapeInPlace(key);
        for (uint8_t i = 0; i < entry.subTotal; ++i) uplineUnescapeInPlace(entry.subs[i]);
        entry.key = key;
        handler(context, entry);
      }
      key = cursor + 1;
      entry.valueCount = 0;
      entry.subTotal = 0;
      valid = true;
      if (cursor + 1 == end) return UplineOk;            // the caret closed the line
    }
  }

  // Fell off the end without a closing caret: whatever is left is an incomplete
  // entry. It is dropped, but everything dispatched above stands (spec §3).
  return UplineTruncated;
}

// ─────────────────────────────────────────────────────────────────────────────
// Integer text conversion — always available, used by addInt and addFixed
// ─────────────────────────────────────────────────────────────────────────────

/** Bytes needed for the largest output of uplineFormatScaled, including NUL. */
#define UPLINE_NUMBER_BUFFER_SIZE 14

/**
 * Format a scaled integer as decimal text.
 *
 * With scale 0 this is a plain integer. With scale N it inserts a decimal point
 * N digits from the right, so 2345 at scale 2 becomes "23.45" and 5 at scale 2
 * becomes "0.05". No division by powers of ten and no floating point — the
 * digits are simply placed.
 *
 * @param value   the scaled integer, e.g. 2345 meaning 23.45 at scale 2
 * @param scale   implied fractional digits, 0 to 9
 * @param buffer  at least UPLINE_NUMBER_BUFFER_SIZE bytes
 * @return        buffer, for convenient chaining
 */
inline char* uplineFormatScaled(int32_t value, uint8_t scale, char* buffer) {
  char digits[12];
  uint8_t digitCount = 0;
  char* write = buffer;

  // Negate through unsigned so INT32_MIN does not overflow (that would be
  // undefined behaviour in signed arithmetic).
  uint32_t magnitude = (value < 0) ? (uint32_t)0 - (uint32_t)value : (uint32_t)value;
  if (value < 0) *write++ = '-';

  do {
    digits[digitCount++] = (char)('0' + (magnitude % 10));
    magnitude /= 10;
  } while (magnitude);

  // Pad so there is at least one digit left of the point: 5 at scale 2 -> "0.05"
  while (digitCount <= scale) digits[digitCount++] = '0';

  for (uint8_t index = digitCount; index--; ) {
    *write++ = digits[index];
    if (scale && index == scale) *write++ = '.';
  }
  *write = '\0';
  return buffer;
}

#if UPLINE_ENABLE_FIXED
/**
 * Parse decimal text into a scaled integer — the inverse of uplineFormatScaled,
 * and the replacement for atof, which costs about 1.6 KB on AVR.
 *
 * Accepts a leading sign, a missing integer part (".5"), and a missing fraction
 * ("23" at scale 2 gives 2300). Extra fractional digits round half up, so
 * "23.456" at scale 2 gives 2346.
 *
 * @param text   decimal text, NUL-terminated
 * @param scale  implied fractional digits, 0 to 9
 * @return       the scaled integer
 */
/** Append one digit, refusing anything that would exceed INT32_MAX. */
inline bool uplineAccumulateDigit(uint32_t* value, uint8_t digit) {
  if (*value > 214748364UL) return false;               // *10 would overflow
  *value = *value * 10UL + digit;
  return *value <= 2147483647UL;                        // the digit pushed it over
}

/**
 * Parse decimal text into a scaled integer, refusing anything it cannot fully
 * consume (spec §8.4). `int` is scale 0, so this serves both types.
 *
 * Accepts a leading sign, a missing integer part (".5"), and a short or absent
 * fraction ("23" at scale 2 becomes 2300). Excess fractional digits round half
 * up. Rejects an empty string, a bare sign or dot, any character outside
 * [+-.0-9], anything that does not fit int32_t, and ANY trailing byte — so
 * "2 3" and "1.5e3" both fail.
 *
 * Returning false rather than 0 is the whole point. A noisy link produces lines
 * that frame perfectly but carry a corrupted value, and a parser that quietly
 * yields 0 for "" and for "abc" turns that into a setpoint. Drop the entry.
 *
 * @param text   NUL-terminated decimal text
 * @param scale  implied fractional digits, 0 for a plain integer
 * @param out    receives the scaled value; untouched on failure
 * @return       true if the whole string was a valid number
 */
inline bool uplineParseNumber(const char* text, uint8_t scale, int32_t* out) {
  uint32_t value = 0;
  bool isNegative = false;
  uint8_t digits = 0;

  if (*text == '-') { isNegative = true; ++text; }
  else if (*text == '+') { ++text; }

  while (*text >= '0' && *text <= '9') {
    if (!uplineAccumulateDigit(&value, (uint8_t)(*text++ - '0'))) return false;
    ++digits;
  }

  if (*text == '.') {
    ++text;
    while (scale && *text >= '0' && *text <= '9') {
      if (!uplineAccumulateDigit(&value, (uint8_t)(*text++ - '0'))) return false;
      ++digits;
      --scale;
    }
    if (*text >= '5' && *text <= '9') {                  // round half up
      if (value == 2147483647UL) return false;
      ++value;
    }
    while (*text >= '0' && *text <= '9') ++text;         // discard excess digits
  }

  if (!digits || *text) return false;                    // no digits, or trailing junk
  while (scale--) {                                      // pad a short fraction
    if (!uplineAccumulateDigit(&value, 0)) return false;
  }

  *out = isNegative ? -(int32_t)value : (int32_t)value;
  return true;
}
#endif  // UPLINE_ENABLE_FIXED

#if UPLINE_ENABLE_BASE64
// The base64url alphabet is computed rather than kept in a table. A 64-byte
// table would cost the same flash plus 64 bytes of RAM on AVR unless carefully
// placed in PROGMEM, and this arithmetic is smaller than either.
inline char uplineBase64Char(uint8_t sixBits) {
  if (sixBits < 26) return (char)('A' + sixBits);
  if (sixBits < 52) return (char)('a' + sixBits - 26);
  if (sixBits < 62) return (char)('0' + sixBits - 52);
  return (sixBits == 62) ? '-' : '_';
}

inline int8_t uplineBase64Value(char character) {
  if (character >= 'A' && character <= 'Z') return (int8_t)(character - 'A');
  if (character >= 'a' && character <= 'z') return (int8_t)(character - 'a' + 26);
  if (character >= '0' && character <= '9') return (int8_t)(character - '0' + 52);
  if (character == '-') return 62;
  if (character == '_') return 63;
  return -1;                                            // not a base64url character
}

/** Encoded length, excluding the NUL, for a payload of `byteCount` bytes. */
#define UPLINE_BASE64_LENGTH(byteCount) (((byteCount) * 4 + 2) / 3)

/**
 * Decode unpadded base64url in place, over the value the parser handed you.
 *
 * Output is always shorter than input, so it is safe to overwrite. Upline never
 * uses padding, so '=' is rejected like any other stray character.
 *
 * @param text  base64url text, NUL-terminated; overwritten with raw bytes
 * @return      number of bytes decoded, or 0 if the text was not valid base64url
 */
inline size_t uplineDecodeBase64(char* text) {
  uint8_t* out = (uint8_t*)text;
  uint32_t accumulator = 0;
  int8_t bitsHeld = 0;
  size_t byteCount = 0;

  for (char* read = text; *read; ++read) {
    int8_t sixBits = uplineBase64Value(*read);
    if (sixBits < 0) return 0;                          // invalid character
    accumulator = (accumulator << 6) | (uint32_t)sixBits;
    bitsHeld += 6;
    if (bitsHeld >= 8) {
      bitsHeld -= 8;
      out[byteCount++] = (uint8_t)(accumulator >> bitsHeld);
    }
  }
  return byteCount;
}
#endif  // UPLINE_ENABLE_BASE64

// ─────────────────────────────────────────────────────────────────────────────
// UplineDevice — reads, dispatches, heartbeats, and builds outgoing records
// ─────────────────────────────────────────────────────────────────────────────

/**
 * An Upline endpoint bound to one serial port.
 *
 * `SerialPort` needs three members, which Arduino's Stream already provides:
 *   int  available()        bytes waiting
 *   int  read()             next byte, or -1
 *   void write(uint8_t)     send one byte
 *
 * On Arduino use the `Upline` alias below; elsewhere instantiate this template
 * with your own shim and #define UPLINE_MILLIS() to your clock.
 */
template <class SerialPort>
class UplineDevice {
 public:
  /**
   * @param port    the serial port to read and write
   * @param schema  descriptor declared with UPLINE_SCHEMA, sent in reply to "?"
   */
  UplineDevice(SerialPort& port, const char* schema)
      : port_(port),
        schema_(schema),
        handler_(NULL),
        lastSendMillis_(0),
        lastSchemaMillis_(0),
        recordIsEmpty_(true)
#if UPLINE_ENABLE_RECEIVE
        , schemaRequested_(false),
        isOverlong_(false),
        receiveLength_(0)
#endif
  {}

  /**
   * Register the callback that receives each inbound entry.
   * By convention the callback is named uplineOnEntry, matching this library's
   * free-function prefix and keeping it greppable in a large sketch.
   */
  void onEntry(UplineEntryHandler uplineOnEntry) { handler_ = uplineOnEntry; }

  /**
   * Service the port. Call this every time through loop().
   *
   * Reads whatever has arrived, dispatches complete records, answers "?" with
   * the schema automatically, and emits a heartbeat when the device has been
   * quiet for UPLINE_HEARTBEAT_MS.
   *
   * Must be called often enough to keep the serial buffer from overflowing —
   * roughly every 5 ms at 115200 baud on an AVR, less on parts with a 16-byte
   * buffer. A long delay() or a slow blocking read in the same loop is what
   * breaks this, not the length of a record.
   */
  void poll() {
#if UPLINE_ENABLE_RECEIVE
    while (port_.available() > 0) {
      int incoming = port_.read();
      if (incoming < 0) break;
      feed((char)incoming);
    }
#endif
#if UPLINE_TRANSMIT_ONLY
    // No "?" can ever reach us, so announce ourselves on a timer instead.
    if ((uint32_t)(UPLINE_MILLIS() - lastSchemaMillis_) >= (uint32_t)UPLINE_SCHEMA_BROADCAST_MS) {
      sendSchema();
    }
#endif
    if ((uint32_t)(UPLINE_MILLIS() - lastSendMillis_) >= (uint32_t)UPLINE_HEARTBEAT_MS) {
      sendHeartbeat();
    }
  }

#if UPLINE_ENABLE_RECEIVE
  /**
   * Hand one received byte to the reader. Use this instead of poll() if your
   * bytes arrive from somewhere other than the port, such as an interrupt.
   *
   * Either line terminator ends a record, and a CRLF pair ends one record
   * rather than two because the second byte finds an empty buffer.
   */
  void feed(char incoming) {
    if (incoming == '\n' || incoming == '\r') {
      if (isOverlong_) {                      // finished skipping a long record
        isOverlong_ = false;
      } else {
        receiveBuffer_[receiveLength_] = '\0';
        if (receiveLength_) dispatch(receiveBuffer_, receiveLength_);
      }
      receiveLength_ = 0;
    } else if (isOverlong_) {
      // Still skipping. Discarding the whole record rather than just resetting
      // matters: the tail of a long record can itself begin with '^' and would
      // otherwise be parsed as a spurious second record.
    } else if (receiveLength_ < UPLINE_RX_BUFFER_SIZE - 1) {
      receiveBuffer_[receiveLength_++] = incoming;
    } else {
      isOverlong_ = true;                     // too long for us: skip to the terminator
      receiveLength_ = 0;
    }
  }
#endif

  // ── Sending ───────────────────────────────────────────────────────────────

  /** Open a record. Follow with any number of add*() calls, then endRecord(). */
  void beginRecord() {
    port_.write((uint8_t)'^');
    recordIsEmpty_ = true;
  }

  /** Close the line and terminate it. Also resets the heartbeat timer. */
  void endRecord() {
    if (recordIsEmpty_) port_.write((uint8_t)'^');   // "^^" — the canonical heartbeat
    port_.write((uint8_t)'\n');
    lastSendMillis_ = UPLINE_MILLIS();
  }

  /** Add the entry `key~value`, with both escaped as needed. */
  void addText(const char* key, const char* value) {
    writeEscaped(key);
    port_.write((uint8_t)'~');
    writeEscaped(value);
    endEntry();
  }

  /** Add the entry `key~value` for a whole number. */
  void addInt(const char* key, int32_t value) {
    char buffer[UPLINE_NUMBER_BUFFER_SIZE];
    writeEscaped(key);
    port_.write((uint8_t)'~');
    writePlain(uplineFormatScaled(value, 0, buffer));
    endEntry();
  }

  /** Add the entry `key~0` or `key~1`. */
  void addBool(const char* key, bool value) {
    writeEscaped(key);
    port_.write((uint8_t)'~');
    port_.write((uint8_t)(value ? '1' : '0'));
    endEntry();
  }

  /**
   * Add a key with no value at all. From a host this is a read, or an execute
   * with no arguments (spec §6); from a device it is rarely what you want —
   * a reading is `addText`/`addInt`/`addFixed`.
   */
  void addFlag(const char* key) {
    writeEscaped(key);
    endEntry();
  }

#if UPLINE_ENABLE_FIXED
  /**
   * Add a fixed-point value, sent as ordinary decimal text.
   *
   * Pass the value already scaled — 2345 with scale 2 sends "23.45". The scale
   * must match what the key declares in the schema, e.g. `temp|fix2`.
   */
  void addFixed(const char* key, int32_t scaledValue, uint8_t scale) {
    char buffer[UPLINE_NUMBER_BUFFER_SIZE];
    writeEscaped(key);
    port_.write((uint8_t)'~');
    writePlain(uplineFormatScaled(scaledValue, scale, buffer));
    endEntry();
  }
#endif

#if UPLINE_ENABLE_BASE64
  /**
   * Add binary data as unpadded base64url. Streams directly to the port, so
   * any payload size works without a temporary buffer. The base64url alphabet
   * contains no reserved character, so nothing needs escaping.
   */
  void addBase64(const char* key, const uint8_t* data, size_t byteCount) {
    writeEscaped(key);
    port_.write((uint8_t)'~');

    uint32_t accumulator = 0;
    int8_t bitsHeld = 0;
    for (size_t index = 0; index < byteCount; ++index) {
      accumulator = (accumulator << 8) | data[index];
      bitsHeld += 8;
      while (bitsHeld >= 6) {
        bitsHeld -= 6;
        port_.write((uint8_t)uplineBase64Char((uint8_t)((accumulator >> bitsHeld) & 0x3F)));
      }
    }
    if (bitsHeld) {  // flush the final partial group, left-aligned, unpadded
      port_.write((uint8_t)uplineBase64Char((uint8_t)((accumulator << (6 - bitsHeld)) & 0x3F)));
    }
    endEntry();
  }
#endif

  /** Send a one-pair record: `^key~value^`. */
  void sendText(const char* key, const char* value) {
    beginRecord();
    addText(key, value);
    endRecord();
  }

  /** Send a one-pair record: `^key~value^` for a whole number. */
  void sendInt(const char* key, int32_t value) {
    beginRecord();
    addInt(key, value);
    endRecord();
  }

  /**
   * Report an error. The text is free-form — a short code, a sentence, a line
   * number, whatever helps. Hosts treat it as opaque and only log or show it,
   * so there is no vocabulary to conform to. Sending errors is entirely
   * optional; a device short on flash may stay silent.
   */
  void sendError(const char* text) {
    beginRecord();
    addText("_e", text);
    endRecord();
  }

  /** Send the descriptor. Done for you when a "?" command arrives. */
  void sendSchema() {
    for (const char* cursor = schema_; ; ++cursor) {
      char character = (char)UPLINE_READ_FLASH_BYTE(cursor);
      if (!character) break;
      port_.write((uint8_t)character);
    }
    port_.write((uint8_t)'\n');
    lastSendMillis_ = UPLINE_MILLIS();
    lastSchemaMillis_ = lastSendMillis_;
  }

  /** Send the empty record `^^`, the canonical no-op that proves liveness. */
  void sendHeartbeat() {
    port_.write((uint8_t)'^');
    port_.write((uint8_t)'^');
    port_.write((uint8_t)'\n');
    lastSendMillis_ = UPLINE_MILLIS();
  }

 private:
  // Write text with every reserved character escaped (spec §6).
  void writeEscaped(const char* text) {
    for (const char* cursor = text; *cursor; ++cursor) {
      char character = *cursor;
      switch (character) {
        case '\\': port_.write((uint8_t)'\\'); port_.write((uint8_t)'\\'); break;
        case '^':  port_.write((uint8_t)'\\'); port_.write((uint8_t)'^');  break;
        case '|':  port_.write((uint8_t)'\\'); port_.write((uint8_t)'|');  break;
        case '~':  port_.write((uint8_t)'\\'); port_.write((uint8_t)'~');  break;
        case '\n': port_.write((uint8_t)'\\'); port_.write((uint8_t)'n');  break;
        case '\r': port_.write((uint8_t)'\\'); port_.write((uint8_t)'r');  break;
        default:   port_.write((uint8_t)character);                        break;
      }
    }
  }

  // Numbers and base64url never contain a reserved character, so they skip the
  // escaper entirely.
  void writePlain(const char* text) {
    for (const char* cursor = text; *cursor; ++cursor) port_.write((uint8_t)*cursor);
  }

  // Every entry closes with its own caret, so the caret that ends one entry is
  // also the caret that opens the next (spec §3). No separator bookkeeping.
  void endEntry() {
    port_.write((uint8_t)'^');
    recordIsEmpty_ = false;
  }

#if UPLINE_ENABLE_RECEIVE
  // Parse one line, answering "?" ourselves and passing everything else on.
  void dispatch(char* line, size_t length) {
    schemaRequested_ = false;
    uplineParseLine(line, length, &UplineDevice::route, this);
    // Malformed entries are dropped individually. Call sendError() from your
    // handler if you would rather say so out loud.
    if (schemaRequested_) sendSchema();
  }

  // Trampoline from the parser's plain function pointer back to this instance.
  // Passing `this` as context is what lets several ports coexist.
  static void route(void* context, const UplineEntry& entry) {
    UplineDevice* self = (UplineDevice*)context;
    if (entry.isRead() && entry.key[0] == '?' && entry.key[1] == '\0') {
      self->schemaRequested_ = true;          // answered once parsing completes
      return;
    }
    if (self->handler_) self->handler_(entry);
  }
#endif

  SerialPort& port_;
  const char* schema_;
  UplineEntryHandler handler_;
  uint32_t lastSendMillis_;
  uint32_t lastSchemaMillis_;
  bool recordIsEmpty_;
#if UPLINE_ENABLE_RECEIVE
  bool schemaRequested_;
  bool isOverlong_;
  uint16_t receiveLength_;
  char receiveBuffer_[UPLINE_RX_BUFFER_SIZE];
#endif
};

#if defined(ARDUINO)
/** Upline bound to an Arduino Stream — use this unless you are off-Arduino. */
typedef UplineDevice<Stream> Upline;
#endif

#endif  // UPLINE_HPP
