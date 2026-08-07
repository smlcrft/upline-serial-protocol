// upline.hpp — Upline serial protocol, single-header C++ implementation.
//
//   ^temp|23.45~rh|48~fan|1^
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
//   //    not RAM. Write only the pairs — the "^!~" and "^" are added for you.
//   //    uuid / name / desc / ver are required; the rest are yours.
//   UPLINE_SCHEMA(mySchema,
//     "uuid|n4o8IUt-TQWzph4vfI2Qqw"      // unique per device
//     "~name|Greenhouse"
//     "~desc|North bed sensors"
//     "~ver|1"                            // Upline version, always 1
//     "~temp|fix2||-40.00|125.00"         // 2-decimal value, range -40..125
//     "~fan|bool|0");                     // on/off, defaults off
//
//   Upline upline(Serial, mySchema);
//
//   // 2. Handle anything the host sends you. Optional.
//   void uplineOnKeyValPair(const char* key, const char* value, bool isFlag) {
//     if (!strcmp(key, "fan")) digitalWrite(FAN_PIN, value[0] == '1');
//   }
//
//   void setup() {
//     Serial.begin(115200);               // 9600 if your clock is an internal RC
//     upline.onPair(uplineOnKeyValPair);
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
// What the parts actually cost, measured on a complete sketch:
//
//                                        Uno            ATtiny85
//   int + bool only ................. 3258 B / 342 B   2650 B / 237 B
//   + addFixed + addBase64 .......... +478 B           +462 B
//   UPLINE_TRANSMIT_ONLY 1 .......... -620 B / -133 B  -626 B / -133 B
//   UPLINE_RX_BUFFER_SIZE, per byte ..        / -1 B            / -1 B
//
// So the two knobs that genuinely change the build are UPLINE_TRANSMIT_ONLY,
// which removes the parser and receive buffer, and UPLINE_RX_BUFFER_SIZE,
// which is pure RAM. UPLINE_ENABLE_FIXED and UPLINE_ENABLE_BASE64 save nothing
// on a normal Arduino build; they exist as a guardrail, so that a project with
// a hard size ceiling gets a compile error instead of silently linking a
// feature, and for toolchains that do not garbage-collect unused sections.
//
// A tight ATtiny85 sensor — integers and booleans only, full send and receive —
// lands at 2694 B of 8192 B flash and 241 B of 512 B RAM.
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
// what the device READS; records it sends may be any length (spec §11).
//
// 128 is the floor the spec requires, and it is deliberately above the 120-byte
// host-to-device recommendation so a conforming host can never overflow it.
// Lowering it below 128 makes the device non-conforming — do that only if you
// also control the host and know its commands are shorter.
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
// This is a conforming profile, not a crippled device (spec §9.1): since such a
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
// uplineParseRecord can ignore this entirely.
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
 * The four mandatory keys are `uuid`, `name`, `desc`, and `ver` (spec §8).
 *
 *   UPLINE_SCHEMA(mySchema,
 *     "uuid|n4o8IUt-TQWzph4vfI2Qqw"
 *     "~name|Greenhouse"
 *     "~desc|North bed sensors"
 *     "~ver|1"
 *     "~temp|fix2||-40.00|125.00"
 *     "~fan|bool|0");
 *
 * @param name      identifier to declare, later passed to the Upline constructor
 * @param ...       the descriptor body, as one or more string literals
 */
#define UPLINE_SCHEMA(name, ...) \
  static const char name[] UPLINE_FLASH_STORAGE = "^!~" __VA_ARGS__ "^"

// ─────────────────────────────────────────────────────────────────────────────
// Parse results
// ─────────────────────────────────────────────────────────────────────────────

/** Outcome of parsing one record. Only UplineOk means the record was valid. */
enum UplineResult {
  UplineOk = 0,            ///< Record parsed; every pair was dispatched.
  UplineNotUpline = -1,    ///< Line did not start with '^'. Ignore it, not an error.
  UplineBadEscape = -2,    ///< Undefined or dangling backslash escape.
  UplineTruncated = -3,    ///< No closing '^' before end of line.
  UplineTrailing = -4,     ///< Bytes appeared after the closing '^'.
  UplineBadKey = -5        ///< Empty key, or an undefined '_'-prefixed key.
};

/** Callback for each key/value pair, as used by the UplineDevice class. */
typedef void (*UplinePairHandler)(const char* key, const char* value, bool isFlag);

/**
 * Callback for each key/value pair, as used by uplineParseRecord directly.
 * `context` is whatever you passed in, so several parsers can run independently.
 */
typedef void (*UplinePairHandlerWithContext)(void* context, const char* key,
                                             const char* value, bool isFlag);

// ─────────────────────────────────────────────────────────────────────────────
// Core codec — free functions, usable without the Upline class
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Decode backslash escapes in place. The result is always the same length or
 * shorter, so it safely overwrites its input.
 *
 * Recognises \\  \^  \|  \~  \n  \r  (spec §5). Assumes the text already passed
 * validation in uplineParseRecord; the trailing-backslash guard only keeps this
 * safe if you call it directly on untrusted text.
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
 * True if a key is legal: non-empty, and the only '_'-prefixed key allowed is
 * "_e" (spec §6). The reserved '_' namespace is what lets future versions of
 * the protocol add fields without colliding with application keys.
 */
inline bool uplineIsKeyAllowed(const char* key) {
  if (!*key) return false;                    // empty key
  if (*key != '_') return true;               // ordinary application key
  return key[1] == 'e' && key[2] == '\0';     // "_e" is the only reserved key
}

/**
 * Parse one complete record and dispatch each key/value pair.
 *
 * The record must already have its line terminator removed — see UplineDevice
 * for the reader that does that. The buffer is modified in place: separators
 * become NUL terminators and escapes are decoded, so `key` and `value` point
 * into it and stay valid only until the next record is read.
 *
 * A pair with no '|' is a flag: `value` is "" and `isFlag` is true. A first-
 * position flag is the record's command (spec §7).
 *
 * Pairs are dispatched as they are scanned, so if a later pair is malformed the
 * handler may already have seen earlier ones. A pair is only ever dispatched
 * once its terminator is seen, so a truncated record never delivers a partial
 * value. Stage and commit in your handler if partial application is unsafe.
 *
 * @param line     one record, terminator stripped; modified in place
 * @param length   bytes in `line`, not counting the NUL
 * @param handler  called once per pair, in wire order
 * @param context  passed straight through to the handler; may be NULL
 * @return         UplineOk, or a negative UplineResult
 */
inline UplineResult uplineParseRecord(char* line, size_t length,
                                      UplinePairHandlerWithContext handler,
                                      void* context) {
  char* cursor = line;
  char* end = line + length;

  // A line that does not open with '^' is not ours — foreign traffic or debug
  // output sharing the same port. Silently ignored, never an error (spec §4).
  if (length < 1 || *cursor++ != '^') return UplineNotUpline;

  char* key = cursor;
  char* value = NULL;

  for (; cursor < end; ++cursor) {
    // Skip over a valid escape so an escaped delimiter is not mistaken for a
    // real one. This is why a plain strtok cannot parse Upline.
    if (*cursor == '\\') {
      if (cursor + 1 >= end) return UplineBadEscape;     // dangling backslash
      switch (cursor[1]) {
        case '\\': case '^': case '|': case '~': case 'n': case 'r':
          ++cursor;
          continue;
        default:
          return UplineBadEscape;                        // undefined escape
      }
    }

    // Only the FIRST unescaped '|' splits key from value; later bars are
    // ordinary value bytes, which is what lets values carry sub-structure.
    if (*cursor == '|' && value == NULL) {
      *cursor = '\0';
      value = cursor + 1;
      continue;
    }

    if (*cursor == '~' || *cursor == '^') {
      char terminator = *cursor;
      *cursor = '\0';

      // Empty segments (from "~~" or a leading/trailing "~") are ignored.
      if (*key || value) {
        if (!uplineIsKeyAllowed(key)) return UplineBadKey;
        uplineUnescapeInPlace(key);
        if (value) uplineUnescapeInPlace(value);
        handler(context, key, value ? value : "", value == NULL);
      }

      // The closing caret must be the line's last byte. Anything after it means
      // a lost newline merged two records — reject rather than silently accept.
      if (terminator == '^') {
        return (cursor + 1 == end) ? UplineOk : UplineTrailing;
      }

      key = cursor + 1;
      value = NULL;
    }
  }

  return UplineTruncated;                                // never saw a closing '^'
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
inline int32_t uplineParseScaled(const char* text, uint8_t scale) {
  int32_t value = 0;
  bool isNegative = false;

  if (*text == '-') { isNegative = true; ++text; }
  else if (*text == '+') { ++text; }

  while (*text >= '0' && *text <= '9') value = value * 10 + (*text++ - '0');

  if (*text == '.') {
    ++text;
    while (scale && *text >= '0' && *text <= '9') {
      value = value * 10 + (*text++ - '0');
      --scale;
    }
    if (*text >= '5' && *text <= '9') ++value;          // round half up
  }

  while (scale--) value *= 10;                          // pad a short fraction
  return isNegative ? -value : value;
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
   * Register the callback that receives each inbound key/value pair.
   * By convention the callback is named uplineOnKeyValPair, matching this
   * library's free-function prefix and keeping it greppable in a large sketch.
   */
  void onPair(UplinePairHandler uplineOnKeyValPair) { handler_ = uplineOnKeyValPair; }

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

  /** Close the record and terminate the line. Also resets the heartbeat timer. */
  void endRecord() {
    port_.write((uint8_t)'^');
    port_.write((uint8_t)'\n');
    lastSendMillis_ = UPLINE_MILLIS();
  }

  /** Add `key|value` with the value escaped as needed. */
  void addText(const char* key, const char* value) {
    writeSeparator();
    writeEscaped(key);
    port_.write((uint8_t)'|');
    writeEscaped(value);
  }

  /** Add `key|value` for a whole number. */
  void addInt(const char* key, int32_t value) {
    char buffer[UPLINE_NUMBER_BUFFER_SIZE];
    writeSeparator();
    writeEscaped(key);
    port_.write((uint8_t)'|');
    writePlain(uplineFormatScaled(value, 0, buffer));
  }

  /** Add `key|0` or `key|1`. */
  void addBool(const char* key, bool value) {
    writeSeparator();
    writeEscaped(key);
    port_.write((uint8_t)'|');
    port_.write((uint8_t)(value ? '1' : '0'));
  }

  /**
   * Add a key with no value. In first position this is the record's command;
   * anywhere else it means "true" or "present".
   */
  void addFlag(const char* key) {
    writeSeparator();
    writeEscaped(key);
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
    writeSeparator();
    writeEscaped(key);
    port_.write((uint8_t)'|');
    writePlain(uplineFormatScaled(scaledValue, scale, buffer));
  }
#endif

#if UPLINE_ENABLE_BASE64
  /**
   * Add binary data as unpadded base64url. Streams directly to the port, so
   * any payload size works without a temporary buffer. The base64url alphabet
   * contains no reserved character, so nothing needs escaping.
   */
  void addBase64(const char* key, const uint8_t* data, size_t byteCount) {
    writeSeparator();
    writeEscaped(key);
    port_.write((uint8_t)'|');

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
  }
#endif

  /** Send a one-pair record: `^key|value^`. */
  void sendText(const char* key, const char* value) {
    beginRecord();
    addText(key, value);
    endRecord();
  }

  /** Send a one-pair record: `^key|value^` for a whole number. */
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
  // Write text with every reserved character escaped (spec §5).
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

  void writeSeparator() {
    if (!recordIsEmpty_) port_.write((uint8_t)'~');
    recordIsEmpty_ = false;
  }

#if UPLINE_ENABLE_RECEIVE
  // Parse one record, answering "?" ourselves and passing everything else on.
  void dispatch(char* line, size_t length) {
    schemaRequested_ = false;
    uplineParseRecord(line, length, &UplineDevice::route, this);
    // A malformed record is simply dropped. Call sendError() from your handler
    // if you would rather say so out loud.
    if (schemaRequested_) sendSchema();
  }

  // Trampoline from the parser's plain function pointer back to this instance.
  // Passing `this` as context is what lets several ports coexist.
  static void route(void* context, const char* key, const char* value, bool isFlag) {
    UplineDevice* self = (UplineDevice*)context;
    if (isFlag && key[0] == '?' && key[1] == '\0') {
      self->schemaRequested_ = true;          // answered once parsing completes
      return;
    }
    if (self->handler_) self->handler_(key, value, isFlag);
  }
#endif

  SerialPort& port_;
  const char* schema_;
  UplinePairHandler handler_;
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
