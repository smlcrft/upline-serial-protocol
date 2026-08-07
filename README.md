# Upline

**The simplest serial protocol.** Newline-delimited ASCII key/value records for microcontrollers.

```text
^temp|23.45~rh|48~fan|1^
```

**Status:** v1 (`ver|1`)

Upline gives any serial-capable device a predictable way to publish state, accept commands, and describe itself — cheaply enough to run on a 20-year-old 8-bit microcontroller, and structured enough that a host can build a complete interface for a device it has never seen before.

> **You do not have to implement this yourself.**
>
> **[`upline-arduino/`](upline-arduino/) is a complete, tested implementation** — one header, drop it in, and any Arduino or C++-capable board speaks Upline:
>
> ```cpp
> #include <upline.hpp>
> ```
>
> - **Small.** A complete device — codec, `fixN`, base64url, descriptor, heartbeat, telemetry, 128-byte receive buffer — is **3,900 B of flash and 364 B of SRAM** on an Uno, and fits an **ATtiny85** with 8 KB flash and 512 B of RAM. The decode core alone is 436 B.
> - **Optimized.** Header-only. No allocation, no libc, no floating point, no `String`. Anything you never call is never linked, so unused features cost nothing.
> - **Tested.** Compiled clean on AVR (ATmega8/168/328P, ATtiny85, ATtiny1614), SAMD21/SAMD51, RP2040, RP2350 (ARM and RISC-V), ESP32 / S3 / C3, and Teensy 4.1. All 61 conformance vectors in §16 pass.
> - **Not Arduino-only.** Supply any port with `available()`, `read()`, and `write(uint8_t)`, then define `UPLINE_MILLIS()`.
>
> Three worked examples ship with it, including a servo-and-temperature node on an ATtiny85 and the onboard RGB plus die temperature on an ItsyBitsy M4. CC0, like this spec.
>
> **The rest of this document is the wire format** — read it to write your own implementation, to build the host side, or to check a detail. You do not need it to put an Upline device on a bench.

---

## Contents

- [1. At a glance](#1-at-a-glance)
- [2. Transports](#2-transports)
- [3. Grammar](#3-grammar)
- [4. Framing](#4-framing)
- [5. Escaping](#5-escaping)
- [6. Pairs, keys, values](#6-pairs-keys-values)
  - [6.1 Only the first `|` is significant](#61-only-the-first--is-significant)
- [7. Commands](#7-commands)
- [8. The descriptor](#8-the-descriptor)
  - [8.1 Device identity (`uuid`)](#81-device-identity-uuid)
  - [8.2 Descriptor values](#82-descriptor-values)
  - [8.3 Types](#83-types)
  - [8.4 `fixN` — decimals without floating point](#84-fixn--decimals-without-floating-point)
  - [8.5 Device-side arithmetic — outside the protocol](#85-device-side-arithmetic--outside-the-protocol)
- [9. Heartbeat](#9-heartbeat)
  - [9.1 Transmit-only devices](#91-transmit-only-devices)
- [10. Errors](#10-errors)
- [11. Limits and timing](#11-limits-and-timing)
  - [11.1 The drain deadline](#111-the-drain-deadline)
  - [11.2 Backpressure differs by transport](#112-backpressure-differs-by-transport)
  - [11.3 Baud rates](#113-baud-rates)
  - [11.4 Discovering the rate](#114-discovering-the-rate)
- [12. Reference implementation — non-normative](#12-reference-implementation--non-normative)
  - [12.1 Decoder](#121-decoder)
  - [12.2 Fixed-point helpers](#122-fixed-point-helpers)
  - [12.3 Partial application](#123-partial-application)
  - [12.4 Line reader](#124-line-reader)
- [13. Footprint](#13-footprint)
- [14. Building on Upline](#14-building-on-upline)
  - [14.1 Descriptor to interface](#141-descriptor-to-interface)
  - [14.2 State, identity, and liveness](#142-state-identity-and-liveness)
  - [14.3 Logging and replay](#143-logging-and-replay)
  - [14.4 What Upline deliberately does not provide](#144-what-upline-deliberately-does-not-provide)
- [15. Extending Upline](#15-extending-upline)
- [16. Conformance vectors](#16-conformance-vectors)
  - [16.1 Well-formed](#161-well-formed)
  - [16.2 Malformed and edge](#162-malformed-and-edge)
  - [16.3 Descriptor and fixed point](#163-descriptor-and-fixed-point)
  - [16.4 Line endings](#164-line-endings)
- [17. Open questions](#17-open-questions)
- [Appendix A — why not JSON or MQTT](#appendix-a--why-not-json-or-mqtt)
- [Appendix B — prior art](#appendix-b--prior-art)
- [References](#references)

---

## 1. At a glance

- **One record per line.** Split on LF or CR; every record parses independently.
- **Five reserved characters.** `^ ~ | \` and the line terminator. Spaces, quotes, colons, commas, and brackets are all legal raw, so records stay readable in any terminal.
- **Complete escaping, including newline** — six sequences, no forbidden-character lists.
- **Binary via base64url**, unpadded, in a `b64` value.
- **Self-describing.** Every device answers `^?^` with its full schema: keys, types, defaults, and ranges — or broadcasts it unprompted if it has no receive path (§9.1).
- **Liveness built in.** A heartbeat tells a host the device is alive and speaks Upline before anything is sent to it.
- **No floating point, anywhere.** Decimals are `fixN` — a decimal string on the wire, a scaled integer in the device.
- **Transport-agnostic.** Anything that carries an ordered stream of bytes (§2).
- **Extensible by design** (§15): application keys, application commands, vendor types, arbitrary binary.

**Measured cost of a complete device** — everything on: codec, `fixN`, base64url, `PROGMEM` descriptor, heartbeat, telemetry, and a 128-byte receive buffer. Built with `arduino-cli` from [`upline-arduino/src/upline.hpp`](upline-arduino/src/upline.hpp):

| Board | Chip | Flash | of | SRAM | of |
|---|---|---|---|---|---|
| Uno / Duemilanove-328 | ATmega328P | 3,900 B | 12% of 32,256 | 364 B | 17% of 2,048 |
| **Duemilanove (2008)** | ATmega168 | **3,900 B** | **27% of 14,336** | 364 B | 35% of 1,024 |
| Arduino NG (2005) | ATmega8 | 3,684 B | 51% of 7,168 | 364 B | 35% of 1,024 |
| ATtiny85 | ATtiny85 | 3,262 B | 39% of 8,192 | 259 B | 50% of 512 |
| ATtiny85 (Tx only) | ATtiny85 | 2,644 B | 32% of 8,192 | 126 B | 24% of 512 |

The last row is the transmit-only profile (§9.1) at the same workload: no receive path, so the parser and receive buffer are gone and the device broadcasts its descriptor instead of answering `?`. Trimming further to integers and booleans only puts a receive-capable ATtiny85 at **2,694 B and 241 B**. The decode core alone is **436 B** and needs no libc, no allocator, and no floating point (§13).

---

## 2. Transports

Upline requires exactly one thing: **an ordered stream of bytes.** It assumes no packet boundaries, no maximum transmission unit, no flow control, no control lines, and no reliability guarantee.

That makes it usable over, among others:

| Transport | Notes |
|---|---|
| **UART / TTL serial** | The baseline. See §11 for timing obligations. |
| **USB CDC-ACM** | Adds a link-layer CRC and retransmission, and real backpressure (§11.2). |
| **USB-to-serial bridges** (FTDI, CH340, CP210x) | Behave as a bare UART to the device; no backpressure. |
| **Bluetooth Classic SPP / RFCOMM** | A byte stream by definition. Works unmodified. |
| **BLE UART services** (Nordic NUS and equivalents) | Packet-oriented with a small MTU, so records arrive fragmented. Harmless: reassembly is "accumulate until a line terminator." |
| **RS-232 / RS-422** | Unmodified. |
| **RS-485** | Works point-to-point. On a multi-drop bus see the caveat below. |
| **TCP, PTYs, pipes, files** | A captured Upline stream replays as a valid stream. |

**Fragmentation is a non-issue.** Because records are line-delimited and self-framing (§4), a receiver never needs to know how bytes were chunked. A record split across three BLE notifications reassembles the same as one delivered whole.

**Loss is survivable; reordering is not.** A corrupted or dropped record is discarded and the next one resynchronizes (§4), and the heartbeat (§9) bounds how long a receiver waits. But Upline assumes bytes arrive **in order and without interleaving from other senders**. A transport that may reorder or interleave must be wrapped in something that does not.

**Multi-drop buses need arbitration.** Upline is point-to-point by default, and a device transmits unprompted — the heartbeat and any telemetry are not solicited. On a shared bus such as RS-485, either suppress unsolicited transmission and poll instead, or arbitrate the bus above Upline. The protocol defines no addressing and no collision handling.

**8-bit clean is not required.** Every byte Upline emits is printable ASCII or a UTF-8 continuation byte, so it survives transports that mangle control characters, and it stays readable in any terminal or logger along the path.

---

## 3. Grammar

```abnf
stream   = *( line eol )
eol      = CRLF / CR / LF          ; any of the three; CRLF is ONE terminator
line     = record / foreign

foreign  = *OCTET                  ; any line whose first byte is not "^"; ignored
record   = "^" payload "^"         ; the closing caret MUST be the line's last byte
payload  = [ pair *( "~" pair ) ]

pair     = flag / kv
flag     = key                     ; no "|" — §6
kv       = key "|" value

key      = 1*kchar
value    = *vchar

kchar    = escape / %x20-5B / %x5D / %x5F-7B / %x7D / %x80-FF
vchar    = kchar / "|"
escape   = "\" ( "\" / "^" / "|" / "~" / "n" / "r" )
```

`kchar` is every printable byte except the reserved four, plus `%x80-FF` for UTF-8. Excluded by construction: `\` (0x5C), `^` (0x5E), `|` (0x7C), `~` (0x7E), and all control bytes 0x00–0x1F and 0x7F.

| Char | Hex | Role |
|---|---|---|
| `^` | 0x5E | Record open / close, and sync byte |
| `~` | 0x7E | Pair separator |
| `\|` | 0x7C | Key/value separator — **first occurrence in a pair only** |
| `\` | 0x5C | Escape introducer |
| LF | 0x0A | Record terminator |
| CR | 0x0D | Record terminator, alone or as CRLF (§4) |

Legal raw in keys and values: space, `"`, `'`, `:`, `,`, `;`, `=`, `/`, `#`, `$`, `%`, `&`, `*`, `+`, `<`, `>`, `?`, `@`, `[`, `]`, `{`, `}`, `` ` ``.

---

## 4. Framing

A receiver MUST:

1. Split the stream on **LF (0x0A) or CR (0x0D)**. A CRLF pair terminates one record, not two — the second terminator finds an empty buffer and is ignored. Empty lines are ignored.
2. If the line's first byte is not `^`, **ignore the line.** Not an error — it is foreign traffic or human-readable debug output.
3. Otherwise scan per §12. The **first unescaped `^` after position 0 closes the record**, and it MUST be the line's last byte.
4. End-of-line with no closing `^` → **truncated.** Discard; MAY report an error (§10).
5. Any byte after the closing `^` → **framing error.** Discard; MAY report an error (§10).

Rule 5 is strict deliberately: `^a|1^b|2^` is a device that dropped a newline, and reading one pair while silently discarding the other would hide the fault. **Multiple records on one line are not supported.**

Resynchronization comes from rule 2 plus line framing — a receiver joining a stream mid-line sees a fragment that fails the leading-`^` test, and the next terminator restores sync. A raw LF or CR inside a value therefore splits the record and the fragment is discarded, never misparsed.

**Accepting all three line endings is deliberate and free.** Terminal programs differ: the Arduino Serial Monitor alone offers LF, CR, CRLF, and none. Accepting only LF means a user who picks "Carriage return" gets *zero records and no error* — silence with no indication of why. Treating either byte as a terminator costs nothing; measured on ATmega328P the universal reader is **98 B against 122 B** for an LF-only reader that strips trailing CR, because there is no trailing-byte special case. The one mode that cannot work is "no line ending" — no line-delimited protocol can function without a terminator.

---

## 5. Escaping

Six sequences. **Escapes are symbolic:** `\n` is the two bytes `\` `n`, not a raw 0x0A — a raw 0x0A is consumed by line splitting (§4) before escape processing runs, so only symbolic encoding makes a newline escape reachable.

```text
sequence   bytes        decodes to    name
--------   ----------   ----------    ---------------
\\         0x5C 0x5C    0x5C  \       backslash
\^         0x5C 0x5E    0x5E  ^       caret
\|         0x5C 0x7C    0x7C  |       vertical bar
\~         0x5C 0x7E    0x7E  ~       tilde
\n         0x5C 0x6E    0x0A  LF      line feed
\r         0x5C 0x72    0x0D  CR      carriage return
```

> Keep the table above as a code block in any copy of this document. GFM rewrites `\|` to `|` even inside code spans, which deletes the backslash and makes an escape table assert the opposite of what it means.

The set is **complete over the reserved set**, and deliberately not over all 256 byte values — there is no `\xHH`. Arbitrary bytes travel as `b64` (§8.3). Raw control bytes other than the framing LF/CR MUST NOT appear in a record.

**Invalid escapes.** A `\` followed by any byte other than `\ ^ | ~ n r`, or appearing as a record's final byte, is invalid: the receiver MUST **reject the entire record** and MAY report an error (§10). Strict rather than lenient, because records drive actuators — a silently mangled setpoint is worse than a dropped line, and the heartbeat (§9) makes a dropped line self-healing.

**Encoder obligations.** Escape `\`, `^`, `~`, LF, and CR wherever they occur in a key or value, and escape `|` in a **key**. Escaping `|` in a value is optional (§6.1); the simplest conforming encoder escapes all five unconditionally in both, and decoders MUST accept that. Never emit raw bytes 0x00–0x1F or 0x7F, and never pad after the closing caret.

**Line endings on emit.** Encoders SHOULD terminate records with LF and MAY use CRLF. Both are conforming: the Arduino core's `Print::println()` is `write("\r\n")`, so CRLF is what the most natural sketch idiom produces, and forbidding it would buy one byte at the cost of real friction.

---

## 6. Pairs, keys, values

A payload is zero or more pairs separated by unescaped `~`. An **empty segment** (`~~`, or a leading/trailing `~`) MUST be ignored by receivers and MUST NOT be emitted.

A pair with no unescaped `|` is a **flag** — a key with no value.

- A flag in **first position** is the record's **command** (§7); remaining pairs are its arguments.
- A flag elsewhere means **boolean true** / "present".

`key` (flag) and `key|` (empty-string value) are distinct; a decoder MUST expose the difference.

Keys are case-sensitive and MUST be non-empty after unescaping; for interoperability draw them from `[a-z0-9_]`. **Keys beginning with `_` are reserved** by this specification, and exactly one is defined: `_e` (§10). A receiver encountering any other `_`-prefixed key MUST reject the record — that is what keeps the namespace usable by future versions. If a key repeats in a record, **the last occurrence wins**; encoders SHOULD NOT emit duplicates.

### 6.1 Only the first `|` is significant

Within a pair, **only the first unescaped `|` separates key from value.** Every later `|` is an ordinary value byte needing no escape, so `note|a|b` is the key `note` with value `a|b`. This is what lets a value carry sub-structure, which the descriptor (§8.2) and array types depend on.

**Sub-field order matters.** An application splitting a value into sub-fields MUST split on **unescaped** `|` first, then unescape each sub-field. A sub-field needing a literal bar uses `\|`. A value with no unescaped `|` yields one sub-field. Treating the value as one opaque string and unescaping the whole range is also valid; both agree unless an escaped bar is present.

---

## 7. Commands

A flag in first position names a command. Two are mandatory; everything else is application-defined.

| Command | Direction | Meaning |
|---|---|---|
| `?` | host → device | Describe yourself. Takes no arguments. |
| `!` | device → host | Descriptor response. |

```text
^?^                         describe yourself
^reboot^                    application command, no arguments
^reboot~delay|500^          application command with an argument
```

A device that can receive MUST answer `^?^` with a single `!` record. A **transmit-only** device cannot, and instead broadcasts its descriptor unprompted — see §9.1. A device receiving an unrecognized command MUST NOT act on it, and MAY report an error (§10).

---

## 8. The descriptor

`!` carries identity and full schema, in **one record**. There is no chunking, no continuation flag, and no per-key query. For most devices it is a compile-time constant — on AVR, in `PROGMEM`.

```text
^!~uuid|4f2a9c~name|Greenhouse~desc|North bed sensors~ver|1~temp|fix2||-40.00|125.00~rh|int||0|100~fan|bool|0^
```

Four keys are **mandatory**:

| Key | Type | Meaning |
|---|---|---|
| `uuid` | `str` | Stable unique identifier for this physical device |
| `name` | `str` | Short human-readable name |
| `desc` | `str` | One-line description |
| `ver` | `int` | Upline version spoken. **`1`** for this document |

`ver` costs six bytes and buys version negotiation — without it a host cannot tell a v1 device from a future one, and key/value protocols that skipped it are the ones whose configs silently corrupt across firmware revisions.

### 8.1 Device identity (`uuid`)

`uuid` is the stable handle a host keys everything to: cached baud rate (§11.4), saved layouts, per-device settings, historical data. It MUST survive reboots and reflashes of unrelated code, so it belongs in flash or EEPROM — never generated at startup, which would orphan a host's stored state on every power cycle. It MUST NOT be shared across units of the same product, which would make two devices indistinguishable.

The protocol treats it as **opaque**. Receivers MUST NOT parse it, infer structure from it, or assume a length. It is a `str` and follows normal escaping rules (§5).

**For local and one-off use, any value that does not collide with the other devices you own is sufficient.** A readable name is often more useful than a random one:

```text
^!~uuid|bench-scope-1~name|Bench scope~desc|4ch logger~ver|1~…^
```

**For devices that may meet strangers — anything shipped, sold, or shared — use a UUID v4 encoded as unpadded base64url.** Generate it once per unit at production or first flash:

```text
canonical   9f8a3c21-4b7e-4d05-b3a6-1e2f7c8d90ab      36 characters
base64url   n4o8IUt-TQWzph4vfI2Qqw                     22 characters
```

The 16 raw bytes of the UUID become 22 base64url characters with padding stripped — the same encoding the `b64` type uses (§8.3), so there is one binary convention in the protocol rather than two. It saves 14 bytes per descriptor over the canonical hyphenated form, contains no Upline reserved character so it never needs escaping, and is URL- and filename-safe, which matters because a host will use it as a cache key or a path segment.

**Getting 128 unique bits on a microcontroller.** Many parts have a factory-unique serial to derive from: SAMD 128-bit serial, ESP32 MAC, STM32 96-bit UID, and the 10-byte `SERNUM` in the signature row of modern AVR (megaAVR 0-series, AVR-Dx). **Classic ATmega parts such as the ATmega328P have no unique serial** — their signature bytes identify only the model — so generate a UUID on the host at flash time and burn it into the sketch or EEPROM.

Derive-and-hash is acceptable where a factory serial exists, but a device MUST NOT present a value derived from something reassignable. A user-set Bluetooth name or a DHCP address is not identity.

Every other pair declares one data key, and **a descriptor is one record however long it needs to be.** A richly capable device with dozens of keys may emit a kilobyte or more, and should — the descriptor is a constant streamed straight out of flash, so its length costs the device no RAM at all, and a host handles a long line without difficulty (§11).

One record is a simplification, not a size budget: a host never deals with reassembly, sequencing, or a schema it only partly knows. That is why there is no chunking mechanism — long records make one unnecessary rather than more urgent.

### 8.2 Descriptor values

```abnf
descriptor-value = type [ "|" default [ "|" min "|" max ] ]
```

Up to four positional sub-fields; empty means absent. `temp|fix2||0|100` is a two-decimal fixed-point value with no default and a range of 0–100.

### 8.3 Types

The set is **closed**; extend with the `x-` prefix (§15). A receiver meeting an unknown type MUST treat it as `str`.

| Type | Wire form |
|---|---|
| `str` | UTF-8 text |
| `int` | Optionally-signed decimal integer |
| `fix1`…`fix9` | Optionally-signed decimal, **N implied fractional digits** — §8.4 |
| `bool` | `0` or `1` |
| `b64` | base64url (RFC 4648 §5), **no padding** |
| `strs` | Sub-fields split on unescaped `\|`, each a `str` |
| `ints` | Sub-fields, each an `int` |
| `fix1s`…`fix9s` | Sub-fields, each a `fixN` at the same scale |
| `b64json` | **OPTIONAL.** base64url of UTF-8 JSON text |
| `x-*` | Vendor extension |

For array types, `default`, `min`, and `max` MUST be empty — their bars are indistinguishable from element separators.

**`b64` is unpadded, normatively.** RFC 4648 §3.2 requires padding *"unless the specification referring to this document explicitly states otherwise"* — this specification so states, following RFC 8949 §6.1 (CBOR→JSON) and RFC 7515 (JOSE/JWT). The alphabet `A-Za-z0-9-_` collides with no reserved character, so `b64` values never need escaping, and dropping padding removes `=` from the wire.

**`b64json` is optional and discouraged** — it reintroduces the multi-kilobyte parser this protocol exists to avoid. The name is load-bearing: JSON travels only as base64url, never as raw text, because raw JSON's own `\` escapes would each need doubling. A device MUST NOT be considered non-compliant for rejecting it.

### 8.4 `fixN` — decimals without floating point

**`fixN` is the only decimal type. There is no floating-point type.** No conforming implementation, on any part however capable, needs a float parser or formatter to speak Upline. A host preferring doubles converts after parsing; that never reaches the wire.

```text
temp|fix2||-40.00|125.00        declared in the descriptor
^temp|23.45^                    on the wire
2345                            in the device, as int32_t
```

- **The wire form is an ordinary decimal string**, never a bare scaled integer. The stream stays human-readable, a plain terminal remains a usable debugger, and a generic consumer that has not read the descriptor still sees correct magnitudes.
- Encoders SHOULD emit exactly N fractional digits. Receivers MUST accept fewer, and MUST accept no `.` at all — `^temp|23^` is 23.00 at `fix2`.
- `default`, `min`, and `max` use the same decimal form.
- **No exponent.** `1.5e3` is not a `fixN` value, so every numeric type here is parseable with an integer scanner alone.
- With `int32_t`, `fixN` spans ±(2³¹−1)/10ᴺ — for `fix2`, ±21,474,836.47.

**Why the decimal point, rather than a bare scaled integer on the wire.** Sending `^temp|2345^` and letting the descriptor supply the scale would save a measured 422 B in a complete sketch — but it makes every scale mismatch a silent order-of-magnitude error. If a device moves from `fix2` to `fix3` while a host holds a stale descriptor, a decimal wire degrades gracefully (`23.456` read at scale 2 gives 23.46, off by 0.004) while a bare integer does not (`23456` read at scale 2 gives **234.56, off by 10×**). It would also make the value meaningless without the descriptor — `2345` could be 23.45, 234.5, or 2345 — which breaks plain-terminal debugging and would mislead any generic consumer that plots or logs the stream.

**If a device genuinely cannot spare it,** declare the key as `int` and document the scale in `desc`. That costs only `atol` (160 B) and is fully conforming — it simply moves the scale out of machine-readable reach, which is the trade being made.

Floats are excluded rather than merely discouraged because the difference is measured: on ATmega328P `atof` costs 1,598 B and `dtostrf` 1,606 B, while the §12.2 helpers do the same work for 310 B and 386 B. In a complete sketch that is **+2,684 B — 44% → 81% of an ATmega8's flash** for identical protocol behavior. Removing the type makes the saving unconditional: a device cannot be handed a value it needs `atof` to read.

### 8.5 Device-side arithmetic — outside the protocol

**Nothing here is part of the wire format.** The protocol only ever sees decimal text, and the conversion at the boundary is textual: `up_fix_str` places a `.` among the digits and `up_fix_parse` accumulates them, with no multiply or divide in either normal path. But the scaled integer a device holds between those boundaries is ordinary fixed-point arithmetic, and two operations are easy to get wrong on a first implementation.

The scaled `int32_t` **is** the value — `2345` is 23.45 at `fix2` — and the scale is compile-time knowledge, since the device declares it. **Addition, subtraction, comparison, clamping, and scaling by a plain integer all work directly** on the scaled integers with no adjustment.

**Multiplication doubles the scale; division cancels it.** Both need a correction:

```c
/* fix2 × fix2 → fix2.  23.45 × 2.00: (2345 * 200) / 100 = 4690 */
int32_t mul = (int32_t)(((int64_t)a * b) / 100);

/* fix2 ÷ fix2 → fix2.  46.90 ÷ 2.00: (4690 * 100) / 200 = 2345 */
int32_t div = (int32_t)(((int64_t)a * 100) / b);
```

**Intermediates overflow far earlier than results do, and the threshold is lower than it looks.** `int32_t` at `fix2` holds values up to ±21,474,836.47, but the *raw product* `a * b` inside a multiply overflows once the scaled operands reach 46,341 — **just 463.40 at `fix2`, or 46.340 at `fix3`.** Two ordinary sensor readings can cross it. That is why the casts above are `int64_t`; without them the failure is a silent wrap, not a diagnostic.

On AVR 64-bit arithmetic is expensive, so where a hot path cannot afford it, restructure instead — divide before multiplying, hold accumulators at a lower scale, or keep a running total in a wider unit and convert once at emit time.

These are the reasons `fixN` caps at scale 9: past that, an `int32_t` holds less than a single integer unit.

---

## 9. Heartbeat

A device MUST emit at least one valid record every **2500 ms**. `^^` — empty payload — is the canonical no-op: 3 bytes including LF, 1.2 B/s.

**Any valid record resets the timer.** A device streaming telemetry needs no separate ping.

A receiver:

- MUST wait for at least one valid record before transmitting anything.
- SHOULD treat the device as stale after **3× the interval (7500 ms)** without one.
- SHOULD send `^?^` as its first transmission once liveness is established.

Receive-before-transmit does two jobs. It avoids poking an unknown device that speaks some other protocol, where a stray byte can trigger a bootloader or reconfigure a modem. And it absorbs reset-on-connect behavior: on many boards, opening the port asserts DTR and resets the microcontroller, so a second or two of bootloader silence follows and anything sent during it is lost (§11.4).

On transports that already signal connection state and liveness — BLE, TCP — the heartbeat is redundant for liveness but still serves as the "this endpoint speaks Upline" proof. Devices SHOULD emit it regardless; the cost is 1.2 B/s.

### 9.1 Transmit-only devices

Some devices have no receive path at all: only a TX pin wired, a pin-starved part doing software-serial output, a deliberately one-way opto-isolated link, or a sensor that must never be commandable. Upline supports these as a **profile**, not as a separate mode — the grammar, the parser, and the host are unchanged.

A transmit-only device:

- **MUST broadcast its `!` descriptor unprompted, at least every 5000 ms.** This replaces the `?`/`!` exchange it cannot participate in.
- **MUST still meet the 2500 ms heartbeat** (§9). Use `^^` or ordinary telemetry between descriptor broadcasts — do not simply send `!` on every beat, since a large descriptor becomes real bandwidth: 200 bytes every 2.5 s is 8% of a 9600 baud link, and a kilobyte is over 40%.
- MAY ignore inbound bytes entirely, and SHOULD leave its UART receiver disabled if it has one.

A host:

- **MUST accept an unsolicited `!`** exactly as it accepts a solicited one.
- SHOULD still send `^?^` once liveness is established (§9). It is harmless — a transmit-only device simply never answers — and it is how a host learns the device is one: no reply within a descriptor interval means treat the device as read-only.
- **SHOULD NOT discard records that arrive before the first `!`.** They are valid, readable Upline. Because `fixN` puts the decimal point on the wire (§8.4), `^temp|23.45^` carries its own magnitude; only the metadata — declared type, range, default — is missing, and it can be applied retroactively when the descriptor lands.

Nothing here is new protocol surface. A transmit-only device is one that happens to answer before it is asked.

---

## 10. Errors

Errors are ordinary records carrying the reserved key `_e`, whose value is **free-form** — a code, a sentence, a line number, whatever the device finds useful.

```text
^_e|badescape^
^_e|unknown key: humidty^
```

**No code vocabulary is specified.** A host's only useful response to a malformed record is to discard it and perhaps show the text to a human, so a registry would oblige every device to carry string constants for no programmatic gain. Emitting `_e` is always OPTIONAL — a device may drop bad records silently, or send `^_e^` as a bare flag.

Receivers MUST NOT branch on `_e` values. Treat them as opaque text for logging and display.

A line not beginning with `^` is **not an error** (§4) and MUST NOT produce `_e`.

---

## 11. Limits and timing

Upline sets no universal maximum record length, because the constraint is **asymmetric** — the two directions have very different memory available.

**Device → host** (telemetry, and the `!` descriptor). No protocol limit. A device streams a record out byte by byte and needs no buffer of its own, so length costs only the flash holding the content: a 1 KB descriptor is a 1 KB constant and nothing more. **Hosts MUST accept records of at least 4096 bytes** and SHOULD accept far more, applying only a sanity cap to bound a malfunctioning device rather than to constrain a legitimate one.

**Host → device** (commands). This is the real limit, because a device MUST buffer a whole record in SRAM before parsing it — and on the smallest targets total SRAM is 512 bytes. Absent any declaration from the device, **hosts SHOULD keep records ≤ 120 bytes** and **devices MUST accept at least 128**. Commands are naturally short; `^set~temp|23.45^` is 18 bytes.

The 128-byte floor sits just above the 120-byte recommendation on purpose, so a conforming host can never overflow a conforming device, and it stays affordable on the smallest parts — a 128-byte buffer is a quarter of an ATtiny85's total RAM. A device that expects larger inbound payloads should simply allocate more; nothing in the protocol caps it.

A receiver that drops an over-long record **MUST also discard the remainder up to the next line terminator**, not merely reset its buffer. The tail of a long record can itself begin with `^` and would otherwise parse as a spurious second record.

Implementations in both directions MUST parse incrementally, or drain the transport while assembling.

### 11.1 The drain deadline

On a receive buffer of size *B* bytes at *R* bytes per second, a reader has *B / R* seconds of slack before bytes are lost. The Arduino AVR core's 64-byte RX ring at 115200 baud 8N1 (11,520 B/s) gives 5.56 ms:

> **The read loop MUST NOT go longer than the buffer's drain time without consuming available bytes.**

The ring is *not* a line-length cap — a sketch can assemble a 255-byte line in its own buffer. What it bounds is how far consumption may lag the wire.

On AVR, overflow is silent. The core's RX ISR is `if (i != _rx_buffer_tail) { … }` with **no `else`** — a full buffer discards the byte and exposes neither that condition nor the hardware `DOR0` overrun bit. On parts where `RAMEND - RAMSTART < 1023` the rings drop to **16 bytes**, cutting the deadline to **1.4 ms**. A blocking delay, a slow sensor read, or a long blocking write in the same loop is what breaks this — not line length.

### 11.2 Backpressure differs by transport

- **USB CDC-ACM** gates on the USB peripheral's buffer-available bit: an undrained OUT endpoint makes the silicon **NAK** and the host retry, so backpressure is free, along with a link-layer CRC and retransmission per packet.
- **A bare UART**, including one behind a USB-to-serial bridge, has no such path. It silently drops.
- **BLE and TCP** provide their own flow control below Upline.

There is **no protocol-level checksum.** Where the transport provides integrity — USB, BLE, TCP — a checksum would be redundant, and §4's framing already catches truncation. On a raw UART over distance, integrity is the integrator's problem: shorten the run, slow the baud, or wrap Upline in a link layer.

### 11.3 Baud rates

Baud is **irrelevant on USB CDC-ACM, Bluetooth SPP/RFCOMM, BLE, and TCP** — the rate a host requests is a no-op the device may ignore. It matters only for a real UART: direct TTL/RS-232, or a USB-to-serial bridge (FTDI, CH340, CP210x) that clocks a physical line.

Divisor error for common AVR clocks, best of normal and double-speed mode. UART 8N1 tolerates roughly ±2% per end, and oscillator error adds on top:

```text
   baud |   1 MHz int |   8 MHz int |  16 MHz xtal |  20 MHz xtal |  12 MHz xtal
   9600 |      +0.16% |      +0.16% |       +0.16% |       +0.16% |       +0.16%
 115200 |      +8.51% |      -3.55% |       +2.12% |       -1.36% |       +0.16%
 250000 |   too fast  |      +0.00% |       +0.00% |       +0.00% |       +0.00%
```

**250000 is exact on every clock that is a multiple of 4 MHz**, because 250000 × 16 = 4,000,000 divides evenly. 115200 × 16 = 1,843,200 divides evenly into almost nothing, which is why it is the least accurate common rate — out of budget at 8 MHz before the oscillator is even considered.

| Rate | Requirement | When |
|---|---|---|
| **9600** | **MUST** be supported | Universal fallback. Always available, on every device. |
| **115200** | **SHOULD** be the default | Interoperable everywhere; `B115200` exists on every host platform. |
| **250000** | **MAY** be used | Exact divisor, 2.17× faster — but a non-standard host rate (§11.4). |

- A device whose clock is an **internal RC oscillator below ~12 MHz SHOULD default to 9600**, not 115200. At 8 MHz internal, 115200 is −3.55% from the divisor alone, and ±1–2% of uncalibrated oscillator drift lands on top of that.
- A device with **no hardware UART** (software serial, e.g. ATtiny85) SHOULD use 9600. At 8 MHz that is 833 cycles per bit and bit-bangable; 250000 is 32 cycles per bit and is not.
- **250000 is not a standard termios rate.** Linux has no `B250000` and macOS needs `IOSSIOSPEED`. Modern libraries handle custom rates, but `screen`, `minicom`, and some tooling will not — which is why 115200 rather than 250000 is the recommended default despite the worse arithmetic.

### 11.4 Discovering the rate

A host opening a raw UART cold does not know the device's rate, and a standard serial API cannot measure one — it can only set a rate and read bytes. Probing is therefore the only portable method, and **Upline needs no dedicated autobaud mechanism because the `?`/`!` handshake already validates a rate in both directions.**

For each candidate rate, in order:

1. **Open the port and send nothing.** §9's receive-before-transmit rule makes probing inherently safe: a wrong-rate probe cannot poke a device that speaks some other protocol.
2. **Wait for a line that parses as a valid record** (§4). Timeout **6 s** — worst case is a DTR-triggered reset (~2 s of bootloader) plus one heartbeat interval (2.5 s). If the host **suppresses DTR on open**, no reset occurs and **3 s** suffices.
3. On seeing one, **send `^?^`.**
4. **A well-formed `!` carrying all four mandatory keys (§8) confirms the rate.** Anything else — garbage, silence, a malformed descriptor — means try the next rate.

An **unsolicited** `!` confirms the rate just as well, and is the only confirmation a transmit-only device (§9.1) can offer. Since such a device broadcasts its descriptor at least every 5000 ms, the 6 s window above catches one without the host having to ask.

Step 4 is what makes this robust. Noise at a wrong rate will occasionally frame as something record-shaped, but it will essentially never produce a valid descriptor *in response to a `?` the host just sent*. The round trip proves both directions at once.

**Recommended probe order: 115200, then 9600, then 250000** — most likely first. On USB CDC and Bluetooth the first attempt always succeeds, since the rate is ignored, so the majority of modern devices never pay the cost at all.

**Cache the result against `uuid`.** The descriptor's stable identifier makes the probe a once-per-device event rather than a once-per-connection one; a host that has seen a device before should open directly at the remembered rate and fall back to probing only if the handshake fails.

---

## 12. Reference implementation — non-normative

> **This section is illustrative, not normative.** Where this code and the prose of §2–§11 disagree, **the prose governs**, and the conformance vectors of §16 — not this code — define what an implementation must do. It is included because a protocol that claims to fit in a few hundred bytes should show them.
>
> A complete, maintained implementation lives beside this document at [`upline-arduino/src/upline.hpp`](upline-arduino/src/upline.hpp): single header, C++, both directions, with the descriptor exchange, heartbeat, escaping-on-emit, and the transmit-only profile. Verified across AVR, SAMD, RP2040, RP2350 (ARM and RISC-V), ESP32, and Teensy. What follows is only the decode half, in plain C, to keep the algorithm readable.

Single pass, in place, zero allocation, no libc. Validation happens during the structural scan, so `up_unescape` never re-validates. `up_parse` consumes one already-delimited record; the line reader that feeds it is §12.4. Compiled `-Wall -Wextra` and run against the vectors of §16: **41 of 41** record-level vectors against `up_parse`, and **8 of 8** stream-level vectors against the §12.4 reader.

### 12.1 Decoder

```c
#include <stddef.h>

enum { UP_OK = 0, UP_NOT_UPLINE = -1, UP_BAD_ESCAPE = -2,
       UP_TRUNCATED = -3, UP_TRAILING = -4, UP_BAD_KEY = -5 };

/* §5 in place; only ever shrinks. The r[1] guard matters: without it a value
   ending in a lone backslash reads past the NUL. up_parse rejects that first,
   but this must be safe standing alone. */
static void up_unescape(char *s) {
    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '\\' && r[1]) {
            switch (*++r) {
                case 'n': *w++ = '\n'; break;
                case 'r': *w++ = '\r'; break;
                default:  *w++ = *r;   break;      /* \\  \^  \|  \~ */
            }
        } else *w++ = *r;
    }
    *w = '\0';
}

/* §6 non-empty, and the only defined _-prefixed key is _e. */
static int up_key_ok(const char *k) {
    if (!*k) return 0;
    if (*k != '_') return 1;
    return k[1] == 'e' && k[2] == '\0';
}

/* `line` is one record with its terminator already removed (§12.4).
   Mutated in place. `val` is "" for a flag; `is_flag` distinguishes `k` from `k|`. */
int up_parse(char *line, size_t len,
             void (*on_pair)(const char *key, const char *val, int is_flag))
{
    char *p = line, *end = line + len;
    if (len < 1 || *p++ != '^') return UP_NOT_UPLINE;        /* §4 not ours: ignore */

    char *k = p, *v = NULL;
    for (; p < end; p++) {
        if (*p == '\\') {                                    /* §5 */
            if (p + 1 >= end) return UP_BAD_ESCAPE;          /* dangling */
            switch (p[1]) {
                case '\\': case '^': case '|': case '~': case 'n': case 'r':
                    p++; continue;
                default: return UP_BAD_ESCAPE;
            }
        }
        if (*p == '|' && v == NULL) {                        /* §6.1 first bar only */
            *p = '\0'; v = p + 1; continue;
        }
        if (*p == '~' || *p == '^') {
            char term = *p;
            *p = '\0';
            if (*k || v) {                                   /* §6 skip empty segments */
                if (!up_key_ok(k)) return UP_BAD_KEY;
                up_unescape(k);
                if (v) up_unescape(v);
                on_pair(k, v ? v : "", v == NULL);
            }
            if (term == '^')                                 /* §4 must be final byte */
                return (p + 1 == end) ? UP_OK : UP_TRAILING;
            k = p + 1; v = NULL;
        }
    }
    return UP_TRUNCATED;                                     /* no closing caret */
}
```

`_e` is dispatched like any other pair; the decoder does not filter it.

**This is not `strtok`.** `strtok_r(payload, "~")` is not escape-aware and would split at `\~`; the closing caret cannot be found with `strrchr` because `\^` is legal in a value. Any format permitting escaped delimiters forfeits `strtok`. The accurate claim is *single-pass, no-allocation, no-libc scan*.

### 12.2 Fixed-point helpers

These replace `atof` and `dtostrf` for `fixN` (§8.4). Integer arithmetic only. Verified against boundary cases and a round-trip over every scale-2 value in ±200000.

```c
#include <stdint.h>

/* Text -> scaled integer. "-23.456" at scale 2 -> -2346 (round half up).
   Accepts a leading sign, a missing integer part (".5"), and no fraction ("23"). */
int32_t up_fix_parse(const char *s, uint8_t scale) {
    int32_t v = 0; uint8_t neg = 0;
    if (*s == '-') { neg = 1; s++; } else if (*s == '+') s++;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    if (*s == '.') {
        s++;
        while (scale && *s >= '0' && *s <= '9') { v = v * 10 + (*s++ - '0'); scale--; }
        if (*s >= '5' && *s <= '9') v++;               /* round half up */
    }
    while (scale--) v *= 10;                           /* "23" at scale 2 -> 2300 */
    return neg ? -v : v;
}

/* Scaled integer -> text. -2346 at scale 2 -> "-23.46". buf needs 14 bytes. */
char *up_fix_str(int32_t v, uint8_t scale, char *buf) {
    char tmp[12]; uint8_t n = 0; char *w = buf;
    uint32_t u = (v < 0) ? (uint32_t)0 - (uint32_t)v : (uint32_t)v;   /* INT32_MIN safe */
    if (v < 0) *w++ = '-';
    do { tmp[n++] = (char)('0' + (u % 10)); u /= 10; } while (u);
    while (n <= scale) tmp[n++] = '0';                 /* 5 at scale 2 -> "0.05" */
    for (uint8_t i = n; i--; ) {
        *w++ = tmp[i];
        if (scale && i == scale) *w++ = '.';
    }
    *w = '\0';
    return buf;
}
```

The negation is `(uint32_t)0 - (uint32_t)v`, not `-v`: negating `INT32_MIN` as a signed value is undefined behavior, while unsigned arithmetic gives the right magnitude with defined wraparound.

### 12.3 Partial application

The scan dispatches through `on_pair` as it goes, so a late error — `UP_BAD_ESCAPE`, `UP_TRUNCATED`, `UP_TRAILING`, `UP_BAD_KEY` — fires after earlier pairs reached the caller. The record is still rejected, but pairs may already have been applied.

One mitigation is free: **a pair is dispatched only when its terminator (`~` or `^`) is seen**, so a truncated record's final incomplete pair is never applied. `^a|1` dispatches nothing; `^a|1~b` dispatches `a`=`1` and then rejects. Truncation mid-value cannot deliver a half-read value.

Earlier complete pairs still land — safe for telemetry, unsafe for actuator commands. Where that matters, either **stage and commit** (buffer in `on_pair`, apply on `UP_OK`), or **pre-validate framing** with a second escape-aware pass confirming `line[0] == '^'` and that the first unescaped `^` is final. That pass is ~7 cycles/byte, well under 1% of the CPU budget at 115200. A `line[len-1] == '^'` shortcut is **not** sufficient — `^a|1^b|2^` passes it while still being a framing error.

### 12.4 Line reader

The layer above `up_parse`: accumulate bytes, and on any terminator hand over one record. This is the whole of §4 rule 1 — LF, CR, and CRLF all work, and an empty buffer at the second byte of a CRLF pair means no double-dispatch.

```c
#include <stdint.h>
#include <stdbool.h>

extern void on_pair(const char *key, const char *val, int is_flag);   /* yours */

static char    rx[128];              /* the §11 floor */
static uint8_t rxn;
static bool    overlong;             /* skipping the rest of a too-long record */

void up_feed(char c) {                       /* call for every received byte */
    if (c == '\n' || c == '\r') {              /* §4 LF, CR, or CRLF */
        if (overlong) {
            overlong = false;                /* done skipping */
        } else {
            rx[rxn] = '\0';
            if (rxn) up_parse(rx, rxn, on_pair);   /* empty line: nothing to do */
        }
        rxn = 0;
    } else if (overlong) {
        /* keep skipping — see §11 on discarding the remainder */
    } else if (rxn < sizeof(rx) - 1) {
        rx[rxn++] = c;
    } else {
        overlong = true;                     /* too long: skip to the terminator */
        rxn = 0;
    }
}
```

Note there is no trailing-CR special case — that is why this is *smaller* than an LF-only reader, at 98 B against 122 B on ATmega328P. The buffer size is the only policy decision, and it bounds what the device can *receive*, not what it may send: 128 bytes is the §11 floor, chosen to sit just above the 120-byte host-to-device recommendation so a conforming host cannot overflow it. Raising it costs SRAM only. A device's own descriptor and telemetry stream out of this buffer's reach entirely.

Callers MUST drain the transport into `up_feed` inside the deadline of §11.1.

---

## 13. Footprint

avr-gcc 7.3.0, ATmega328P, `-Os -ffunction-sections -fdata-sections -Wl,--gc-sections`, as deltas over a 138 B empty-`main` baseline:

| Component | Flash |
|---|---|
| **`up_parse` + `up_unescape` + `up_key_ok`** | **436 B** |
| `up_feed` line reader (§12.4) | 98 B |
| `up_fix_parse` + `up_fix_str` | 680 B |
| base64url decode, computed alphabet | 250 B |
| `atoi` | 108 B |
| `strtol` | 892 B |
| `atof` | 1,598 B |
| `dtostrf` | 1,606 B |
| float multiply, no text conversion | 448 B |

Guidance:

- **Use `atoi`, not `strtol`** — 8.3× smaller for base-10, and a hand-rolled integer loop is no smaller than avr-libc's assembly.
- **Never link `atof` or `dtostrf`.** The §12.2 helpers do the same work for 680 B rather than 3,204 B.
- **Compute the base64url alphabet, don't table it.** A 64-byte table costs the same flash *and* 66 more bytes of SRAM unless placed in `PROGMEM` — 19.5% of an ATtiny85's 512 B for a table never written to.
- **If a sensor library hands you a `float`,** you still pay ~448 B for float arithmetic but not for the text engine: convert to a scaled integer and format with `up_fix_str` for 1,124 B against 2,006 B via `dtostrf`. Where the sensor offers a raw integer read, skip float entirely.

Whole-sketch totals across boards are in §1. On the smallest targets the binding constraint is **SRAM, not flash**. The ATtiny85 also has **no hardware UART** — only a USI — so serial there is a software implementation that burns CPU and borrows a timer; Upline fits, but whether a given sketch services the line inside the drain deadline is a per-application question.

CPU cost is not a consideration at ordinary rates. A delimiter scan runs about 7 cycles per byte; at 115200 baud on a 16 MHz part there are roughly 1,389 cycles available per byte, so parsing consumes well under 1% of the budget.

---

## 14. Building on Upline

The point of the descriptor is that **a host needs no device-specific code.** One `^?^` yields identity, every key, its type, its default, and its range — enough to render a complete interface, validate input before sending it, and label a data series correctly.

### 14.1 Descriptor to interface

A generic host can map types to controls directly:

| Type | Natural control | Notes |
|---|---|---|
| `bool` | Toggle / checkbox | `default` sets the initial position |
| `fixN` with `min`/`max` | Slider | Step is 10⁻ᴺ; the declared scale is the display precision |
| `fixN` without range | Numeric field | Validate as a decimal, N places |
| `int` with `min`/`max` | Stepper or slider | |
| `int` without range | Numeric field | |
| `str` | Text field | |
| `strs` / `ints` / `fixNs` | List or multi-series plot | |
| `b64` | Binary blob — download, hex view, or decode by convention | |
| `x-*` | Falls back to a text field | Unknown types are `str` (§8.3) |
| Command (flag) | Button | Arguments become a small form |

`min` and `max` are also the validation contract: a host SHOULD refuse to send an out-of-range value rather than rely on the device to reject it, since §10 makes error reporting optional.

### 14.2 State, identity, and liveness

- **State** is just the most recent value seen per key. Every record is a partial update; keys not present are unchanged. Within one record, last occurrence wins (§6).
- **Identity** comes from `uuid`, `name`, and `desc`. `uuid` is the stable key for persisting per-device settings, layouts, and history across reconnects — `name` and `desc` are for display and may change.
- **Liveness** comes from the heartbeat (§9): connected, and stale after 3× the interval. This is a protocol-level signal, so it works identically over a cable, Bluetooth, or a socket.
- **Versioning** comes from `ver`, which a host should check before assuming any grammar beyond this document.
- **Transmit-only devices** (§9.1) never answer `^?^` but broadcast `!` on their own. A host that gets no reply within a descriptor interval should present every key as read-only and hide any controls it would otherwise render.

### 14.3 Logging and replay

An Upline stream is plain text with one record per line, so it can be appended to a file as-is and replayed verbatim through the same parser. No framing metadata is lost, and standard text tooling — `grep`, `tail`, `wc`, a spreadsheet — works on a capture without a decoder.

### 14.4 What Upline deliberately does not provide

Know these before building on it: **no addressing or multiplexing** (one device per stream — see §2 on multi-drop), **no reliability or retransmission** (a lost record is lost; the heartbeat bounds detection), **no authentication or encryption** (add a layer below if the transport is untrusted), **no timestamps** (a host stamps on arrival; a device that needs its own clock declares a key for it), and **no request/response correlation** (there are no message IDs, so a host that issues overlapping commands must track them itself).

---

## 15. Extending Upline

Upline specifies framing, escaping, discovery, and liveness. **Everything above that is the application's.** Four extension points, none requiring a spec change:

**Keys.** Any key not beginning with `_` is yours. Declare it in the descriptor with a type, a default, and a range; nothing else is reserved.

**Commands.** Any first-position flag other than `?` and `!` is an application command, optionally with argument pairs: `^reboot~delay|500^`, `^calibrate^`, `^zero~axis|2^`. Unknown commands MUST be ignored rather than guessed at, so adding one is backward-compatible with older hosts.

**Types.** The `x-` prefix carries vendor types, and a receiver that does not know one treats it as `str` — so an extension degrades to readable text rather than a parse failure.

**Arbitrary payloads.** `b64` carries any byte sequence at all: a packed struct, a firmware chunk, a compressed frame, a nested protocol. base64url's alphabet collides with nothing reserved, so no escaping interacts with it.

Two properties make extension safe rather than merely possible:

- **`ver`** lets a host detect a device speaking a newer grammar before it misparses one.
- **The `_` namespace is reserved and enforced** — receivers reject unknown `_` keys — so future versions can add protocol-level fields without colliding with any application that shipped first.

**Coexistence.** Because lines not starting with `^` are ignored (§4), a device may emit Upline *and* another format on the same link — plain `printf` debugging, or `Label:Value` lines for a plotter — and each consumer sees only what it understands.

---

## 16. Conformance vectors

The vectors are the deliverable: a machine-readable TSV of `id`, `input`, `expected`, run in CI by every implementation.

`expected` is one of three outcomes — a pair list, `REJECT`, or `IGNORE` — deliberately **not** an error code, since §10 leaves `_e` device-defined and asserting on codes would make the file unportable. The trailing comment naming *why* a record is rejected is informative. These vectors confirm that an implementation rejects, not that it rejected for the right reason; implementations distinguishing causes internally should assert those in their own unit tests.

Vectors live in fenced code blocks, never markdown tables — GFM rewrites `\|` to `|` even inside code spans, which would make an escaped bar indistinguishable from a raw one. Notation: bytes are literal except `\x20` for a trailing space; `→` separates input from expectation; `∅` means no pairs.

### 16.1 Well-formed

```text
 1  ^^                                    → ∅                       heartbeat
 2  ^a|1^                                 → a=1
 3  ^a|1~b|2~c|3^                         → a=1, b=2, c=3
 4  ^a|^                                  → a=""                    empty value, not a flag
 5  ^a^                                   → a=<flag>                §7 reads a first-position
                                                                     flag as a command
 6  ^reboot~delay|500^                    → cmd reboot, delay=500
 7  ^a|1~flag~b|2^                        → a=1, flag=<flag>, b=2
 8  ^note|a|b^                            → note=a|b                §6.1 only first bar splits
 9  ^desc|A "quoted", {list}: fine!^      → desc=A "quoted", {list}: fine!
10  ^t|12:34:56^                          → t=12:34:56              colon is not reserved
11  ^p|C:\\tmp^                           → p=C:\tmp                escaped backslash
12  ^v|a\^b^                              → v=a^b
13  ^v|a\~b^                              → v=a~b
14  ^v|a\|b^                              → v=a|b                   escaped bar; same as #8
15  ^v|line1\nline2^                      → v=line1<LF>line2        THE NEWLINE ESCAPE
16  ^v|x\r\ny^                            → v=x<CR><LF>y
17  ^k\^ey|1^                             → key "k^ey" = 1
18  ^k\|ey|1^                             → key "k|ey" = 1          bar in a key MUST be escaped
19  ^v|a~b^                               → v=a, b=<flag>           unescaped ~ always separates
20  ^raw|-_8AAQ^                          → raw=-_8AAQ              base64url needs no escaping
21  ^a|1~a|2^                             → a=2                     §6 last wins
22  ^utf|温度^                             → utf=温度
```

Contrast #8 with #14, and #11 with #32 — those pairs are why this is not a table.

### 16.2 Malformed and edge

```text
23  temp:23.5 rpm:1200                    → IGNORE                  foreign line, NOT an error
24  hello world                           → IGNORE
25  <empty line>                          → IGNORE
26  ^a|1                                  → REJECT                  truncated
27  ^                                     → REJECT                  truncated
28  ^a|1~b                                → REJECT                  truncated
29  ^v|a\qb^                              → REJECT                  \q undefined, §5
30  ^v|a\                                 → REJECT                  dangling escape
31  ^v|a\^                                → REJECT                  valid \^ eats the close caret
32  ^v|a\\^                               → v=a\                    contrast with #31
33  ^~~^                                  → ∅                       empty segments ignored, §6
34  ^~a|1~^                               → a=1                     leading/trailing ~ ignored
35  ^|1^                                  → REJECT                  empty key
36  ^a|1~_z|2^                            → REJECT                  undefined _-prefixed key
37  <over-long record>                    → REJECT                  host→device, past the device
                                                                     buffer (§11); the remainder up
                                                                     to the next terminator is
                                                                     discarded too, not reparsed
38  ^_e|badescape^                        → _e=badescape            the one defined _ key, §10
39  ^_e^                                  → cmd _e                  bare error flag is legal
40  ^a|1^trailing junk                    → REJECT                  §4 rule 5 — NOT a=1
41  ^a|1^b|2^                             → REJECT                  dropped newline is an error
42  ^a|1^\x20                             → REJECT                  one trailing space; no padding
```

The §12 decoder covers 1–36 and 38–42. Three cases sit above it: **37** is the line reader's (the decoder receives a complete line); **21** dispatches both pairs, leaving last-wins to the caller; **5 and 6** rely on position, and `is_flag` carries no position, so command-vs-flag is a caller-level determination.

Verified behavior worth noting: **26 dispatches nothing**, because a pair is emitted only once its terminator is seen (§12.3).

### 16.3 Descriptor and fixed point

```text
43  ^!~uuid|4f2a~name|Greenhouse~desc|North bed~ver|1~temp|fix2||0|100^
      → temp: type=fix2, default=(none), min=0, max=100
44  ^!~uuid|x~name|y~desc|z~ver|1~fan|bool|0^
      → fan: type=bool, default=0
45  ^!~uuid|x~name|y~desc|z~ver|1~tags|strs^
      → tags: type=strs, no default/min/max
46  ^!~uuid|x~name|y~desc|z~ver|1~q|x-custom^
      → q: unknown type → treat as str
47  ^!~uuid|x~name|y~desc|z~ver|1~note|str|a\|b^
      → note: type=str, default=a|b   (sub-fields split on UNESCAPED bars only, §6.1)
48  ^!~uuid|x~name|y~desc|z~ver|1^
      → mandatory keys only; a device with no data keys is valid
49  ^!~name|x~desc|y~ver|1^
      → incomplete descriptor: uuid missing (§8)
50  ^!~uuid|x~name|y~desc|z~ver|1~temp|fix2||-40.00|125.00^
      → temp: type=fix2, scale=2, min=-40.00, max=125.00
51  ^temp|23.45^                          → 2345 at fix2
52  ^temp|23^                             → 2300 at fix2; missing fraction MUST be accepted
53  ^temp|23.456^                         → 2346 at fix2; excess digits round half up
```

### 16.4 Line endings

Stream-level, not record-level: these exercise the reader of §12.4 rather than `up_parse`. `<CR>` and `<LF>` denote single bytes; the expectation is how many records the reader should dispatch.

```text
54  ^a|1^<LF>                             → 1 record                LF
55  ^a|1^<CR><LF>                         → 1 record                CRLF is ONE terminator
56  ^a|1^<CR>                             → 1 record                CR alone
57  ^a|1^<LF>^b|2^<LF>                    → 2 records
58  ^a|1^<CR><LF>^b|2^<CR><LF>            → 2 records               no double-dispatch
59  ^a|1^<CR>^b|2^<CR>                    → 2 records
60  <LF><LF>^a|1^<LF>                     → 1 record                empty lines ignored
61  ^a|1^                                 → 0 records               no terminator: incomplete
```

Vector 61 is not an error — the record is simply unfinished, and a byte-oriented reader cannot know whether more is coming. It is also the failure a user hits when a terminal is set to send no line ending.

---

## 17. Open questions

1. **Read/write direction is not declared.** The descriptor says a key's type and range but not whether a host may write it — so a generic UI cannot distinguish a sensor reading from a setpoint without out-of-band knowledge. The pragmatic convention is that a key carrying a `default` is settable and one without is read-only, but that is convention, not specification. Candidate fixes: a fifth positional sub-field, an access marker on the type, or a separate reserved key. This is the most likely v1 addition.
2. **Strict vs. lenient invalid escapes** (§5). Currently strict-reject; RFC 5424 chose lenient. Strict suits actuators, lenient suits log-style telemetry.
3. **Should `ver` travel outside `!`?** A host cannot learn the version without a successful `?`/`!` exchange — fine for v1, but it forecloses grammar changes. A version suffix on the sync byte (`^1…^`) costs one byte per record forever.
4. **Heartbeat interval discoverability** (§9). 2500 ms is fixed. A battery-powered or shared-bus device may want longer; a `_hb` key in `!` is cheap, but whether receivers would honor it is the question.

---

## Appendix A — why not JSON or MQTT

**JSON.** On an ATmega328P, ArduinoJson v7.4.2 costs **9,680 B — 30% of flash** for work a hand-rolled splitter does in 2,432 B; v6.21.4 costs 6,486 B. The maintainer's own [v7 announcement](https://arduinojson.org/news/2024/01/03/arduinojson-7/) advises staying on v6 when memory is tight, and v7 requires dynamic allocation, which [the v6 docs](https://arduinojson.org/v6/how-to/reduce-memory-usage/) tell ATmega328 users to avoid. RAM scales with document *structure*, not payload — `[1,2,3,4]` is 9 bytes of JSON but 64 bytes of document — and AVR is capped at 255 slots and 255-char strings. On an ATtiny85 it does not compile at all. The tax is memory, not speed: at 115200 baud there are ~1,389 cycles per byte and ArduinoJson uses ~165, so performance cannot justify the 4×.

**MQTT.** For a point-to-point link it is inapplicable, not merely costly: [OASIS MQTT 3.1.1](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.pdf) §4.2 requires an ordered, lossless byte stream to a **Server**, and a cable has no broker and no socket. Where it does apply, Ethernet + PubSubClient measures 16,414 B — 50% of flash — with 256-byte messages; a software TCP stack reaches ~77%; lwIP wants ~40 KB of ROM, more than the whole chip. Real "MQTT over serial" projects have the microcontroller emit plain text while a host daemon owns the broker connection — which is what Upline formalizes.

**Both conclusions are 8-bit-specific.** On a 32-bit part with hundreds of kilobytes, JSON is a rounding error and MQTT is natural. The asymmetry is what settles it: one wire format that works on an ATmega168 also works on an ESP32; the reverse is false.

## Appendix B — prior art

A survey of 25 ASCII line formats — NMEA 0183, G-code/Marlin, SCPI/IEEE 488.2, Modbus ASCII, AT/V.250, logfmt, InfluxDB line protocol, RFC 5424, Graphite, StatsD, Teleplot, Arduino Serial Plotter, Betaflight CLI, FIX, RFC 7464, NDJSON and others — found none combining newline-delimited ASCII, key/value pairs, complete escaping, and binary carriage.

| Format | Where it stops |
|---|---|
| **logfmt** | Only escape is `\"`; a literal backslash is unrepresentable and newline has no escape. No binary. Standardization is an explicit non-goal of its reference implementation. |
| **InfluxDB line protocol** | The most complete escape table surveyed, one character short — *"does not support the newline character in tag or field values."* No bytes type. |
| **RFC 5424 structured data** | Closest registered precedent; escapes `\"` `\\` `\]` and defines invalid-escape behavior. No newline escape, no binary. |
| **FIX tag=value** | Solves key/value, escaping, and binary — via non-printable SOH and length-prefixing, which breaks `readline()` by construction. |

Two findings shaped Upline. **Only 4 of the 25 have a real escape mechanism, and none can escape a newline** — the rest use prohibition (Teleplot: *"avoid `:|` characters"*), destructive substitution (StatsD's `.replace(';','_')`; Datadog silently collapsing distinct labels), or nothing. The costs are documented: [statsd #585](https://github.com/statsd/statsd/issues/585) has been open ten years over a colon in a metric name, and StatsD fragmented into five incompatible tagging dialects because it could not be extended without claiming a new sigil. **And nothing surveyed is self-describing or has a liveness handshake at baseline** — the closest precedent is Sparkplug B's mandatory birth-certificate metric declaration, which `!` adapts to a point-to-point link.

## References

- [RFC 4648](https://www.rfc-editor.org/rfc/rfc4648.html) §5, §3.2 · [RFC 8949](https://www.rfc-editor.org/rfc/rfc8949.html#section-6.1) §6.1 · [RFC 7515](https://www.rfc-editor.org/rfc/rfc7515.html) — unpadded base64url precedent
- [RFC 5424](https://www.rfc-editor.org/rfc/rfc5424.txt) §6.3.3 — invalid-escape behavior
- [MODBUS over Serial Line V1.02](https://www.modbus.org/file/secure/modbusoverserial.pdf) §2.5.2 — normative resynchronization
- [Sparkplug B 3.0.0](https://sparkplug.eclipse.org/specification/version/3.0/documents/sparkplug-specification-3.0.0.pdf) — birth-certificate self-description
- [kr/logfmt](https://pkg.go.dev/github.com/kr/logfmt) · [InfluxDB line protocol](https://docs.influxdata.com/influxdb/v2/reference/syntax/line-protocol/) · [Teleplot](https://github.com/nesnes/teleplot) · [Arduino Serial Plotter protocol](https://github.com/arduino/Arduino/blob/master/build/shared/ArduinoSerialPlotterProtocol.md)
- [ArduinoJson v7 announcement](https://arduinojson.org/news/2024/01/03/arduinojson-7/) · [ArduinoJson-size CI](https://github.com/bblanchon/ArduinoJson-size)
- [ATmega328P datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) Table 19-12 · Arduino AVR core `HardwareSerial.h` / `HardwareSerial_private.h` — ring sizes, silent overflow

---

This work made possible by Small Craft, Inc. and is released under CC0. You are free to use,
extend, modify, redistribute, etc. as you see fit. Keep making awesome things!
