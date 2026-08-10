#!/usr/bin/env python3
"""Extract the conformance vectors from ../README.md into vectors.tsv.

The markdown is the source of truth — it carries the reasoning alongside each
case. This produces the machine-readable form so an implementation's test
harness does not have to parse markdown, and so the two cannot drift: run it in
CI and diff the result.

Columns: id, kind, input, scale, expected, note

`scale` is filled only for kind `num`, where the markdown writes the scale inline
("23.45  at fix2"); splitting it out keeps `input` the literal text under test.

`kind` says which function is under test, because the vectors do not all exercise
the same one:

  line    one line in  -> the entries a receiver dispatches
  schema  one ? response in -> the declarations a host derives
  num     text + scale -> scaled integer, or REJECT (§8.4)
  stream  raw bytes in -> how many lines the reader emits
  limit   parameterised by the device's _r; an implementation supplies the line

Metasyntax is passed through verbatim from the markdown: <LF>, <CR>, \\x20 for a
trailing space, <empty>, and <line longer than _r>.  No vector input contains a
tab, so the TSV needs no escaping of its own.
"""

import re
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "README.md"
OUT = Path(__file__).with_name("vectors.tsv")

KIND = {
    "13.1": "line", "13.2": "line", "13.3": "line", "13.4": "line",
    "13.5": "schema", "13.6": "num", "13.7": "stream", "13.8": "limit",
}

VECTOR = re.compile(r"^\s*(\d+)\s\s+(.*)$")
HEADING = re.compile(r"^### (13\.\d)\b")


def main() -> int:
    kind = None
    in_block = False
    rows: list[list[str]] = []

    for raw in SRC.read_text(encoding="utf-8").splitlines():
        h = HEADING.match(raw)
        if h:
            kind = KIND.get(h.group(1))
            continue
        if raw.startswith("```"):
            in_block = not in_block
            continue
        if not in_block or kind is None:
            continue

        m = VECTOR.match(raw)
        if m:
            rows.append([m.group(1), kind, m.group(2).rstrip()])
        elif rows and raw.strip() and not raw[:1].strip():
            # A continuation of the vector above — fold it onto one line.
            rows[-1][2] += " " + raw.strip()

    out = ["\t".join(("id", "kind", "input", "scale", "expected", "note"))]
    at_fix = re.compile(r"^(.*?)\s+at\s+fix(\d)$")
    for vid, k, body in rows:
        if "→" not in body:
            print(f"vector {vid}: no arrow, skipped", file=sys.stderr)
            continue
        lhs, rhs = body.split("→", 1)
        scale = ""
        f = at_fix.match(lhs.strip())
        if k == "num" and f:
            lhs, scale = f.group(1), f.group(2)
        # expected is the first run of non-blank text; two or more spaces start the note
        parts = re.split(r"\s{2,}", rhs.strip(), maxsplit=1)
        expected = parts[0].strip()
        note = parts[1].strip() if len(parts) > 1 else ""
        out.append("\t".join((vid, k, lhs.strip(), scale, expected, note)))

    OUT.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"{len(out) - 1} vectors -> {OUT.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
