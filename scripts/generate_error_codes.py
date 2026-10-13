#!/usr/bin/env python3
"""Generate ``ur_client_library/ur/_error_code_texts.py`` from URCL error-code data.

Upstream URCL (2.15.1+, #564) generates ``include/ur_client_library/ur/
error_code_texts.h`` from UR's online ErrorCodes JSON. This port does not
download anything: by default it reads that generated header from a local URCL
checkout at a pinned git ref, so the Python table matches the C++ table
byte-for-byte::

    scripts/generate_error_codes.py --urcl ../Universal_Robots_Client_Library --ref 2.15.2

Alternatively, a locally saved UR ErrorCodes JSON can be used, optionally
merged with an overlay JSON using upstream's rule (entries without ``text`` are
skipped; overlay entries win on ``(code, arg)`` collision)::

    scripts/generate_error_codes.py --json ErrorCodes.json --overlay overrides.json

Entries without an argument (C++ sentinel ``0xFFFFFFFF``) are keyed with
``arg=None``. Placeholders such as ``{float}`` are kept verbatim, as upstream
does.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "src" / "ur_client_library" / "ur" / "_error_code_texts.py"
DEFAULT_URCL = Path(
    os.environ.get("URCL_SOURCE_DIR", REPO_ROOT.parent / "Universal_Robots_Client_Library")
)
DEFAULT_REF = "2.15.2"
HEADER_PATH = "include/ur_client_library/ur/error_code_texts.h"

NO_ARG_SENTINEL = 0xFFFFFFFF

Key = tuple[int, int | None]

_VERSION_RE = re.compile(r"^[0-9]+\.[0-9]+\.[0-9]+\Z")
_HEADER_VERSION_RE = re.compile(r'ERROR_CODE_JSON_VERSION\s*=\s*"([^"]+)"')
_HEADER_ENTRY_RE = re.compile(r'\{ UINT64_C\(0x([0-9A-F]{16})\), "((?:[^"\\]|\\.)*)" \}')
_C_ESCAPE_RE = re.compile(r"\\(?:([0-7]{1,3})|x([0-9A-Fa-f]+)|(.))", re.DOTALL)
_C_NAMED_ESCAPES = {
    "\\": 0x5C,
    '"': 0x22,
    "'": 0x27,
    "?": 0x3F,
    "n": 0x0A,
    "r": 0x0D,
    "t": 0x09,
    "a": 0x07,
    "b": 0x08,
    "f": 0x0C,
    "v": 0x0B,
}


def validate_version(version: str, label: str = "version") -> None:
    if not _VERSION_RE.match(version):
        raise ValueError(f"Unexpected {label} format: {version!r}; expected MAJOR.MINOR.PATCH")


def _signed32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value - (1 << 32) if value & 0x80000000 else value


def split_key(key: int) -> Key:
    """Split upstream's packed ``(code << 32) | arg`` key into ``(code, arg | None)``."""
    code = _signed32(key >> 32)
    arg_bits = key & 0xFFFFFFFF
    return code, None if arg_bits == NO_ARG_SENTINEL else _signed32(arg_bits)


def unescape_c_string(literal: str) -> str:
    """Decode a C string-literal body whose non-ASCII text is octal UTF-8 bytes."""
    data = bytearray()
    position = 0
    for match in _C_ESCAPE_RE.finditer(literal):
        data.extend(literal[position : match.start()].encode("ascii"))
        octal, hexadecimal, named = match.groups()
        if octal is not None:
            data.append(int(octal, 8) & 0xFF)
        elif hexadecimal is not None:
            data.append(int(hexadecimal, 16) & 0xFF)
        elif named in _C_NAMED_ESCAPES:
            data.append(_C_NAMED_ESCAPES[named])
        else:
            raise ValueError(f"Unsupported C escape sequence \\{named!s}")
        position = match.end()
    data.extend(literal[position:].encode("ascii"))
    return data.decode("utf-8")


def parse_header(content: str) -> tuple[str, dict[Key, str]]:
    """Return the JSON version and entries of a generated ``error_code_texts.h``."""
    version_match = _HEADER_VERSION_RE.search(content)
    if version_match is None:
        raise ValueError("ERROR_CODE_JSON_VERSION not found in header")
    version = version_match.group(1)
    validate_version(version, "header ERROR_CODE_JSON_VERSION")
    entries: dict[Key, str] = {}
    for match in _HEADER_ENTRY_RE.finditer(content):
        entries[split_key(int(match.group(1), 16))] = unescape_c_string(match.group(2))
    if not entries:
        raise ValueError("No error-code entries found in header")
    expected = content.count("UINT64_C(0x")
    if len(entries) != expected:
        raise ValueError(f"Parsed {len(entries)} header entries, expected {expected}")
    return version, entries


def read_header_at_ref(urcl: Path, ref: str) -> str:
    return subprocess.run(
        ["git", "-C", str(urcl), "show", f"{ref}:{HEADER_PATH}"],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout


def _json_entries(data: Mapping[str, Any]) -> Iterable[tuple[Key, str]]:
    for entry in data.get("codes", []):
        if "text" not in entry:
            continue
        code = int(entry["code"])
        arg = int(entry["arg"]) if "arg" in entry else None
        # Same packing as upstream: an explicit arg of -1 aliases the no-arg sentinel.
        arg_bits = NO_ARG_SENTINEL if arg is None else arg & 0xFFFFFFFF
        yield split_key(((code & 0xFFFFFFFF) << 32) | arg_bits), str(entry["text"])


def merge_overlay(entries: dict[Key, str], overlay: Mapping[str, Any] | None) -> dict[Key, str]:
    merged = dict(entries)
    if overlay:
        merged.update(_json_entries(overlay))
    return merged


def load_json_source(path: Path) -> tuple[str, dict[Key, str]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    version = str(data.get("version", ""))
    validate_version(version, "UR JSON 'version' field")
    return version, dict(_json_entries(data))


def _sort_key(item: tuple[Key, str]) -> int:
    (code, arg), _ = item
    arg_bits = NO_ARG_SENTINEL if arg is None else arg & 0xFFFFFFFF
    return ((code & 0xFFFFFFFF) << 32) | arg_bits


def render_module(version: str, entries: Mapping[Key, str], source: str) -> str:
    """Render the checked-in Python module; output is deterministic for a given input."""
    validate_version(version)
    lines = [
        '"""UR controller error-code texts.',
        "",
        "AUTO-GENERATED by scripts/generate_error_codes.py -- DO NOT EDIT MANUALLY.",
        f"Source: {source} (UR ErrorCodes JSON v{version}).",
        "",
        "Keys are ``(code, arg)``; ``arg`` is ``None`` for code-only entries.",
        '"""',
        "",
        "from __future__ import annotations",
        "",
        "from collections.abc import Mapping",
        "from types import MappingProxyType",
        "from typing import Final",
        "",
        f"ERROR_CODE_JSON_VERSION: Final = {version!r}",
        "",
        "ERROR_CODE_TEXTS: Final[Mapping[tuple[int, int | None], str]] = MappingProxyType({",
    ]
    for (code, arg), text in sorted(entries.items(), key=_sort_key):
        lines.append(f"    ({code}, {arg}): {text!r},")
    lines.append("})")
    lines.append("")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0] if __doc__ else None)
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--urcl",
        type=Path,
        default=DEFAULT_URCL,
        help="URCL git checkout to read the generated header from "
        "(default: $URCL_SOURCE_DIR or ../Universal_Robots_Client_Library)",
    )
    source.add_argument("--json", type=Path, help="Local UR ErrorCodes JSON file instead")
    parser.add_argument("--ref", default=DEFAULT_REF, help=f"URCL git ref (default {DEFAULT_REF})")
    parser.add_argument("--overlay", type=Path, help="Overlay JSON; its entries win on collision")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    if args.json is not None:
        version, entries = load_json_source(args.json)
        description = f"UR ErrorCodes JSON {args.json.name}"
    else:
        version, entries = parse_header(read_header_at_ref(args.urcl, args.ref))
        description = f"URCL {args.ref}:{HEADER_PATH}"
    overlay = json.loads(args.overlay.read_text(encoding="utf-8")) if args.overlay else None
    entries = merge_overlay(entries, overlay)

    args.output.write_text(render_module(version, entries, description), encoding="utf-8")
    print(f"Wrote {len(entries)} entries (UR JSON v{version}) to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
