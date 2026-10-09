"""Human-readable texts for UR controller error codes (``C<code>A<arg>``).

Mirrors upstream ``ur/error_code_overrides.cpp`` and the lookup in
``ErrorCodeMessage::toString()``: a dynamic override wins, then the exact
``(code, arg)`` entry of the generated table, then the code-only entry.
Placeholders such as ``{float}`` are returned verbatim, as upstream does.
"""

from __future__ import annotations

from ur_client_library.ur._error_code_texts import ERROR_CODE_JSON_VERSION, ERROR_CODE_TEXTS
from ur_client_library.ur.datatypes import RobotMode

__all__ = [
    "ERROR_CODE_JSON_VERSION",
    "ERROR_CODE_TEXTS",
    "error_code_text",
    "error_code_text_override",
]

_INT8_MIN = -128
_INT8_MAX = 127


def _int32(value: int) -> int:
    value &= 0xFFFFFFFF
    return value - (1 << 32) if value & 0x80000000 else value


def error_code_text_override(code: int, arg: int) -> str | None:
    """Return a runtime-computed text for ``code``/``arg``, or ``None``."""
    if code == 100:
        # C100: robot changed mode; the argument encodes the new RobotMode.
        if not _INT8_MIN <= arg <= _INT8_MAX:
            return f"arg {arg} out of range for RobotMode"
        try:
            mode = RobotMode(arg)
        except ValueError:
            return f"Unknown robot mode: {arg}"
        return f"Robot mode changed to: {mode.name}"
    return None


def error_code_text(code: int, arg: int) -> str | None:
    """Return the text for an error code, or ``None`` when no entry is known."""
    override = error_code_text_override(code, arg)
    if override is not None:
        return override
    code = _int32(code)
    arg = _int32(arg)
    # Upstream packs arg -1 as 0xFFFFFFFF, the code-only sentinel, so it lands on
    # the code-only entry; the second lookup covers that case as well.
    text = ERROR_CODE_TEXTS.get((code, arg))
    if text is None:
        text = ERROR_CODE_TEXTS.get((code, None))
    return text
