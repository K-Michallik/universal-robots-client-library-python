"""Port of upstream 2.15.2 ``tests/test_error_code_message.cpp`` plus logging/generator checks."""

from __future__ import annotations

import importlib.util
import logging
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

from ur_client_library.primary import ErrorCodeMessage, RobotMessageType
from ur_client_library.primary.consumer import PrimaryConsumer
from ur_client_library.ur.datatypes import ReportLevel
from ur_client_library.ur.error_codes import (
    ERROR_CODE_JSON_VERSION,
    ERROR_CODE_TEXTS,
    error_code_text,
    error_code_text_override,
)

ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "src" / "ur_client_library" / "ur" / "_error_code_texts.py"


def make_msg(
    code: int, arg: int, level: ReportLevel = ReportLevel.INFO, text: str = ""
) -> ErrorCodeMessage:
    return ErrorCodeMessage(
        timestamp=0,
        source=0,
        message_type=RobotMessageType.ERROR_CODE,
        message_code=code,
        message_argument=arg,
        report_level=level,
        text=text,
    )


def fallback(code: int, arg: int) -> str:
    return f"C{code}A{arg}"


def load_generator() -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        "generate_error_codes", ROOT / "scripts" / "generate_error_codes.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# ERROR_CODE_JSON_VERSION


def test_version_constant_matches_upstream_2_15_2() -> None:
    assert ERROR_CODE_JSON_VERSION == "40.121.0"


# Generated lookup table


def test_code_with_no_arg_is_present() -> None:
    assert (0, None) in ERROR_CODE_TEXTS


def test_code_with_arg_is_present() -> None:
    assert ERROR_CODE_TEXTS[(4, 1)] == "Communication issue: Lost communication with Controller"


def test_absent_code_is_not_present() -> None:
    assert (0x00FFFFFF, 0) not in ERROR_CODE_TEXTS
    assert (0x00FFFFFF, None) not in ERROR_CODE_TEXTS


def test_table_size_matches_upstream_header() -> None:
    assert len(ERROR_CODE_TEXTS) == 1348


def test_octal_utf8_and_escaped_quote_are_decoded() -> None:
    assert ERROR_CODE_TEXTS[(742, 1)] == (
        "Control Box temperature: The temperature of {signed} °C is close to the limit"
    )
    assert '"euromap67 activated"' in ERROR_CODE_TEXTS[(50, 19)]


# Dynamic override hook


def test_code_100_known_robot_mode() -> None:
    result = error_code_text_override(100, 7)
    assert result is not None
    assert "RUNNING" in result


@pytest.mark.parametrize(
    ("arg", "mode_name"),
    [
        (-1, "NO_CONTROLLER"),
        (0, "DISCONNECTED"),
        (1, "CONFIRM_SAFETY"),
        (2, "BOOTING"),
        (3, "POWER_OFF"),
        (4, "POWER_ON"),
        (5, "IDLE"),
        (6, "BACKDRIVE"),
        (7, "RUNNING"),
        (8, "UPDATING_FIRMWARE"),
    ],
)
def test_code_100_all_defined_robot_modes(arg: int, mode_name: str) -> None:
    assert error_code_text_override(100, arg) == f"Robot mode changed to: {mode_name}"


def test_code_100_unknown_robot_mode_returns_value() -> None:
    assert error_code_text_override(100, 127) == "Unknown robot mode: 127"


def test_code_100_out_of_range_returns_value() -> None:
    assert error_code_text_override(100, 99999) == "arg 99999 out of range for RobotMode"
    assert error_code_text_override(100, -129) == "arg -129 out of range for RobotMode"


def test_unknown_code_override_returns_none() -> None:
    assert error_code_text_override(99999, 0) is None
    assert error_code_text_override(0, 0) is None
    assert error_code_text_override(1, 1) is None


# ErrorCodeMessage.description -- full lookup chain (upstream toString())


def test_description_uses_override_for_code_100() -> None:
    result = make_msg(100, 7).description
    assert "RUNNING" in result
    assert result != fallback(100, 7)


def test_description_uses_override_for_code_100_unknown_arg() -> None:
    assert make_msg(100, 127).description == "C100A127: Unknown robot mode: 127"


def test_description_uses_override_for_code_100_out_of_range_arg() -> None:
    assert (
        make_msg(100, 99999).description
        == "C100A99999: arg 99999 out of range for RobotMode"
    )


def test_description_uses_static_map_exact_match() -> None:
    result = make_msg(4, 1).description
    assert result != fallback(4, 1)
    assert result == "C4A1: Communication issue: Lost communication with Controller"


def test_description_uses_code_only_entry_for_sentinel_arg() -> None:
    assert make_msg(0, -1).description == "C0A-1: No error"


def test_description_uses_code_only_entry_for_arbitrary_arg() -> None:
    assert make_msg(0, 42).description == "C0A42: No error"


def test_description_falls_back_for_unknown_code() -> None:
    assert make_msg(99999, 0).description == fallback(99999, 0)
    assert make_msg(99999, 999).description == fallback(99999, 999)
    assert error_code_text(99999, 0) is None


def test_description_keeps_placeholders_and_controller_text() -> None:
    message = make_msg(742, 1, text="controller payload")
    assert message.description == (
        "C742A1: Control Box temperature: The temperature of {signed} °C is close to the limit"
    )
    assert message.text == "controller payload"
    assert message.to_error_code().description == message.description


# Consumer logging (upstream PrimaryConsumer::consume(ErrorCodeMessage))


@pytest.mark.parametrize(
    ("level", "expected"),
    [
        (ReportLevel.DEBUG, logging.DEBUG),
        (ReportLevel.INFO, logging.INFO),
        (ReportLevel.WARNING, logging.WARNING),
        (ReportLevel.VIOLATION, logging.ERROR),
        (ReportLevel.FAULT, logging.ERROR),
        (ReportLevel.CRITICAL_FAULT, logging.ERROR),
        (ReportLevel.DEVL_DEBUG, logging.DEBUG),
        (ReportLevel.DEVL_INFO, logging.DEBUG),
        (ReportLevel.DEVL_WARNING, logging.DEBUG),
        (ReportLevel.DEVL_VIOLATION, logging.DEBUG),
        (ReportLevel.DEVL_FAULT, logging.DEBUG),
        (ReportLevel.DEVL_CRITICAL_FAULT, logging.DEBUG),
    ],
)
def test_consumer_logs_error_codes_at_mapped_level(
    caplog: pytest.LogCaptureFixture, level: ReportLevel, expected: int
) -> None:
    consumer = PrimaryConsumer()
    with caplog.at_level(logging.DEBUG, logger="ur_client_library.primary.consumer"):
        consumer.consume(make_msg(100, 3, level))
    records = [r for r in caplog.records if r.name == "ur_client_library.primary.consumer"]
    assert [(r.levelno, r.getMessage()) for r in records] == [
        (expected, "C100A3: Robot mode changed to: POWER_OFF")
    ]
    assert len(consumer.pop_error_codes()) == 1


# Generator


def test_header_unescaping_and_key_split() -> None:
    generator = load_generator()
    assert generator.split_key(0x00000000FFFFFFFF) == (0, None)
    assert generator.split_key(0x0000006400000007) == (100, 7)
    assert generator.split_key(0x00000064FFFFFFFE) == (100, -2)
    assert generator.unescape_c_string(r"a \"b\" \302\260C \\") == 'a "b" °C \\'


def test_json_source_and_overlay_follow_upstream_merge_rule(tmp_path: Path) -> None:
    generator = load_generator()
    source = tmp_path / "codes.json"
    source.write_text(
        '{"version": "1.2.3", "codes": ['
        '{"code": 1, "text": "code only"}, {"code": 1, "arg": 2, "text": "base"},'
        ' {"code": 3, "arg": 4}]}',
        encoding="utf-8",
    )
    version, entries = generator.load_json_source(source)
    merged = generator.merge_overlay(
        entries, {"codes": [{"code": 1, "arg": 2, "text": "overlay"}]}
    )
    assert version == "1.2.3"
    assert merged == {(1, None): "code only", (1, 2): "overlay"}
    with pytest.raises(ValueError):
        generator.render_module("1.2.3*/", merged, "test")


def _urcl_checkout() -> Path:
    generator = load_generator()
    urcl: Path = generator.DEFAULT_URCL
    if not urcl.exists():
        pytest.skip(f"URCL checkout not found at {urcl}")
    ref = generator.DEFAULT_REF
    resolved = subprocess.run(
        ["git", "-C", str(urcl), "rev-parse", "--verify", "--quiet", f"{ref}^{{commit}}"],
        capture_output=True,
        check=False,
    )
    if resolved.returncode != 0:
        pytest.skip(f"URCL ref {ref} not available in {urcl}")
    return urcl


def test_checked_in_table_matches_urcl_header() -> None:
    urcl = _urcl_checkout()
    generator = load_generator()
    ref = generator.DEFAULT_REF
    version, entries = generator.parse_header(generator.read_header_at_ref(urcl, ref))
    expected = generator.render_module(
        version, entries, f"URCL {ref}:{generator.HEADER_PATH}"
    )
    assert GENERATED.read_text(encoding="utf-8") == expected
