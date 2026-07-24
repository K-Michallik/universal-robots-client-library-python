from __future__ import annotations

from pathlib import Path
from typing import cast

import pytest

from test_control_protocols import FakeServer
from ur_client_library.control import ScriptData, ScriptReader, ScriptSender
from ur_client_library.exceptions import (
    InvalidRangeError,
    ScriptSyntaxError,
    ToolCommunicationUnavailableError,
    UnknownVariableError,
)
from ur_client_library.resources import (
    external_control_script_data,
    external_control_script_path,
)
from ur_client_library.ur.tool_communication import Parity, ToolCommSetup, ToolVoltage
from ur_client_library.ur.version_information import VersionInformation


def write(path: Path, contents: str) -> Path:
    path.write_text(contents)
    return path


def test_reader_replaces_upstream_types_and_nested_conditionals(tmp_path: Path) -> None:
    script = write(
        tmp_path / "main.urscript",
        """{{ TEXT }}
{% if VERSION >= v5.23.0 %}
new
{% elif ENABLED %}
fallback
{% else %}
old
{% endif %}
{{ NUMBER }} {{ FLAG }}""",
    )
    reader = ScriptReader()
    result = reader.read_script_file(
        script,
        {
            "TEXT": "header",
            "VERSION": VersionInformation.from_string("5.24.0"),
            "ENABLED": False,
            "NUMBER": 6.28,
            "FLAG": True,
        },
    )
    assert result == "header\nnew\n6.280000 True"


def test_reader_includes_relative_files_and_is_not_jinja(tmp_path: Path) -> None:
    write(tmp_path / "child.urscript", "value={{ VALUE }}")
    main = write(
        tmp_path / "main.urscript",
        """{% include "child.urscript" %}
{{ 1 + 1 }}""",
    )
    result = ScriptReader().read_script_file(main, {"VALUE": 42})
    assert result == "value=42\n{{ 1 + 1 }}"


def test_reader_enforces_restricted_comparison_coercions() -> None:
    data: ScriptData = {"A": True, "X": 5, "S": "hello"}
    assert ScriptReader.evaluate_expression("A == 1", data)
    assert ScriptReader.evaluate_expression("X == 5.0", data)
    assert ScriptReader.evaluate_expression("S == 'hello'", data)
    with pytest.raises(TypeError):
        ScriptReader.evaluate_expression("X < true", data)
    with pytest.raises(ScriptSyntaxError):
        ScriptReader.evaluate_expression("X + 1", data)
    with pytest.raises(UnknownVariableError):
        ScriptReader.evaluate_expression("MISSING", data)


def test_reader_reports_unbalanced_grammar(tmp_path: Path) -> None:
    script = write(tmp_path / "bad.urscript", "{% if FLAG %}\nhello")
    with pytest.raises(ScriptSyntaxError):
        ScriptReader().read_script_file(script, {"FLAG": True})


def test_script_sender_lifecycle_and_fragmented_request() -> None:
    server = FakeServer(50002)
    sender = ScriptSender("def test():\nend", server=server)
    sender.start()
    server.connect_client()
    server.receive(b"request_")
    assert server.writes == []
    server.receive(b"program\n")
    assert server.writes == [b"def test():\nend\n"]
    sender.close()
    assert not sender.is_running


def test_script_sender_rejects_bool_port() -> None:
    with pytest.raises(TypeError):
        ScriptSender("def test():\nend", port=False)


def test_tool_comm_defaults_validation_and_urscript_substitution() -> None:
    setup = ToolCommSetup(
        tool_voltage=ToolVoltage.V24,
        parity=Parity.EVEN,
        baud_rate=115_200,
        stop_bits=2,
        rx_idle_chars=2.5,
        tx_idle_chars=1.5,
    )
    assert setup.to_urscript(VersionInformation.from_string("5.10.0")) == (
        "set_tool_voltage(24)\n"
        "set_tool_communication(True, 115200, 2, 2, 2.5, 1.5)"
    )
    data = external_control_script_data(
        server_ip="192.0.2.1",
        robot_software_version=VersionInformation.from_string("5.25.1"),
        tool_comm_setup=setup,
    )
    assert data["SERVER_PORT_REPLACE"] == "50001"
    assert data["TRAJECTORY_SERVER_PORT_REPLACE"] == "50003"
    assert data["SCRIPT_COMMAND_SERVER_PORT_REPLACE"] == "50004"
    assert data["TIME_REPLACE"] == "1000000"


def test_tool_comm_rejects_invalid_configuration_and_old_versions() -> None:
    with pytest.raises(InvalidRangeError):
        ToolCommSetup(baud_rate=12345)
    with pytest.raises(InvalidRangeError):
        ToolCommSetup(stop_bits=3)
    with pytest.raises(InvalidRangeError):
        ToolCommSetup(rx_idle_chars=0.5)
    with pytest.raises(ToolCommunicationUnavailableError):
        ToolCommSetup().to_urscript(VersionInformation.from_string("3.15.0"))


def test_tool_comm_and_script_data_reject_bool_numeric_inputs() -> None:
    with pytest.raises(TypeError):
        ToolCommSetup(tool_voltage=cast(ToolVoltage, False))
    with pytest.raises(TypeError):
        ToolCommSetup(parity=cast(Parity, False))
    with pytest.raises(TypeError):
        ToolCommSetup(baud_rate=False)
    with pytest.raises(TypeError):
        ToolCommSetup(stop_bits=False)
    with pytest.raises(TypeError):
        ToolCommSetup(rx_idle_chars=False)
    with pytest.raises(TypeError):
        ToolCommSetup(tx_idle_chars=False)

    version = VersionInformation.from_string("5.25.1")
    with pytest.raises(TypeError):
        external_control_script_data(
            server_ip="192.0.2.1",
            robot_software_version=version,
            reverse_port=False,
        )
    with pytest.raises(TypeError):
        external_control_script_data(
            server_ip="192.0.2.1",
            robot_software_version=version,
            servoj_lookahead_time=False,
        )
    with pytest.raises(TypeError):
        external_control_script_data(
            server_ip="192.0.2.1",
            robot_software_version=version,
            servoj_gain=False,
        )


def test_packaged_external_control_script_uses_current_grammar() -> None:
    version = VersionInformation.from_string("5.25.1")
    data = external_control_script_data(
        server_ip="192.0.2.1",
        robot_software_version=version,
    )
    with external_control_script_path() as path:
        rendered = ScriptReader().read_script_file(path, data)
    assert "{{" not in rendered
    assert "{%" not in rendered
    assert "MULT_time = 1000000" in rendered
    assert (
        "direct_torque(torque, viscous_scale=viscous_scale, "
        "coulomb_scale=coulomb_scale)"
    ) in rendered
