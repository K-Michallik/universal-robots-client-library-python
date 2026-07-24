from __future__ import annotations

import struct
from typing import cast

import pytest

from test_control_protocols import FakeServer, unpack
from ur_client_library.control import ScriptCommand, ScriptCommandInterface, ToolContactResult
from ur_client_library.exceptions import IncompatibleRobotVersionError, InvalidRangeError
from ur_client_library.ur.tool_communication import ToolVoltage
from ur_client_library.ur.version_information import VersionInformation


def connected_interface(
    version: str = "99.99.9",
) -> tuple[ScriptCommandInterface, FakeServer]:
    server = FakeServer(50004)
    interface = ScriptCommandInterface(
        robot_software_version=VersionInformation.from_string(version),
        server=server,
    )
    server.connect_client()
    return interface, server


def test_set_target_payload_is_command_12_with_current_time_scaling() -> None:
    interface, server = connected_interface()
    interface.set_target_payload(
        1.0,
        (0.2, 0.3, 0.1),
        (0.4, 0.7, 0.8, 0.2, 0.5, 0.6),
        0.002,
    )
    words = unpack(server.writes[-1])
    assert len(words) == 28
    assert words[:12] == (
        ScriptCommand.SET_TARGET_PAYLOAD,
        1_000_000,
        200_000,
        300_000,
        100_000,
        400_000,
        700_000,
        800_000,
        200_000,
        500_000,
        600_000,
        2_000,
    )
    assert words[12:] == (0,) * 16


def test_all_script_commands_use_fixed_28_word_packets() -> None:
    interface, server = connected_interface()
    interface.zero_ft_sensor()
    interface.set_payload(1, (0, 0, 0))
    interface.set_tool_voltage(ToolVoltage.V12)
    interface.end_force_mode()
    interface.start_tool_contact()
    interface.end_tool_contact()
    interface.ft_rtde_input_enable(True)
    interface.set_gravity((0, 0, -9.81))
    interface.set_tcp_offset((0, 0, 0, 0, 0, 0))
    interface.set_friction_compensation(True)
    interface.set_friction_scales((2, -1, 0.5, 0, 1, 0.2), (0,) * 6)

    assert all(len(packet) == 112 for packet in server.writes)
    friction_words = unpack(server.writes[-1])
    assert friction_words[:7] == (
        ScriptCommand.SET_FRICTION_SCALES,
        1_000_000,
        0,
        500_000,
        0,
        1_000_000,
        200_000,
    )


def test_force_mode_validates_documented_ranges() -> None:
    interface, _ = connected_interface()
    with pytest.raises(InvalidRangeError):
        interface.start_force_mode(
            (0,) * 6,
            (0, 0, 2, 0, 0, 0),
            (0,) * 6,
            2,
            (0.1,) * 6,
        )
    with pytest.raises(InvalidRangeError):
        interface.start_force_mode(
            (0,) * 6,
            (0, 0, 1, 0, 0, 0),
            (0,) * 6,
            4,
            (0.1,) * 6,
        )


def test_tool_voltage_rejects_bool_coercion() -> None:
    interface, _ = connected_interface()
    with pytest.raises(TypeError):
        interface.set_tool_voltage(cast(ToolVoltage, False))


@pytest.mark.parametrize(
    ("version", "method"),
    [
        ("5.22.0", "set_friction_compensation"),
        ("10.9.0", "set_friction_compensation"),
        ("5.25.0", "set_friction_scales"),
        ("10.12.0", "set_friction_scales"),
    ],
)
def test_version_gated_commands_reject_old_controllers(
    version: str, method: str
) -> None:
    interface, _ = connected_interface(version)
    with pytest.raises(IncompatibleRobotVersionError):
        if method == "set_friction_compensation":
            interface.set_friction_compensation(True)
        else:
            interface.set_friction_scales((0,) * 6, (0,) * 6)


def test_tool_contact_callbacks_are_fragment_safe_and_unsubscribable() -> None:
    interface, server = connected_interface()
    results: list[ToolContactResult] = []
    unsubscribe = interface.subscribe_tool_contact_result(results.append)
    payload = struct.pack("!ii", ToolContactResult.SUCCESS, ToolContactResult.CANCELED)
    server.receive(payload[:5])
    server.receive(payload[5:])
    assert results == [ToolContactResult.SUCCESS, ToolContactResult.CANCELED]
    unsubscribe()
    server.receive(struct.pack("!i", ToolContactResult.SUCCESS))
    assert results == [ToolContactResult.SUCCESS, ToolContactResult.CANCELED]
