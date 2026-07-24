"""One-shot URScript command protocol."""

from __future__ import annotations

import logging
import math
import struct
from collections.abc import Callable, Sequence
from enum import IntEnum
from numbers import Real

from ur_client_library.exceptions import (
    IncompatibleRobotVersionError,
    InvalidRangeError,
)
from ur_client_library.helpers import clamp_to_unit_range
from ur_client_library.types import validate_floats, validate_ints
from ur_client_library.ur.tool_communication import ToolVoltage
from ur_client_library.ur.version_information import VersionInformation

from ._serialization import fixed, pack_int32s
from ._server import ClientToken, Server, ServerFactory, socket_server_factory
from .reverse_interface import (
    DEFAULT_SCRIPT_COMMAND_PORT,
    MULT_JOINTSTATE,
    MULT_TIME,
    ReverseInterface,
    ReverseInterfaceConfig,
    Unsubscribe,
)

_LOGGER = logging.getLogger(__name__)


class ToolContactResult(IntEnum):
    SUCCESS = 0
    CANCELED = 1


class ScriptCommand(IntEnum):
    ZERO_FT_SENSOR = 0
    SET_PAYLOAD = 1
    SET_TOOL_VOLTAGE = 2
    START_FORCE_MODE = 3
    END_FORCE_MODE = 4
    START_TOOL_CONTACT = 5
    END_TOOL_CONTACT = 6
    SET_FRICTION_COMPENSATION = 7
    FT_RTDE_INPUT_ENABLE = 8
    SET_GRAVITY = 9
    SET_TCP_OFFSET = 10
    SET_FRICTION_SCALES = 11
    SET_TARGET_PAYLOAD = 12


ToolContactCallback = Callable[[ToolContactResult], None]


def _bounded_real(value: object, lower: float, upper: float, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result) or not lower <= result <= upper:
        raise InvalidRangeError(f"{name} must be in range [{lower:g}, {upper:g}]")
    return result


class ScriptCommandInterface(ReverseInterface):
    """Send the current 28-word script-command packet format."""

    MESSAGE_WORDS = 28

    def __init__(
        self,
        port: int = DEFAULT_SCRIPT_COMMAND_PORT,
        *,
        host: str = "0.0.0.0",
        robot_software_version: VersionInformation | None = None,
        server: Server | None = None,
        server_factory: ServerFactory = socket_server_factory,
    ) -> None:
        super().__init__(
            ReverseInterfaceConfig(
                port=port,
                host=host,
                robot_software_version=robot_software_version or VersionInformation(),
            ),
            server=server,
            server_factory=server_factory,
        )
        self._tool_contact_callbacks: dict[int, ToolContactCallback] = {}
        self._receive_buffers: dict[ClientToken, bytearray] = {}

    def subscribe_tool_contact_result(
        self, callback: ToolContactCallback
    ) -> Unsubscribe:
        callback_id = self._add_callback(self._tool_contact_callbacks, callback)
        return self._unsubscribe(self._tool_contact_callbacks, callback_id)

    def zero_ft_sensor(self) -> None:
        self._send(ScriptCommand.ZERO_FT_SENSOR)

    def set_payload(self, mass: float, cog: Sequence[float]) -> None:
        mass_value = _bounded_real(mass, 0.0, float("inf"), name="mass")
        cog_values = validate_floats(cog, 3, name="cog")
        self._send(
            ScriptCommand.SET_PAYLOAD,
            [
                fixed(mass_value, MULT_JOINTSTATE, name="mass"),
                *(
                    fixed(value, MULT_JOINTSTATE, name=f"cog[{index}]")
                    for index, value in enumerate(cog_values)
                ),
            ],
        )

    def set_target_payload(
        self,
        mass: float,
        cog: Sequence[float],
        inertia: Sequence[float],
        transition_time: float = 0.0,
    ) -> None:
        mass_value = _bounded_real(mass, 0.0, float("inf"), name="mass")
        transition = _bounded_real(
            transition_time, 0.0, float("inf"), name="transition_time"
        )
        cog_values = validate_floats(cog, 3, name="cog")
        inertia_values = validate_floats(inertia, 6, name="inertia")
        values = [fixed(mass_value, MULT_JOINTSTATE, name="mass")]
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"cog[{index}]")
            for index, value in enumerate(cog_values)
        )
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"inertia[{index}]")
            for index, value in enumerate(inertia_values)
        )
        values.append(fixed(transition, MULT_TIME, name="transition_time"))
        self._send(ScriptCommand.SET_TARGET_PAYLOAD, values)

    def set_gravity(self, gravity: Sequence[float]) -> None:
        values = validate_floats(gravity, 3, name="gravity")
        self._send(
            ScriptCommand.SET_GRAVITY,
            [
                fixed(value, MULT_JOINTSTATE, name=f"gravity[{index}]")
                for index, value in enumerate(values)
            ],
        )

    def set_tool_voltage(self, voltage: ToolVoltage | int) -> None:
        if isinstance(voltage, bool):
            raise TypeError("voltage must not be a bool")
        try:
            normalized = ToolVoltage(voltage)
        except ValueError as exc:
            raise ValueError(f"Unsupported tool voltage: {voltage}") from exc
        self._send(
            ScriptCommand.SET_TOOL_VOLTAGE,
            [fixed(int(normalized), MULT_JOINTSTATE, name="tool_voltage")],
        )

    def start_force_mode(
        self,
        task_frame: Sequence[float],
        selection_vector: Sequence[int],
        wrench: Sequence[float],
        force_type: int,
        limits: Sequence[float],
        damping_factor: float = 0.025,
        gain_scaling_factor: float = 0.5,
    ) -> None:
        frame_values = validate_floats(task_frame, 6, name="task_frame")
        selections = validate_ints(selection_vector, 6, name="selection_vector")
        if any(value not in (0, 1) for value in selections):
            raise InvalidRangeError("selection_vector values must be 0 or 1")
        wrench_values = validate_floats(wrench, 6, name="wrench")
        limit_values = validate_floats(limits, 6, name="limits")
        if isinstance(force_type, bool) or not isinstance(force_type, int):
            raise TypeError("force_type must be an integer")
        if not 1 <= force_type <= 3:
            raise InvalidRangeError("force_type must be in range [1, 3]")
        damping = _bounded_real(damping_factor, 0.0, 1.0, name="damping_factor")
        gain = _bounded_real(
            gain_scaling_factor, 0.0, 2.0, name="gain_scaling_factor"
        )
        values: list[int] = []
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"task_frame[{index}]")
            for index, value in enumerate(frame_values)
        )
        values.extend(value * MULT_JOINTSTATE for value in selections)
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"wrench[{index}]")
            for index, value in enumerate(wrench_values)
        )
        values.append(force_type * MULT_JOINTSTATE)
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"limits[{index}]")
            for index, value in enumerate(limit_values)
        )
        values.extend(
            [
                fixed(damping, MULT_JOINTSTATE, name="damping_factor"),
                fixed(gain, MULT_JOINTSTATE, name="gain_scaling_factor"),
            ]
        )
        self._send(ScriptCommand.START_FORCE_MODE, values)

    def end_force_mode(self) -> None:
        self._send(ScriptCommand.END_FORCE_MODE)

    def start_tool_contact(self) -> None:
        self._send(ScriptCommand.START_TOOL_CONTACT)

    def end_tool_contact(self) -> None:
        self._send(ScriptCommand.END_TOOL_CONTACT)

    def set_tcp_offset(self, offset: Sequence[float]) -> None:
        values = validate_floats(offset, 6, name="offset")
        self._send(
            ScriptCommand.SET_TCP_OFFSET,
            [
                fixed(value, MULT_JOINTSTATE, name=f"offset[{index}]")
                for index, value in enumerate(values)
            ],
        )

    def set_friction_compensation(self, enabled: bool) -> None:
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        self._require_version(
            VersionInformation.from_string("5.23.0"),
            VersionInformation.from_string("10.10.0"),
            "set_friction_compensation",
        )
        self._send(ScriptCommand.SET_FRICTION_COMPENSATION, [int(enabled)])

    def set_friction_scales(
        self, viscous_scale: Sequence[float], coulomb_scale: Sequence[float]
    ) -> None:
        self._require_version(
            VersionInformation.from_string("5.25.1"),
            VersionInformation.from_string("10.12.1"),
            "set_friction_scales",
        )
        viscous = validate_floats(viscous_scale, 6, name="viscous_scale")
        coulomb = validate_floats(coulomb_scale, 6, name="coulomb_scale")
        if not all(math.isfinite(value) for value in (*viscous, *coulomb)):
            raise InvalidRangeError("friction scale values must be finite")
        values = [
            fixed(value, MULT_JOINTSTATE, name=f"viscous_scale[{index}]")
            for index, value in enumerate(clamp_to_unit_range(viscous))
        ]
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"coulomb_scale[{index}]")
            for index, value in enumerate(clamp_to_unit_range(coulomb))
        )
        self._send(ScriptCommand.SET_FRICTION_SCALES, values)

    def ft_rtde_input_enable(
        self,
        enabled: bool,
        sensor_mass: float = 0.0,
        sensor_measuring_offset: Sequence[float] = (0.0, 0.0, 0.0),
        sensor_cog: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        mass = _bounded_real(sensor_mass, 0.0, float("inf"), name="sensor_mass")
        offset = validate_floats(
            sensor_measuring_offset, 3, name="sensor_measuring_offset"
        )
        cog = validate_floats(sensor_cog, 3, name="sensor_cog")
        values = [int(enabled), fixed(mass, MULT_JOINTSTATE, name="sensor_mass")]
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"sensor_measuring_offset[{index}]")
            for index, value in enumerate(offset)
        )
        values.extend(
            fixed(value, MULT_JOINTSTATE, name=f"sensor_cog[{index}]")
            for index, value in enumerate(cog)
        )
        self._send(ScriptCommand.FT_RTDE_INPUT_ENABLE, values)

    def _send(self, command: ScriptCommand, values: Sequence[int] = ()) -> None:
        words = [int(command), *values]
        if len(words) > self.MESSAGE_WORDS:
            raise AssertionError("Script-command packet exceeds 28 words")
        words.extend([0] * (self.MESSAGE_WORDS - len(words)))
        self._write_packet(pack_int32s(words))

    def _require_version(
        self,
        minimum_polyscope5: VersionInformation,
        minimum_polyscope_x: VersionInformation,
        command: str,
    ) -> None:
        actual = self.robot_software_version
        required = minimum_polyscope_x if actual.major > 5 else minimum_polyscope5
        if actual < required:
            raise IncompatibleRobotVersionError(
                f"{command} is unavailable on this controller",
                required,
                actual,
            )

    def _connection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client is not None:
                _LOGGER.error("Ignoring a second script-command client")
                return
            self._client = client
            self._receive_buffers[client] = bytearray()

    def _disconnection_callback(self, client: ClientToken) -> None:
        with self._lock:
            if self._client != client:
                return
            self._client = None
            self._receive_buffers.pop(client, None)
            callbacks = tuple(self._disconnection_callbacks.values())
        self._invoke_callbacks(callbacks, client)

    def _message_callback(self, client: ClientToken, data: bytes) -> None:
        with self._lock:
            buffer = self._receive_buffers.setdefault(client, bytearray())
            buffer.extend(data)
            raw_results: list[int] = []
            while len(buffer) >= 4:
                raw_results.append(struct.unpack("!i", buffer[:4])[0])
                del buffer[:4]
            callbacks = tuple(self._tool_contact_callbacks.values())
        for raw_result in raw_results:
            try:
                result = ToolContactResult(raw_result)
            except ValueError:
                _LOGGER.warning("Ignoring unknown tool-contact result %d", raw_result)
                continue
            self._invoke_callbacks(callbacks, result)
