import struct
import threading
from dataclasses import FrozenInstanceError

import pytest

from ur_client_library.exceptions import (
    LifecycleError,
    ReadOnlyInterfaceError,
    RobotErrorCodeError,
    RobotRuntimeError,
    ScriptSyntaxError,
    URCLTimeoutError,
)
from ur_client_library.primary import (
    ErrorCodeMessage,
    KeyMessage,
    PrimaryClient,
    RobotMessageType,
    RobotModeData,
    RobotStateType,
    RuntimeExceptionMessage,
    SafetyModeMessage,
)
from ur_client_library.ur.datatypes import ReportLevel, RobotMode, SafetyMode


class FakeTransport:
    def __init__(self) -> None:
        self.connected = False
        self.connect_count = 0
        self.writes: list[str] = []
        self.callback = None

    def connect(self) -> None:
        self.connected = True
        self.connect_count += 1

    def close(self) -> None:
        self.connected = False

    def read(self, size: int) -> bytes:
        raise AssertionError("These state-machine tests do not start the reader")

    def write(self, data: bytes) -> int:
        assert self.connected
        text = data.decode()
        self.writes.append(text)
        if self.callback is not None:
            self.callback(text)
        return len(data)


class StreamingFakeTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self._condition = threading.Condition()
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        with self._condition:
            self._buffer.extend(data)
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            super().close()
            self._condition.notify_all()

    def read(self, size: int) -> bytes:
        with self._condition:
            self._condition.wait_for(lambda: self._buffer or not self.connected, 1.0)
            if not self._buffer:
                return b""
            chunk = bytes(self._buffer[:size])
            del self._buffer[:size]
            return chunk


class FlakyWriteTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self._fail_next_write = True

    def write(self, data: bytes) -> int:
        if self._fail_next_write:
            self._fail_next_write = False
            return 0
        return super().write(data)


def robot_mode(
    mode: RobotMode = RobotMode.RUNNING,
    *,
    protective_stopped: bool = False,
    program_running: bool = False,
) -> RobotModeData:
    return RobotModeData(
        state_type=RobotStateType.ROBOT_MODE_DATA,
        robot_mode=mode,
        is_protective_stopped=protective_stopped,
        is_program_running=program_running,
    )


def key(title: str, text: str) -> KeyMessage:
    return KeyMessage(0, 0, RobotMessageType.KEY, b"", 0, 0, title, text)


def error(code: int, level: ReportLevel, text: str = "simulated") -> ErrorCodeMessage:
    return ErrorCodeMessage(
        0,
        0,
        RobotMessageType.ERROR_CODE,
        b"",
        code,
        0,
        level,
        0,
        0,
        text,
    )


def connected_client() -> tuple[PrimaryClient, FakeTransport]:
    transport = FakeTransport()
    client = PrimaryClient("127.0.0.1", transport=transport)
    client.connect()
    client.process_package(robot_mode())
    return client, transport


def test_constructor_is_lightweight_and_lifecycle_is_explicit() -> None:
    transport = FakeTransport()
    client = PrimaryClient("127.0.0.1", transport=transport)

    assert not transport.connected
    with pytest.raises(LifecycleError):
        client.start()
    with pytest.raises(LifecycleError):
        client.send_script("textmsg(\"no\")")

    client.connect()
    assert client.is_connected
    client.close()
    client.close()
    assert not client.is_connected


def test_started_reader_parses_frames_into_current_snapshots() -> None:
    transport = StreamingFakeTransport()
    client = PrimaryClient("127.0.0.1", transport=transport)
    received = threading.Event()
    client.subscribe(lambda _: received.set())
    payload = struct.pack(
        ">Q7?bB3d",
        123,
        True,
        True,
        True,
        False,
        False,
        False,
        False,
        RobotMode.IDLE,
        0,
        1.0,
        1.0,
        1.0,
    )
    subpackage = struct.pack(
        ">IB", len(payload) + 5, RobotStateType.ROBOT_MODE_DATA
    ) + payload
    frame = struct.pack(">iB", len(subpackage) + 5, 16) + subpackage

    client.connect()
    client.start()
    transport.feed(frame)

    assert received.wait(1.0)
    assert client.robot_mode == RobotMode.IDLE
    client.close()
    assert not client.is_running


def test_script_upload_reconnects_and_retries_one_failed_write() -> None:
    transport = FlakyWriteTransport()
    client = PrimaryClient("127.0.0.1", transport=transport)
    client.connect()

    client.send_script('textmsg("retry")')

    assert transport.connect_count == 2
    assert transport.writes == ['textmsg("retry")\n']


def test_snapshots_are_immutable_and_callbacks_run_outside_lock() -> None:
    client, _ = connected_client()
    observations = []
    unsubscribe = client.subscribe(lambda package: observations.append(client.robot_mode))

    snapshot = robot_mode(RobotMode.IDLE)
    client.process_package(snapshot)
    unsubscribe()
    client.process_package(robot_mode(RobotMode.RUNNING))

    assert observations == [RobotMode.IDLE]
    assert client.robot_mode == RobotMode.RUNNING
    with pytest.raises(FrozenInstanceError):
        snapshot.robot_mode = RobotMode.POWER_OFF  # type: ignore[misc]


def test_script_feedback_happy_path_and_post_stop_drain() -> None:
    client, transport = connected_client()

    def respond(script: str) -> None:
        if "textmsg" in script:
            client.process_package(key("PROGRAM_XXX_STARTED", "test_fun"))
            client.process_package(key("PROGRAM_XXX_STOPPED", "test_fun"))

    transport.callback = respond

    client.send_script_blocking('textmsg("running")', "test_fun", start_timeout=0.2)

    assert transport.writes == ['def test_fun():\n  textmsg("running")\nend\n\n']


def test_script_feedback_reports_runtime_exception() -> None:
    client, transport = connected_client()

    def respond(_: str) -> None:
        client.process_package(
            RuntimeExceptionMessage(
                0,
                0,
                RobotMessageType.RUNTIME_EXCEPTION,
                b"",
                2,
                3,
                "compile_error_name_not_found",
            )
        )

    transport.callback = respond

    with pytest.raises(RobotRuntimeError, match="line 2"):
        client.send_script_blocking("does_not_exist()", "bad", start_timeout=0.2)


def test_script_feedback_reports_fault_and_stops_program() -> None:
    client, transport = connected_client()

    def respond(script: str) -> None:
        if "movel" in script:
            client.process_package(error(999, ReportLevel.FAULT))

    transport.callback = respond

    with pytest.raises(RobotErrorCodeError, match="C999A0"):
        client.send_script_blocking("movel(p[10,0,0,0,0,0])", "faulty")
    assert transport.writes[-1] == "stop program\n"


def test_warning_policy_and_feedback_draining_apis() -> None:
    client, transport = connected_client()

    def warn_and_stop(script: str) -> None:
        if "protective_stop" in script:
            client.process_package(error(123, ReportLevel.WARNING))
            client.process_package(key("PROGRAM_XXX_STOPPED", "warning"))

    transport.callback = warn_and_stop
    with pytest.raises(RobotErrorCodeError):
        client.send_script_blocking("protective_stop()", "warning")

    client.process_package(error(1, ReportLevel.INFO))
    client.process_package(key("TITLE", "text"))
    client.process_package(
        RuntimeExceptionMessage(
            0, 0, RobotMessageType.RUNTIME_EXCEPTION, b"", 1, 1, "runtime"
        )
    )
    assert len(client.pop_error_codes()) == 1
    assert len(client.pop_key_messages()) == 2
    assert len(client.pop_runtime_exceptions()) == 1
    assert client.pop_error_codes() == ()

    client.send_script_blocking(
        "protective_stop()",
        "warning",
        start_timeout=0.2,
        fail_on_warnings=False,
    )


def test_read_only_interface_restarts_and_retries_once() -> None:
    client, transport = connected_client()
    attempts = 0

    def respond(script: str) -> None:
        nonlocal attempts
        if "textmsg" not in script:
            return
        attempts += 1
        if attempts == 1:
            client.process_package(error(210, ReportLevel.VIOLATION, "read only"))
        else:
            client.process_package(key("PROGRAM_XXX_STARTED", "test_fun"))
            client.process_package(key("PROGRAM_XXX_STOPPED", "test_fun"))

    transport.callback = respond

    client.send_script_blocking('textmsg("ok")', "test_fun", start_timeout=0.2)

    assert attempts == 2
    assert transport.connect_count == 2


def test_read_only_retry_can_be_disabled() -> None:
    client, transport = connected_client()
    transport.callback = lambda _: client.process_package(
        error(210, ReportLevel.VIOLATION, "read only")
    )

    with pytest.raises(ReadOnlyInterfaceError):
        client.send_script_blocking(
            'textmsg("no")',
            "test_fun",
            start_timeout=0.2,
            retry_on_readonly_interface=False,
        )


def test_script_start_timeout_and_secondary_program() -> None:
    client, transport = connected_client()

    with pytest.raises(URCLTimeoutError, match="not started"):
        client.send_script_blocking('textmsg("quiet")', "timeout", start_timeout=0.02)

    client.send_script_blocking('sec test_sec():\n  textmsg("quiet")\nend')
    assert transport.writes[-1].startswith("sec test_sec()")


@pytest.mark.parametrize(
    "script,name",
    [
        ("# comments only\n\n", ""),
        ("def missing_paren:\nend", ""),
        ("def 7_bad():\nend", ""),
        ('textmsg("x")', "0_bad"),
        ('def missing_end():\n  textmsg("x")', ""),
    ],
)
def test_prepare_script_rejects_invalid_scripts(script: str, name: str) -> None:
    with pytest.raises(ScriptSyntaxError):
        PrimaryClient.prepare_script(script, name)


def test_commands_write_exact_primary_tokens_and_validate_snapshots() -> None:
    client, transport = connected_client()

    def update(command: str) -> None:
        if command == "power on\n":
            client.process_package(robot_mode(RobotMode.IDLE))
        elif command == "power off\n":
            client.process_package(robot_mode(RobotMode.POWER_OFF))
        elif command == "set robotmode run\n":
            client.process_package(robot_mode(RobotMode.RUNNING))
        elif command == "set unlock protective stop\n":
            client.process_package(robot_mode(protective_stopped=False))
        elif command == "stop program\n":
            client.process_package(robot_mode(program_running=False))

    transport.callback = update
    client.command_power_on(timeout=0.1)
    client.command_power_off(timeout=0.1)
    client.command_brake_release(timeout=0.1)
    client.process_package(robot_mode(protective_stopped=True))
    client.command_unlock_protective_stop(timeout=0.1)
    client.process_package(robot_mode(program_running=True))
    client.command_stop(timeout=0.1)

    assert transport.writes[-5:] == [
        "power on\n",
        "power off\n",
        "set robotmode run\n",
        "set unlock protective stop\n",
        "stop program\n",
    ]


def test_safety_snapshot_blocks_script_execution() -> None:
    client, _ = connected_client()
    client.process_package(
        SafetyModeMessage(
            0,
            0,
            RobotMessageType.SAFETY_MODE,
            b"",
            0,
            0,
            SafetyMode.PROTECTIVE_STOP,
            0,
            0,
        )
    )

    assert not client.safety_mode_allows_execution()
