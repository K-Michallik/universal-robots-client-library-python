from __future__ import annotations

import socket
import struct
import threading
import time
from contextlib import suppress

import pytest

from ur_client_library.exceptions import LifecycleError, RTDEInvalidKeyError
from ur_client_library.rtde import ClientState, DataPackage, PackageHeader, PackageType, RTDEClient


class _FakeRTDEServer:
    def __init__(
        self,
        *,
        unavailable_output: str | None = None,
        conflict_input_once: bool = False,
        unexpected_output_once: bool = False,
        port: int = 0,
    ) -> None:
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", port))
        self.socket.listen()
        self.socket.settimeout(0.1)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.requested_versions: list[int] = []
        self.output_recipe: tuple[str, ...] = ()
        self.unavailable_output = unavailable_output
        self.conflict_input_once = conflict_input_once
        self.unexpected_output_once = unexpected_output_once
        self.output_setup_count = 0
        self.input_setup_count = 0
        self.connection: socket.socket | None = None
        self.connection_lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def drop_connection(self) -> None:
        with self.connection_lock:
            connection = self.connection
        if connection is not None:
            with suppress(OSError):
                connection.shutdown(socket.SHUT_RDWR)
            connection.close()

    def close(self) -> None:
        self.stop.set()
        self.socket.close()
        self.thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                connection, _ = self.socket.accept()
            except (TimeoutError, OSError):
                continue
            connection.settimeout(0.1)
            with self.connection_lock:
                self.connection = connection
            try:
                self._serve(connection)
            finally:
                connection.close()
                with self.connection_lock:
                    if self.connection is connection:
                        self.connection = None

    def _serve(self, connection: socket.socket) -> None:
        protocol_version = 1
        while not self.stop.is_set():
            try:
                prefix = self._read_exact(connection, 2)
            except TimeoutError:
                if self.stop.is_set():
                    return
                continue
            except (EOFError, OSError):
                return
            size = struct.unpack(">H", prefix)[0]
            packet = prefix + self._read_exact(connection, size - 2)
            package_type = packet[2]
            payload = packet[3:]
            if package_type == PackageType.REQUEST_PROTOCOL_VERSION:
                requested = struct.unpack(">H", payload)[0]
                self.requested_versions.append(requested)
                accepted = requested == 1
                if accepted:
                    protocol_version = 1
                self._send(connection, PackageType.REQUEST_PROTOCOL_VERSION, bytes((accepted,)))
            elif package_type == PackageType.GET_URCONTROL_VERSION:
                self._send(
                    connection, PackageType.GET_URCONTROL_VERSION, struct.pack(">4I", 4, 12, 0, 1)
                )
            elif package_type == PackageType.CONTROL_PACKAGE_SETUP_OUTPUTS:
                assert protocol_version == 1
                self.output_setup_count += 1
                if self.unexpected_output_once and self.output_setup_count == 1:
                    self._send(connection, PackageType.TEXT_MESSAGE, b"\x00retry output setup")
                    continue
                self.output_recipe = tuple(payload.decode().split(","))
                field_types = ",".join(
                    "NOT_FOUND" if name == self.unavailable_output else "DOUBLE"
                    for name in self.output_recipe
                ).encode()
                self._send(connection, PackageType.CONTROL_PACKAGE_SETUP_OUTPUTS, field_types)
            elif package_type == PackageType.CONTROL_PACKAGE_SETUP_INPUTS:
                recipe = tuple(payload.decode().split(","))
                self.input_setup_count += 1
                field_types = ",".join(
                    "IN_USE"
                    if self.conflict_input_once and self.input_setup_count == 1
                    else "UINT32"
                    for _ in recipe
                ).encode()
                self._send(
                    connection, PackageType.CONTROL_PACKAGE_SETUP_INPUTS, b"\x01" + field_types
                )
            elif package_type == PackageType.CONTROL_PACKAGE_START:
                self._send(connection, PackageType.CONTROL_PACKAGE_START, b"\x01")
                data = DataPackage(self.output_recipe, protocol_version=1)
                data["timestamp"] = 45.0
                connection.sendall(data.serialize())
            elif package_type == PackageType.CONTROL_PACKAGE_PAUSE:
                self._send(connection, PackageType.CONTROL_PACKAGE_PAUSE, b"\x01")
            elif package_type == PackageType.DATA_PACKAGE:
                pass

    @staticmethod
    def _read_exact(connection: socket.socket, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = connection.recv(size - len(data))
            if not chunk:
                raise EOFError
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _send(connection: socket.socket, package_type: PackageType, payload: bytes) -> None:
        connection.sendall(PackageHeader.serialize(package_type, len(payload)) + payload)


def test_client_negotiates_v2_then_v1_and_reads_in_background() -> None:
    server = _FakeRTDEServer()
    client = RTDEClient(
        "127.0.0.1",
        ["actual_q"],
        ["speed_slider_mask"],
        target_frequency=10.0,
        port=server.port,
    )
    with pytest.raises(LifecycleError):
        client.start()

    client.connect(max_connection_attempts=1, max_initialization_attempts=1)
    assert server.requested_versions == [2, 1]
    assert client.protocol_version == 1
    assert client.version.major == 4
    assert client.max_frequency == 125.0
    assert client.output_recipe == ("actual_q", "timestamp")
    assert client.state is ClientState.INITIALIZED

    client.start(background=True)
    package = client.get_data_package(timeout=1.0)
    assert package is not None
    assert package["timestamp"] == 45.0
    assert package["actual_q"] == (0.0,) * 6
    with pytest.raises(LifecycleError):
        client.read_data_package()

    client.pause()
    assert int(client.state) == int(ClientState.PAUSED)
    client.close()
    client.close()
    server.close()


def test_background_reader_reconnects_with_full_handshake() -> None:
    server = _FakeRTDEServer()
    client = RTDEClient(
        "127.0.0.1",
        ["timestamp"],
        target_frequency=10.0,
        port=server.port,
    )
    client.connect(
        max_connection_attempts=1,
        reconnection_timeout=0.01,
        max_initialization_attempts=3,
        initialization_timeout=0.01,
    )
    client.start()
    assert client.get_data_package(timeout=1.0) is not None

    server.drop_connection()
    deadline = time.monotonic() + 3.0
    while (
        len(server.requested_versions) < 4 or client.state is not ClientState.RUNNING
    ) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.requested_versions == [2, 1, 2, 1]
    assert client.state is ClientState.RUNNING
    assert client.get_data_package(timeout=1.0) is not None

    client.close()
    server.close()


def test_reconnect_does_not_count_socket_outage_as_initialization_failure() -> None:
    server = _FakeRTDEServer()
    port = server.port
    client = RTDEClient(
        "127.0.0.1",
        ["timestamp"],
        target_frequency=10.0,
        port=port,
    )
    client.connect(
        max_connection_attempts=1,
        reconnection_timeout=0.01,
        max_initialization_attempts=2,
        initialization_timeout=0.01,
    )
    client.start()
    assert client.get_data_package(timeout=1.0) is not None

    server.close()
    time.sleep(0.1)
    assert client.get_data_package(timeout=0) is None
    replacement = _FakeRTDEServer(port=port)
    deadline = time.monotonic() + 3.0
    while client.state is not ClientState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.01)

    assert client.state is ClientState.RUNNING
    assert replacement.requested_versions == [2, 1]
    assert client.get_data_package(timeout=1.0) is not None

    client.close()
    replacement.close()


def test_client_removes_unavailable_outputs_with_fresh_handshake() -> None:
    server = _FakeRTDEServer(unavailable_output="target_base_wrench")
    client = RTDEClient(
        "127.0.0.1",
        ["target_base_wrench"],
        ignore_unavailable_outputs=True,
        port=server.port,
    )
    client.connect(max_connection_attempts=1, max_initialization_attempts=2)
    assert client.output_recipe == ("timestamp",)
    assert server.requested_versions == [2, 1, 2, 1]
    client.close()
    server.close()


@pytest.mark.parametrize("recipe_kind", ["output", "input"])
def test_unknown_recipe_key_is_rejected_at_construction(recipe_kind: str) -> None:
    output = ["actual_q", "i_do_not_exist"] if recipe_kind == "output" else ["actual_q"]
    inputs = ["speed_slider_mask", "i_do_not_exist"] if recipe_kind == "input" else []

    with pytest.raises(RTDEInvalidKeyError) as raised:
        RTDEClient("127.0.0.1", output, inputs)

    assert "i_do_not_exist" in str(raised.value)
    assert raised.value.invalid_keys == ("i_do_not_exist",)
    assert isinstance(raised.value, KeyError)


def test_unavailable_output_raises_invalid_key_without_retry() -> None:
    server = _FakeRTDEServer(unavailable_output="target_base_wrench")
    client = RTDEClient("127.0.0.1", ["target_base_wrench"], port=server.port)
    try:
        with pytest.raises(RTDEInvalidKeyError) as raised:
            client.connect(max_connection_attempts=1, max_initialization_attempts=3)

        assert raised.value.invalid_keys == ("target_base_wrench",)
        assert "'target_base_wrench'" in str(raised.value)
        assert server.output_setup_count == 1
        assert server.requested_versions == [2, 1]
        assert client.state is ClientState.UNINITIALIZED
        assert not client._stream.is_connected
    finally:
        client.close()
        server.close()


def test_close_interrupts_unbounded_reconnect() -> None:
    # Regression for upstream #519: close() must cancel an endless reconnect.
    server = _FakeRTDEServer()
    client = RTDEClient("127.0.0.1", ["timestamp"], target_frequency=10.0, port=server.port)
    client.connect(
        max_connection_attempts=0,
        reconnection_timeout=5.0,
        max_initialization_attempts=1,
        initialization_timeout=0.01,
    )
    client.start()
    assert client.get_data_package(timeout=1.0) is not None

    server.close()
    deadline = time.monotonic() + 3.0
    while not client._reconnecting.is_set() and time.monotonic() < deadline:
        time.sleep(0.01)
    # Give the worker time to enter its (5 s) back-off between socket attempts.
    time.sleep(0.2)
    assert client._reconnecting.is_set()
    assert client.state in (ClientState.CONNECTION_LOST, ClientState.INITIALIZING)
    thread = client._reconnect_thread
    assert thread is not None
    assert thread.is_alive()

    started = time.monotonic()
    client.close()
    assert time.monotonic() - started < 2.5
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert client.state is ClientState.UNINITIALIZED


def test_client_retries_transient_input_conflict() -> None:
    server = _FakeRTDEServer(conflict_input_once=True)
    client = RTDEClient(
        "127.0.0.1",
        ["timestamp"],
        ["speed_slider_mask"],
        port=server.port,
    )
    client.connect(max_connection_attempts=1, max_initialization_attempts=2)
    assert client.state is ClientState.INITIALIZED
    assert server.input_setup_count == 2
    client.close()
    server.close()


def test_client_resends_output_setup_after_unexpected_package() -> None:
    server = _FakeRTDEServer(unexpected_output_once=True)
    client = RTDEClient("127.0.0.1", ["timestamp"], port=server.port)
    client.connect(max_connection_attempts=1, max_initialization_attempts=1)

    assert server.output_setup_count == 2
    assert client.state is ClientState.INITIALIZED

    client.close()
    server.close()


def test_client_rejects_non_finite_and_boolean_numeric_options() -> None:
    with pytest.raises(TypeError):
        RTDEClient("127.0.0.1", ["timestamp"], target_frequency=True)
    with pytest.raises(ValueError):
        RTDEClient("127.0.0.1", ["timestamp"], target_frequency=float("nan"))

    client = RTDEClient("127.0.0.1", ["timestamp"])
    with pytest.raises(TypeError):
        client.connect(max_connection_attempts=True)
    client.close()
