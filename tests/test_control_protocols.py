from __future__ import annotations

import socket
import struct
import threading
from collections.abc import Callable
from typing import cast

import pytest

from ur_client_library.comm.control_mode import ControlMode
from ur_client_library.control import (
    FreedriveControlMessage,
    ReverseInterface,
    ReverseInterfaceConfig,
    TrajectoryControlMessage,
)
from ur_client_library.control._server import SocketServer
from ur_client_library.exceptions import InvalidRangeError, StreamNotConnectedError


class FakeServer:
    def __init__(self, port: int) -> None:
        self.port = port
        self.is_running = False
        self.client = object()
        self.writes: list[bytes] = []

    def set_callbacks(
        self,
        *,
        on_connect: Callable[[object], None],
        on_disconnect: Callable[[object], None],
        on_message: Callable[[object, bytes], None],
    ) -> None:
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.on_message = on_message

    def start(self) -> None:
        self.is_running = True

    def write(self, client: object, data: bytes) -> None:
        assert client is self.client
        self.writes.append(data)

    def close(self) -> None:
        self.is_running = False

    def connect_client(self) -> None:
        self.on_connect(self.client)

    def disconnect_client(self) -> None:
        self.on_disconnect(self.client)

    def receive(self, data: bytes) -> None:
        self.on_message(self.client, data)


def unpack(packet: bytes) -> tuple[int, ...]:
    return struct.unpack(f"!{len(packet) // 4}i", packet)


def test_reverse_packet_is_exact_big_endian_and_uses_cpp_rounding() -> None:
    server = FakeServer(50001)
    interface = ReverseInterface(
        ReverseInterfaceConfig(step_time_ms=2),
        server=server,
    )
    with pytest.raises(StreamNotConnectedError):
        interface.write()

    server.connect_client()
    interface.write(
        (0.0000005, -0.0000005, 1, -2, 3.25, -4.25),
        ControlMode.SERVOJ,
    )

    assert len(server.writes[-1]) == 32
    assert unpack(server.writes[-1]) == (
        20,
        1,
        -1,
        1_000_000,
        -2_000_000,
        3_250_000,
        -4_250_000,
        1,
    )


def test_reverse_control_messages_have_fixed_zero_padding() -> None:
    server = FakeServer(50001)
    interface = ReverseInterface(server=server)
    server.connect_client()

    interface.write_trajectory_control_message(TrajectoryControlMessage.STREAM_END, 42)
    assert unpack(server.writes[-1]) == (200, 3, 42, 0, 0, 0, 0, 3)

    interface.write_freedrive_control_message(FreedriveControlMessage.START)
    assert unpack(server.writes[-1]) == (200, 1, 0, 0, 0, 0, 0, 6)


def test_reverse_callbacks_run_and_unsubscribe_is_idempotent() -> None:
    server = FakeServer(50001)
    states: list[bool] = []
    disconnected: list[object] = []
    interface = ReverseInterface(server=server)
    interface.subscribe_program_state(states.append)
    unsubscribe = interface.subscribe_disconnection(disconnected.append)

    interface.start()
    server.connect_client()
    server.disconnect_client()
    assert states == [False, True, False]
    assert disconnected == [server.client]

    unsubscribe()
    unsubscribe()
    server.connect_client()
    server.disconnect_client()
    assert disconnected == [server.client]


def test_reverse_rejects_values_outside_signed_int32_wire_range() -> None:
    server = FakeServer(50001)
    interface = ReverseInterface(server=server)
    server.connect_client()
    with pytest.raises(InvalidRangeError):
        interface.write((3_000, 0, 0, 0, 0, 0))


def test_reverse_rejects_bool_for_integer_and_enum_inputs() -> None:
    with pytest.raises(TypeError):
        ReverseInterfaceConfig(port=False)
    with pytest.raises(TypeError):
        SocketServer("127.0.0.1", False)
    with pytest.raises(TypeError):
        SocketServer("127.0.0.1", 0, max_clients=False)

    server = FakeServer(50001)
    interface = ReverseInterface(server=server)
    server.connect_client()
    with pytest.raises(TypeError):
        interface.write(control_mode=cast(ControlMode, False))
    with pytest.raises(TypeError):
        interface.write_trajectory_control_message(
            cast(TrajectoryControlMessage, False)
        )
    with pytest.raises(TypeError):
        interface.write_freedrive_control_message(
            cast(FreedriveControlMessage, False)
        )


def test_socket_server_lifecycle_and_tcp_nodelay() -> None:
    accepted: list[socket.socket] = []
    connected = threading.Event()
    server = SocketServer("127.0.0.1", 0)

    def on_connect(client: object) -> None:
        assert isinstance(client, socket.socket)
        accepted.append(client)
        connected.set()

    def on_disconnect(_client: object) -> None:
        pass

    def on_message(_client: object, _data: bytes) -> None:
        pass

    server.set_callbacks(
        on_connect=on_connect,
        on_disconnect=on_disconnect,
        on_message=on_message,
    )
    server.start()
    server.start()
    try:
        with socket.create_connection(("127.0.0.1", server.port), timeout=1):
            assert connected.wait(1)
            assert (
                accepted[0].getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) == 1
            )
            assert server.is_running
    finally:
        server.close()
        server.close()
    assert not server.is_running
