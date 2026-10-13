"""Binding the control servers on a port another local socket is using.

The default control ports (50001-50004) are inside Linux's ephemeral range,
so an outgoing connection can be handed one of them as its source port. That
blocks bind() with EADDRINUSE despite SO_REUSEADDR, which is what CI hit as
"Could not start reverse interface on port 50004".
"""

from __future__ import annotations

import socket
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager

import pytest

from ur_client_library.control import ReverseInterface
from ur_client_library.control._server import SocketServer
from ur_client_library.exceptions import LifecycleError, URCLConnectionError


def _noop(*_: object) -> None:
    pass


def _server(port: int, **kwargs: float) -> SocketServer:
    server = SocketServer("0.0.0.0", port, 1, **kwargs)
    server.set_callbacks(on_connect=_noop, on_disconnect=_noop, on_message=_noop)
    return server


@contextmanager
def _outgoing_connection() -> Iterator[tuple[int, socket.socket, socket.socket]]:
    """Yield (source_port, client, accepted) for an established local connection.

    The client socket plays the part of an outgoing driver connection (RTDE,
    primary, dashboard) whose kernel-chosen source port collides with a
    control-server port.
    """
    peer = socket.create_server(("127.0.0.1", 0))
    client = socket.socket()
    accepted: socket.socket | None = None
    try:
        client.bind(("127.0.0.1", 0))
        client.connect(peer.getsockname())
        accepted, _ = peer.accept()
        yield client.getsockname()[1], client, accepted
    finally:
        # Closing the accepted side first leaves TIME_WAIT on the peer, not
        # on the client's source port, so the port frees immediately.
        if accepted is not None:
            accepted.close()
        client.close()
        peer.close()


def _release(client: socket.socket, accepted: socket.socket) -> None:
    accepted.close()
    client.recv(1)  # wait for the FIN so the client does not end in TIME_WAIT
    client.close()


def test_start_waits_for_a_port_held_as_a_source_port() -> None:
    with _outgoing_connection() as (port, client, accepted):
        server = _server(port)  # production defaults
        errors: list[BaseException] = []

        def start() -> None:
            try:
                server.start()
            except BaseException as exc:
                errors.append(exc)

        starter = threading.Thread(target=start)
        starter.start()
        time.sleep(0.3)
        assert not errors, f"start() gave up on a busy port: {errors[0]!r}"
        assert starter.is_alive()
        _release(client, accepted)
        starter.join(5.0)
        try:
            assert not errors
            assert server.is_running
            assert server.port == port
        finally:
            server.close()


def test_start_failure_names_the_socket_error() -> None:
    with _outgoing_connection() as (port, _client, _accepted):
        interface = ReverseInterface(
            server=_server(port, bind_timeout=0.2, bind_retry_interval=0.05)
        )
        started = time.monotonic()
        with pytest.raises(URCLConnectionError, match="Address already in use"):
            interface.start()
        assert time.monotonic() - started >= 0.2


def test_close_cancels_a_pending_bind() -> None:
    with _outgoing_connection() as (port, _client, _accepted):
        server = _server(port, bind_timeout=30.0, bind_retry_interval=0.05)
        errors: list[BaseException] = []

        def start() -> None:
            try:
                server.start()
            except BaseException as exc:
                errors.append(exc)

        starter = threading.Thread(target=start)
        starter.start()
        time.sleep(0.2)
        server.close()
        starter.join(2.0)
        assert not starter.is_alive()
        assert len(errors) == 1
        assert isinstance(errors[0], (OSError, LifecycleError))
        assert not server.is_running


def test_restart_on_same_port_while_previous_client_is_still_connected() -> None:
    """The next example can rebind even if the robot still holds the old socket."""
    first = _server(0)
    first.start()
    port = first.port
    robot = socket.create_connection(("127.0.0.1", port))
    try:
        time.sleep(0.1)
        first.close()
        second = _server(port, bind_timeout=0.0)
        second.start()
        assert second.port == port
        second.close()
    finally:
        robot.close()


@pytest.mark.parametrize("value", [-1.0, float("nan"), True])
def test_bind_timing_is_validated(value: float) -> None:
    with pytest.raises((TypeError, ValueError)):
        SocketServer("0.0.0.0", 0, bind_timeout=value)
