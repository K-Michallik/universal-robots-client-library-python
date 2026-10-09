"""Thread-safe TCP client transport."""

from __future__ import annotations

import logging
import math
import socket
import threading
from collections.abc import Callable
from contextlib import suppress
from enum import Enum

from ur_client_library.exceptions import (
    StreamNotConnectedError,
    URCLConnectionError,
    URCLTimeoutError,
)

LOGGER = logging.getLogger(__name__)


class SocketState(Enum):
    INVALID = "invalid"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    CLOSED = "closed"


class TCPSocket:
    """A lightweight, explicitly connected TCP client.

    Reads and writes have separate locks, so one blocked reader does not prevent
    a writer from sending a command.
    """

    DEFAULT_RECONNECTION_TIME = 10.0

    def __init__(
        self,
        host: str,
        port: int,
        *,
        receive_timeout: float | None = 1.0,
        connect_timeout: float | None = None,
        socket_factory: Callable[[int, int, int], socket.socket] = socket.socket,
    ) -> None:
        if not isinstance(host, str):
            raise TypeError("host must be a string")
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError("port must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        self._validate_timeout(receive_timeout, "receive_timeout")
        self._validate_timeout(connect_timeout, "connect_timeout")
        self._host = host
        self._port = port
        self._receive_timeout = receive_timeout
        self._connect_timeout = connect_timeout
        self._socket_factory = socket_factory
        self._socket: socket.socket | None = None
        self._connecting_socket: socket.socket | None = None
        self._state = SocketState.INVALID
        self._state_lock = threading.Lock()
        self._read_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._cancel = threading.Event()

    @property
    def host(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port

    @property
    def state(self) -> SocketState:
        with self._state_lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        return self.state is SocketState.CONNECTED

    @property
    def local_ip(self) -> str:
        sock = self._connected_socket()
        return str(sock.getsockname()[0])

    def set_receive_timeout(self, timeout: float | None) -> None:
        self._validate_timeout(timeout, "timeout")
        self._receive_timeout = timeout
        sock = self._socket
        if sock is not None:
            sock.settimeout(timeout)

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = DEFAULT_RECONNECTION_TIME,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Connect, retrying until success.

        ``max_attempts=0`` means unlimited attempts.  Calling :meth:`close`, or
        setting ``cancel_event``, interrupts the retry wait.  An explicit
        ``connect()`` clears a previous :meth:`close`.
        """
        retry_interval = self._validate_connect_arguments(max_attempts, retry_interval)
        if self.is_connected:
            return
        self._cancel.clear()
        self._connect_attempts(max_attempts, retry_interval, cancel_event)

    def reconnect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = DEFAULT_RECONNECTION_TIME,
        cancel_event: threading.Event | None = None,
    ) -> None:
        """Re-establish a connection that was lost, honouring a deliberate close.

        Unlike :meth:`connect`, this only proceeds from the lost-connection
        (``DISCONNECTED``) state and never clears a :meth:`close` request, so an
        automatic reconnect cannot resurrect a socket that was closed while it
        was backing off.  Raises :class:`URCLConnectionError` when the socket was
        closed, was never connected, or the attempts fail.
        """
        retry_interval = self._validate_connect_arguments(max_attempts, retry_interval)
        if self._is_connected_or_check_lost():
            return
        try:
            self._connect_attempts(max_attempts, retry_interval, cancel_event)
        except URCLConnectionError:
            with self._state_lock:
                # A failed attempt leaves the socket INVALID; keep it eligible for
                # the next automatic reconnect unless it was closed meanwhile.
                if self._state is SocketState.INVALID and not self._cancel.is_set():
                    self._state = SocketState.DISCONNECTED
            raise

    def _is_connected_or_check_lost(self) -> bool:
        """Return ``True`` when already connected; raise unless the connection was lost."""
        with self._state_lock:
            if self._state is SocketState.CONNECTED:
                return True
            if self._cancel.is_set() or self._state is SocketState.CLOSED:
                raise URCLConnectionError(
                    f"Connection to {self._host}:{self._port} was closed; not reconnecting"
                )
            if self._state is not SocketState.DISCONNECTED:
                raise URCLConnectionError(
                    f"Connection to {self._host}:{self._port} was not lost; "
                    "use connect() to establish it"
                )
            return False

    @property
    def stop_requested(self) -> bool:
        """Whether :meth:`close` was called since the last :meth:`connect`."""
        return self._cancel.is_set()

    def _validate_connect_arguments(self, max_attempts: int, retry_interval: float) -> float:
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int):
            raise TypeError("max_attempts must be an integer")
        if max_attempts < 0:
            raise ValueError("max_attempts cannot be negative")
        validated_retry_interval = self._validate_timeout(retry_interval, "retry_interval")
        assert validated_retry_interval is not None
        return validated_retry_interval

    def _connect_attempts(
        self,
        max_attempts: int,
        retry_interval: float,
        cancel_event: threading.Event | None,
    ) -> None:
        attempts = 0
        last_error: OSError | None = None
        while max_attempts == 0 or attempts < max_attempts:
            if self._is_cancelled(cancel_event):
                with self._state_lock:
                    self._state = SocketState.CLOSED
                raise URCLConnectionError(f"Connection to {self._host}:{self._port} was cancelled")
            attempts += 1
            try:
                addresses = socket.getaddrinfo(
                    self._host or None,
                    self._port,
                    socket.AF_UNSPEC,
                    socket.SOCK_STREAM,
                )
            except OSError as exc:
                last_error = exc
                addresses = []
            for family, socktype, protocol, _, address in addresses:
                candidate: socket.socket | None = None
                try:
                    candidate = self._socket_factory(family, socktype, protocol)
                    with self._state_lock:
                        if self._is_cancelled(cancel_event):
                            self._state = SocketState.CLOSED
                            raise URCLConnectionError(
                                f"Connection to {self._host}:{self._port} was cancelled"
                            )
                        self._connecting_socket = candidate
                    candidate.settimeout(self._connect_timeout)
                    self._connect_candidate(candidate, address, cancel_event)
                    self._configure(candidate)
                    candidate.settimeout(self._receive_timeout)
                    with self._state_lock:
                        self._connecting_socket = None
                        if self._is_cancelled(cancel_event):
                            self._state = SocketState.CLOSED
                            raise URCLConnectionError(
                                f"Connection to {self._host}:{self._port} was cancelled"
                            )
                        previous = self._socket
                        self._socket = candidate
                        self._state = SocketState.CONNECTED
                    if previous is not None:
                        previous.close()
                    return
                except URCLConnectionError:
                    if candidate is not None:
                        candidate.close()
                    raise
                except OSError as exc:
                    last_error = exc
                    if candidate is not None:
                        candidate.close()
                finally:
                    with self._state_lock:
                        if self._connecting_socket is candidate:
                            self._connecting_socket = None
            with self._state_lock:
                self._state = SocketState.INVALID
            if max_attempts != 0 and attempts >= max_attempts:
                break
            if self._wait_cancelled(retry_interval, cancel_event):
                with self._state_lock:
                    self._state = SocketState.CLOSED
                raise URCLConnectionError(f"Connection to {self._host}:{self._port} was cancelled")
        message = f"Failed to connect to {self._host}:{self._port} after {attempts} attempts"
        raise URCLConnectionError(message) from last_error

    def _configure(self, sock: socket.socket) -> None:
        for level, option, value, name in (
            (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1, "TCP_NODELAY"),
        ):
            try:
                sock.setsockopt(level, option, value)
            except OSError as exc:
                LOGGER.warning("Failed to set socket option %s: %s", name, exc)
        quickack = getattr(socket, "TCP_QUICKACK", None)
        if quickack is not None:
            try:
                sock.setsockopt(socket.IPPROTO_TCP, quickack, 1)
            except OSError as exc:
                LOGGER.warning("Failed to set socket option TCP_QUICKACK: %s", exc)

    def _connect_candidate(
        self,
        candidate: socket.socket,
        address: tuple[object, ...] | str | bytes | bytearray | memoryview,
        external_cancel: threading.Event | None,
    ) -> None:
        if external_cancel is None:
            candidate.connect(address)
            return
        completed = threading.Event()

        def cancel_when_requested() -> None:
            while not completed.wait(0.05):
                if self._is_cancelled(external_cancel):
                    candidate.close()
                    return

        watcher = threading.Thread(
            target=cancel_when_requested,
            name="tcp-connect-cancellation",
            daemon=True,
        )
        watcher.start()
        try:
            candidate.connect(address)
        finally:
            completed.set()
            watcher.join(timeout=0.1)

    def receive(self, maximum_bytes: int) -> bytes:
        if isinstance(maximum_bytes, bool) or not isinstance(maximum_bytes, int):
            raise TypeError("maximum_bytes must be an integer")
        if maximum_bytes <= 0:
            raise ValueError("maximum_bytes must be positive")
        with self._read_lock:
            sock = self._connected_socket()
            try:
                data = sock.recv(maximum_bytes)
            except TimeoutError as exc:
                raise URCLTimeoutError(
                    "Timed out while reading from TCP stream", self._receive_timeout
                ) from exc
            except OSError as exc:
                self._mark_disconnected()
                raise URCLConnectionError("Reading from TCP stream failed") from exc
            if not data:
                self._mark_disconnected()
                raise URCLConnectionError("TCP peer closed the connection")
            return data

    def read_exact(self, size: int) -> bytes:
        if isinstance(size, bool) or not isinstance(size, int):
            raise TypeError("size must be an integer")
        if size < 0:
            raise ValueError("size cannot be negative")
        with self._read_lock:
            sock = self._connected_socket()
            data = bytearray()
            while len(data) < size:
                try:
                    chunk = sock.recv(size - len(data))
                except TimeoutError as exc:
                    raise URCLTimeoutError(
                        f"Timed out after reading {len(data)} of {size} bytes",
                        self._receive_timeout,
                    ) from exc
                except OSError as exc:
                    self._mark_disconnected()
                    raise URCLConnectionError(
                        f"Reading {size} bytes from TCP stream failed"
                    ) from exc
                if not chunk:
                    self._mark_disconnected()
                    raise URCLConnectionError(f"TCP peer closed after {len(data)} of {size} bytes")
                data.extend(chunk)
            return bytes(data)

    def write(self, data: bytes | bytearray | memoryview) -> int:
        view = memoryview(data).cast("B")
        with self._write_lock:
            sock = self._connected_socket()
            written = 0
            while written < len(view):
                try:
                    sent = sock.send(view[written:])
                except (TimeoutError, OSError) as exc:
                    self._mark_disconnected()
                    raise URCLConnectionError(
                        f"Sending TCP data failed after {written} of {len(view)} bytes"
                    ) from exc
                if sent <= 0:
                    self._mark_disconnected()
                    raise URCLConnectionError(
                        f"Sending TCP data failed after {written} of {len(view)} bytes"
                    )
                written += sent
            return written

    def close(self) -> None:
        self._cancel.set()
        with self._state_lock:
            sock = self._socket
            connecting = self._connecting_socket
            self._socket = None
            self._connecting_socket = None
            self._state = SocketState.CLOSED
        if connecting is not None and connecting is not sock:
            connecting.close()
        if sock is not None:
            with suppress(OSError):
                sock.shutdown(socket.SHUT_RDWR)
            sock.close()

    disconnect = close

    def _connected_socket(self) -> socket.socket:
        with self._state_lock:
            if self._state is not SocketState.CONNECTED or self._socket is None:
                raise StreamNotConnectedError(
                    f"TCP stream {self._host}:{self._port} is not connected"
                )
            return self._socket

    def _mark_disconnected(self) -> None:
        with self._state_lock:
            if self._state is SocketState.CONNECTED:
                self._state = SocketState.DISCONNECTED

    def _is_cancelled(self, external: threading.Event | None) -> bool:
        return self._cancel.is_set() or (external is not None and external.is_set())

    def _wait_cancelled(self, seconds: float, external: threading.Event | None) -> bool:
        if external is None:
            return self._cancel.wait(seconds)
        # Small slices keep an external cancellation responsive without an
        # extra waiter thread.
        remaining = seconds
        while remaining > 0:
            interval = min(remaining, 0.05)
            if self._cancel.wait(interval) or external.is_set():
                return True
            remaining -= interval
        return self._is_cancelled(external)

    @staticmethod
    def _validate_timeout(value: float | None, name: str) -> float | None:
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be a real number or None")
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0:
            raise ValueError(f"{name} must be finite and non-negative")
        return normalized

    def __enter__(self) -> TCPSocket:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
