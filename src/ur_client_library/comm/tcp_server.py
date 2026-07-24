"""Small callback-driven TCP server used by UR control interfaces."""

from __future__ import annotations

import logging
import math
import selectors
import socket
import threading
from collections.abc import Callable
from contextlib import suppress

from ur_client_library.exceptions import (
    LifecycleError,
    URCLConnectionError,
    WorkerThreadError,
)

LOGGER = logging.getLogger(__name__)
ConnectCallback = Callable[[int], None]
DisconnectCallback = Callable[[int], None]
MessageCallback = Callable[[int, bytes, int], None]


class TCPServer:
    """Explicitly started multi-client TCP server."""

    INPUT_BUFFER_SIZE = 4096

    def __init__(
        self,
        port: int,
        *,
        host: str = "",
        max_attempts: int = 0,
        retry_interval: float = 1.0,
        max_clients: int = 0,
    ) -> None:
        if not isinstance(host, str):
            raise TypeError("host must be a string")
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError("port must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        self._validate_limit(max_attempts, "max_attempts")
        self._validate_limit(max_clients, "max_clients")
        retry_interval = self._validate_retry_interval(retry_interval)
        self._host = host
        self._port = port
        self._max_attempts = max_attempts
        self._retry_interval = retry_interval
        self._max_clients = max_clients
        self._listen_socket: socket.socket | None = None
        self._selector: selectors.BaseSelector | None = None
        self._clients: dict[int, socket.socket] = {}
        self._clients_lock = threading.Lock()
        self._callback_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._connect_callback: ConnectCallback | None = None
        self._disconnect_callback: DisconnectCallback | None = None
        self._message_callback: MessageCallback | None = None
        self._running = threading.Event()
        self._closed = threading.Event()
        self._thread: threading.Thread | None = None
        self._worker_error: WorkerThreadError | None = None
        self._error_lock = threading.Lock()

    @property
    def port(self) -> int:
        return self._port

    @property
    def max_clients(self) -> int:
        return self._max_clients

    @max_clients.setter
    def max_clients(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("max_clients must be a non-negative integer")
        self._max_clients = value

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @property
    def clients(self) -> tuple[socket.socket, ...]:
        self._raise_worker_error()
        with self._clients_lock:
            return tuple(self._clients.values())

    def set_connect_callback(self, callback: ConnectCallback | None) -> None:
        with self._callback_lock:
            self._connect_callback = callback

    def set_disconnect_callback(self, callback: DisconnectCallback | None) -> None:
        with self._callback_lock:
            self._disconnect_callback = callback

    def set_message_callback(self, callback: MessageCallback | None) -> None:
        with self._callback_lock:
            self._message_callback = callback

    def start(self) -> None:
        self._raise_worker_error()
        if self._running.is_set():
            return
        if self._closed.is_set():
            raise LifecycleError("TCP server cannot be restarted after close")
        if self._listen_socket is not None:
            raise LifecycleError("TCP server is already starting")
        listen_socket = socket.socket(socket.AF_INET6 if ":" in self._host else socket.AF_INET)
        try:
            listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError as exc:
            LOGGER.warning("Failed to set server socket option: %s", exc)
        attempts = 0
        while True:
            attempts += 1
            try:
                listen_socket.bind((self._host, self._port))
                break
            except OSError as exc:
                if self._max_attempts and attempts >= self._max_attempts:
                    listen_socket.close()
                    raise URCLConnectionError(
                        f"Failed to bind TCP server to {self._host or '*'}:{self._port}"
                    ) from exc
                if self._closed.wait(self._retry_interval):
                    listen_socket.close()
                    raise URCLConnectionError("TCP server startup was cancelled") from exc
        listen_socket.listen(1)
        listen_socket.setblocking(False)
        self._port = int(listen_socket.getsockname()[1])
        selector = selectors.DefaultSelector()
        selector.register(listen_socket, selectors.EVENT_READ, None)
        self._listen_socket = listen_socket
        self._selector = selector
        self._running.set()
        self._thread = threading.Thread(
            target=self._worker,
            name=f"tcp-server-{self._port}",
            daemon=True,
        )
        self._thread.start()

    def write(
        self,
        client: socket.socket | int,
        data: bytes | bytearray | memoryview,
    ) -> int:
        self._raise_worker_error()
        sock = self._resolve_client(client)
        return self._write_socket(sock, data)

    def write_unchecked(
        self,
        client: socket.socket | int,
        data: bytes | bytearray | memoryview,
    ) -> int:
        if isinstance(client, socket.socket):
            return self._write_socket(client, data)
        try:
            borrowed = socket.socket(fileno=client)
        except OSError as exc:
            raise URCLConnectionError(f"TCP client {client} is not a valid socket") from exc
        try:
            return self._write_socket(borrowed, data)
        finally:
            borrowed.detach()

    def _write_socket(
        self,
        sock: socket.socket,
        data: bytes | bytearray | memoryview,
    ) -> int:
        view = memoryview(data).cast("B")
        written = 0
        with self._write_lock:
            while written < len(view):
                try:
                    sent = sock.send(view[written:])
                except OSError as exc:
                    raise URCLConnectionError("Sending data to TCP client failed") from exc
                if sent <= 0:
                    raise URCLConnectionError("Sending data to TCP client failed")
                written += sent
        return written

    def close(self) -> None:
        self._closed.set()
        self._running.clear()
        listen_socket = self._listen_socket
        if listen_socket is not None:
            with suppress(OSError):
                listen_socket.shutdown(socket.SHUT_RDWR)
            listen_socket.close()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        selector = self._selector
        if selector is not None:
            selector.close()
        with self._clients_lock:
            clients = tuple(self._clients.values())
            self._clients.clear()
        for client in clients:
            client.close()
        self._listen_socket = None
        self._selector = None

    shutdown = close

    def _worker(self) -> None:
        try:
            while self._running.is_set():
                selector = self._selector
                if selector is None:
                    break
                try:
                    events = selector.select(timeout=0.1)
                except (OSError, ValueError):
                    if not self._running.is_set():
                        break
                    raise
                for key, _ in events:
                    if key.data is None:
                        self._accept()
                    else:
                        self._read_client(key.data)
        except BaseException as exc:
            with self._error_lock:
                self._worker_error = WorkerThreadError("TCP server", exc)
            self._running.clear()

    def _accept(self) -> None:
        listen_socket = self._listen_socket
        selector = self._selector
        if listen_socket is None or selector is None:
            return
        try:
            client, _ = listen_socket.accept()
        except OSError:
            if self._running.is_set():
                raise
            return
        with self._clients_lock:
            accepted = self._max_clients == 0 or len(self._clients) < self._max_clients
            if accepted:
                self._clients[client.fileno()] = client
        if not accepted:
            client.close()
            LOGGER.warning("Rejected TCP client because the client limit is reached")
            return
        try:
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError as exc:
            LOGGER.warning("Failed to set accepted socket TCP_NODELAY: %s", exc)
        client.setblocking(False)
        selector.register(client, selectors.EVENT_READ, client)
        callback = self._connect_callback_snapshot()
        if callback is not None:
            callback(client.fileno())

    def _read_client(self, client: socket.socket) -> None:
        try:
            data = client.recv(self.INPUT_BUFFER_SIZE)
        except OSError:
            data = b""
        if data:
            callback = self._message_callback_snapshot()
            if callback is not None:
                callback(client.fileno(), data, len(data))
            return
        self._disconnect(client)

    def _disconnect(self, client: socket.socket) -> None:
        descriptor = client.fileno()
        selector = self._selector
        if selector is not None:
            with suppress(KeyError, ValueError):
                selector.unregister(client)
        with self._clients_lock:
            self._clients.pop(descriptor, None)
        client.close()
        callback = self._disconnect_callback_snapshot()
        if callback is not None and self._running.is_set():
            callback(descriptor)

    def _connect_callback_snapshot(self) -> ConnectCallback | None:
        with self._callback_lock:
            return self._connect_callback

    def _disconnect_callback_snapshot(self) -> DisconnectCallback | None:
        with self._callback_lock:
            return self._disconnect_callback

    def _message_callback_snapshot(self) -> MessageCallback | None:
        with self._callback_lock:
            return self._message_callback

    def _resolve_client(self, client: socket.socket | int) -> socket.socket:
        descriptor = client.fileno() if isinstance(client, socket.socket) else client
        with self._clients_lock:
            resolved = self._clients.get(descriptor)
        if resolved is None:
            raise URCLConnectionError(f"TCP client {descriptor} is not connected")
        return resolved

    def _raise_worker_error(self) -> None:
        with self._error_lock:
            error = self._worker_error
        if error is not None:
            raise error from error.cause

    @staticmethod
    def _validate_limit(value: int, name: str) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} cannot be negative")

    @staticmethod
    def _validate_retry_interval(value: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("retry_interval must be a real number")
        normalized = float(value)
        if not math.isfinite(normalized) or normalized < 0:
            raise ValueError("retry_interval must be finite and non-negative")
        return normalized

    def __enter__(self) -> TCPServer:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
