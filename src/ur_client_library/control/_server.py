"""Injectable TCP server contract and a small standard-library implementation."""

from __future__ import annotations

import errno
import logging
import math
import socket
import threading
import time
from collections.abc import Callable
from contextlib import suppress
from typing import Protocol, TypeAlias

from ur_client_library.exceptions import LifecycleError, URCLConnectionError

_LOGGER = logging.getLogger(__name__)

ClientToken: TypeAlias = object
ConnectCallback: TypeAlias = Callable[[ClientToken], None]
DisconnectCallback: TypeAlias = Callable[[ClientToken], None]
MessageCallback: TypeAlias = Callable[[ClientToken, bytes], None]


class Server(Protocol):
    """Transport expected by the external-control interfaces."""

    @property
    def port(self) -> int: ...

    @property
    def is_running(self) -> bool: ...

    def set_callbacks(
        self,
        *,
        on_connect: ConnectCallback,
        on_disconnect: DisconnectCallback,
        on_message: MessageCallback,
    ) -> None: ...

    def start(self) -> None: ...

    def write(self, client: ClientToken, data: bytes) -> None: ...

    def close(self) -> None: ...


ServerFactory: TypeAlias = Callable[[str, int, int], Server]


class SocketServer:
    """Threaded TCP server used when no communication-layer server is injected.

    The default control ports (50001-50004) sit inside Linux's ephemeral port
    range. An outgoing connection that the kernel gave one of them as its
    source port blocks ``bind()`` with ``EADDRINUSE`` even with SO_REUSEADDR,
    and keeps blocking it for the 60 s TIME_WAIT after that connection closes.
    Like upstream ``TCPServer::bind``, ``start()`` therefore retries a busy port
    instead of failing on the first attempt; unlike upstream it gives up after
    ``bind_timeout`` seconds rather than retrying forever.
    """

    BIND_TIMEOUT = 65.0
    BIND_RETRY_INTERVAL = 0.5

    def __init__(
        self,
        host: str,
        port: int,
        max_clients: int = 1,
        *,
        bind_timeout: float = BIND_TIMEOUT,
        bind_retry_interval: float = BIND_RETRY_INTERVAL,
    ) -> None:
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError("port must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError("port must be in range [0, 65535]")
        if isinstance(max_clients, bool) or not isinstance(max_clients, int):
            raise TypeError("max_clients must be an integer")
        if max_clients <= 0:
            raise ValueError("max_clients must be positive")
        for name, value in (
            ("bind_timeout", bind_timeout),
            ("bind_retry_interval", bind_retry_interval),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a real number")
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        self._bind_timeout = float(bind_timeout)
        self._bind_retry_interval = float(bind_retry_interval)
        self._host = host
        self._configured_port = port
        self._bound_port = port
        self._max_clients = max_clients
        self._listener: socket.socket | None = None
        self._clients: set[socket.socket] = set()
        self._client_threads: set[threading.Thread] = set()
        self._accept_thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._on_connect: ConnectCallback | None = None
        self._on_disconnect: DisconnectCallback | None = None
        self._on_message: MessageCallback | None = None

    @property
    def port(self) -> int:
        with self._lock:
            return self._bound_port

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._listener is not None

    def set_callbacks(
        self,
        *,
        on_connect: ConnectCallback,
        on_disconnect: DisconnectCallback,
        on_message: MessageCallback,
    ) -> None:
        with self._lock:
            if self._listener is not None:
                raise LifecycleError("Server callbacks cannot be changed after start()")
            self._on_connect = on_connect
            self._on_disconnect = on_disconnect
            self._on_message = on_message

    def start(self) -> None:
        with self._lock:
            if self._listener is not None:
                return
            if None in (self._on_connect, self._on_disconnect, self._on_message):
                raise LifecycleError("Server callbacks must be configured before start()")
            self._stop.clear()
        # Bind outside the lock: a busy port may take up to bind_timeout, and
        # close() must stay able to cancel the wait through self._stop.
        listener = self._bind_listener()
        with self._lock:
            if self._stop.is_set():
                listener.close()
                raise LifecycleError("Server was closed while starting")
            # Re-read: mypy keeps the narrowing from the check before the bind.
            existing: socket.socket | None = self._listener
            if existing is not None:  # a concurrent start() bound first
                listener.close()
                return
            self._listener = listener
            self._bound_port = int(listener.getsockname()[1])
            thread = threading.Thread(
                target=self._accept_loop,
                name=f"urcl-control-server-{self._bound_port}",
                daemon=True,
            )
            self._accept_thread = thread
            thread.start()

    def _bind_listener(self) -> socket.socket:
        deadline = time.monotonic() + self._bind_timeout
        warned = False
        while True:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self._host, self._configured_port))
                listener.listen(self._max_clients)
                listener.settimeout(0.2)
                return listener
            except OSError as exc:
                listener.close()
                if exc.errno != errno.EADDRINUSE or time.monotonic() >= deadline:
                    raise
                if not warned:
                    _LOGGER.warning(
                        "Port %d is busy (%s); retrying for up to %.0f s. A local "
                        "connection may hold it as its source port; reserving the "
                        "port with net.ipv4.ip_local_reserved_ports prevents this.",
                        self._configured_port,
                        exc.strerror,
                        self._bind_timeout,
                    )
                    warned = True
                if self._stop.wait(self._bind_retry_interval):
                    raise

    def write(self, client: ClientToken, data: bytes) -> None:
        if not isinstance(client, socket.socket):
            raise TypeError("SocketServer client token must be a socket")
        with self._lock:
            connected = client in self._clients
        if not connected:
            raise URCLConnectionError("Control-interface client is not connected")
        try:
            with self._write_lock:
                client.sendall(data)
        except OSError as exc:
            raise URCLConnectionError("Failed to write control-interface packet") from exc

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            listener = self._listener
            self._listener = None
            clients = tuple(self._clients)
            self._clients.clear()
            accept_thread = self._accept_thread
            client_threads = tuple(self._client_threads)
        if listener is not None:
            with suppress(OSError):
                listener.close()
        for client in clients:
            with suppress(OSError):
                client.shutdown(socket.SHUT_RDWR)
            client.close()
        current = threading.current_thread()
        if accept_thread is not None and accept_thread is not current:
            accept_thread.join(timeout=1.0)
        for thread in client_threads:
            if thread is not current:
                thread.join(timeout=1.0)

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                listener = self._listener
            if listener is None:
                return
            try:
                client, _ = listener.accept()
            except TimeoutError:
                continue
            except OSError:
                if not self._stop.is_set():
                    _LOGGER.exception("Control server accept failed")
                return
            try:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError as exc:
                _LOGGER.warning("Could not enable TCP_NODELAY on accepted socket: %s", exc)
            client.settimeout(0.2)
            with self._lock:
                if len(self._clients) >= self._max_clients:
                    accepted = False
                else:
                    self._clients.add(client)
                    accepted = True
            if not accepted:
                client.close()
                continue
            thread = threading.Thread(
                target=self._client_loop,
                args=(client,),
                name=f"urcl-control-client-{self._bound_port}",
                daemon=True,
            )
            with self._lock:
                self._client_threads.add(thread)
            thread.start()

    def _client_loop(self, client: socket.socket) -> None:
        on_connect = self._on_connect
        if on_connect is not None:
            on_connect(client)
        try:
            while not self._stop.is_set():
                try:
                    data = client.recv(4096)
                except TimeoutError:
                    continue
                except OSError:
                    break
                if not data:
                    break
                on_message = self._on_message
                if on_message is not None:
                    on_message(client, data)
        finally:
            with self._lock:
                was_connected = client in self._clients
                self._clients.discard(client)
                self._client_threads.discard(threading.current_thread())
            client.close()
            if was_connected:
                on_disconnect = self._on_disconnect
                if on_disconnect is not None:
                    on_disconnect(client)


def socket_server_factory(host: str, port: int, max_clients: int) -> Server:
    return SocketServer(host, port, max_clients)
