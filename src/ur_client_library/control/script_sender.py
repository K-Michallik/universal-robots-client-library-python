"""Lifecycle-managed robot program sender."""

from __future__ import annotations

import logging
import threading

from ur_client_library.exceptions import URCLConnectionError

from ._server import ClientToken, Server, ServerFactory, socket_server_factory
from .reverse_interface import DEFAULT_SCRIPT_SENDER_PORT

_LOGGER = logging.getLogger(__name__)


class ScriptSender:
    """Send a configured URScript after ``request_program\\n``."""

    PROGRAM_REQUEST = b"request_program\n"

    def __init__(
        self,
        program: str,
        port: int = DEFAULT_SCRIPT_SENDER_PORT,
        *,
        host: str = "0.0.0.0",
        server: Server | None = None,
        server_factory: ServerFactory = socket_server_factory,
    ) -> None:
        if not isinstance(program, str):
            raise TypeError("program must be a string")
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError("port must be an integer")
        if not 0 <= port <= 65535:
            raise ValueError("port must be in range [0, 65535]")
        self._program = program
        self._lock = threading.RLock()
        self._buffers: dict[ClientToken, bytearray] = {}
        self._server = server or server_factory(host, port, 16)
        self._server.set_callbacks(
            on_connect=self._connection_callback,
            on_disconnect=self._disconnection_callback,
            on_message=self._message_callback,
        )

    @property
    def port(self) -> int:
        return self._server.port

    @property
    def is_running(self) -> bool:
        return self._server.is_running

    @property
    def program(self) -> str:
        with self._lock:
            return self._program

    def update_program(self, program: str) -> None:
        if not isinstance(program, str):
            raise TypeError("program must be a string")
        with self._lock:
            self._program = program

    def start(self) -> None:
        try:
            self._server.start()
        except OSError as exc:
            raise URCLConnectionError(
                f"Could not start script sender on port {self._server.port}"
            ) from exc

    connect = start

    def close(self) -> None:
        self._server.close()
        with self._lock:
            self._buffers.clear()

    def __enter__(self) -> ScriptSender:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _connection_callback(self, client: ClientToken) -> None:
        with self._lock:
            self._buffers[client] = bytearray()

    def _disconnection_callback(self, client: ClientToken) -> None:
        with self._lock:
            self._buffers.pop(client, None)

    def _message_callback(self, client: ClientToken, data: bytes) -> None:
        with self._lock:
            buffer = self._buffers.setdefault(client, bytearray())
            buffer.extend(data)
            requests: list[bytes] = []
            while b"\n" in buffer:
                end = buffer.index(b"\n") + 1
                requests.append(bytes(buffer[:end]))
                del buffer[:end]
            if len(buffer) > 4096:
                _LOGGER.warning("Discarding oversized script-sender request")
                buffer.clear()
            program = self._program
        for request in requests:
            if request == self.PROGRAM_REQUEST:
                self._server.write(client, (program + "\n").encode())
