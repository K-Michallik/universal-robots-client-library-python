"""Length-framed Universal Robots TCP streams."""

from __future__ import annotations

import struct
import threading
from collections.abc import Callable
from typing import cast

from ur_client_library.comm.tcp_socket import SocketState, TCPSocket
from ur_client_library.exceptions import ParseError

# Upper bound for a single length-prefixed UR package.
DEFAULT_MAX_PACKET_SIZE = 65535


class URStream:
    """Read complete length-prefixed packages from a composed :class:`TCPSocket`."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        length_size: int = 2,
        length_decoder: Callable[[bytes], int] | None = None,
        max_packet_size: int = DEFAULT_MAX_PACKET_SIZE,
        receive_timeout: float | None = 1.0,
        transport: TCPSocket | None = None,
    ) -> None:
        if isinstance(length_size, bool) or not isinstance(length_size, int):
            raise TypeError("length_size must be an integer")
        if length_size <= 0:
            raise ValueError("length_size must be positive")
        if isinstance(max_packet_size, bool) or not isinstance(max_packet_size, int):
            raise TypeError("max_packet_size must be an integer")
        if max_packet_size < length_size:
            raise ValueError("max_packet_size cannot be smaller than length_size")
        self._transport = transport or TCPSocket(
            host,
            port,
            receive_timeout=receive_timeout,
        )
        self._length_size = length_size
        self._decode_length = length_decoder or self._default_decoder
        self._max_packet_size = max_packet_size
        self._read_lock = threading.Lock()
        self._write_lock = threading.Lock()

    @staticmethod
    def _default_decoder(prefix: bytes) -> int:
        if len(prefix) != 2:
            raise ParseError("Default package length prefix must contain two bytes")
        return cast(int, struct.unpack(">H", prefix)[0])

    @property
    def host(self) -> str:
        return self._transport.host

    @property
    def port(self) -> int:
        return self._transport.port

    @property
    def state(self) -> SocketState:
        return self._transport.state

    @property
    def is_connected(self) -> bool:
        return self._transport.is_connected

    @property
    def closed(self) -> bool:
        return self.state is SocketState.CLOSED

    @property
    def local_ip(self) -> str:
        return self._transport.local_ip

    def connect(
        self,
        *,
        max_attempts: int = 0,
        retry_interval: float = TCPSocket.DEFAULT_RECONNECTION_TIME,
        cancel_event: threading.Event | None = None,
    ) -> None:
        self._transport.connect(
            max_attempts=max_attempts,
            retry_interval=retry_interval,
            cancel_event=cancel_event,
        )

    def read(self) -> bytes:
        """Read exactly one complete frame, including its length prefix."""
        with self._read_lock:
            prefix = self._transport.read_exact(self._length_size)
            try:
                package_size = self._decode_length(prefix)
            except Exception as exc:
                if isinstance(exc, ParseError):
                    raise
                raise ParseError("Could not decode package length") from exc
            if package_size < self._length_size:
                raise ParseError(
                    f"Package size {package_size} is smaller than its "
                    f"{self._length_size}-byte length prefix"
                )
            if package_size > self._max_packet_size:
                raise ParseError(
                    f"Package size {package_size} exceeds maximum {self._max_packet_size}"
                )
            return prefix + self._transport.read_exact(package_size - self._length_size)

    read_packet = read

    def write(self, data: bytes | bytearray | memoryview) -> int:
        with self._write_lock:
            return self._transport.write(data)

    def close(self) -> None:
        self._transport.close()

    disconnect = close

    def __enter__(self) -> URStream:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
