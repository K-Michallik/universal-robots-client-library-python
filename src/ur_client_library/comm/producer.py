"""Stream-to-package producer implementation."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Generic, Protocol, TypeVar

from ur_client_library.comm.stream import URStream
from ur_client_library.exceptions import URCLConnectionError, URCLTimeoutError

LOGGER = logging.getLogger(__name__)
T = TypeVar("T")
T_co = TypeVar("T_co", covariant=True)


class FrameParser(Protocol[T_co]):
    def parse(self, data: bytes) -> T_co: ...


class URProducer(Generic[T]):
    """Read framed packages, parse them, and reconnect when configured to do so."""

    def __init__(self, stream: URStream, parser: FrameParser[T]) -> None:
        self._stream = stream
        self._parser = parser
        self._running = threading.Event()
        self._stop = threading.Event()
        self._on_reconnect: Callable[[], None] | None = None

    def set_reconnect_callback(self, callback: Callable[[], None] | None) -> None:
        self._on_reconnect = callback

    def setup(self, *, max_attempts: int = 0, retry_interval: float = 10.0) -> None:
        self._stop.clear()
        self._stream.connect(
            max_attempts=max_attempts,
            retry_interval=retry_interval,
            cancel_event=self._stop,
        )

    def start(self) -> None:
        self._running.set()

    def stop(self) -> None:
        self._running.clear()
        self._stop.set()

    def close(self) -> None:
        self.stop()
        self._stream.close()

    def try_get(self) -> T | None:
        backoff = 1.0
        while self._running.is_set():
            try:
                result = self._parser.parse(self._stream.read())
                return result
            except URCLTimeoutError:
                continue
            except URCLConnectionError:
                if self._should_give_up():
                    return None
                if self._on_reconnect is not None:
                    LOGGER.warning("RTDE stream was lost; requesting a full reconnect")
                    self._on_reconnect()
                    self._running.clear()
                    return None
                LOGGER.warning("Stream was lost; reconnecting in %.1f seconds", backoff)
                if self._wait_backoff(backoff):
                    return None
                try:
                    # reconnect() (not connect()) so a close() issued during the
                    # back-off is honoured instead of being silently undone.
                    self._stream.reconnect(
                        max_attempts=1,
                        retry_interval=backoff,
                        cancel_event=self._stop,
                    )
                    backoff = 1.0
                except URCLConnectionError:
                    backoff = min(backoff * 2.0, 120.0)
        return None

    def _should_give_up(self) -> bool:
        return (
            not self._running.is_set()
            or self._stream.closed
            or self._stream.stop_requested
        )

    def _wait_backoff(self, seconds: float) -> bool:
        """Sleep in short slices; return ``True`` when the producer should stop."""
        deadline = time.monotonic() + seconds
        while True:
            if self._should_give_up():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if self._stop.wait(min(remaining, 0.1)):
                return True
