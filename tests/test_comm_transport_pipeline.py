from __future__ import annotations

import builtins
import socket
import threading
import time
from contextlib import suppress
from typing import cast

import pytest

from ur_client_library.comm import (
    BinParser,
    Pipeline,
    SocketState,
    TCPServer,
    TCPSocket,
    URProducer,
    URStream,
)
from ur_client_library.exceptions import (
    LifecycleError,
    ParseError,
    URCLConnectionError,
    URCLException,
    WorkerThreadError,
)


def test_connection_error_preserves_builtin_and_package_categories() -> None:
    error = URCLConnectionError("disconnected")

    assert isinstance(error, builtins.ConnectionError)
    assert isinstance(error, URCLException)


def test_bin_parser_is_big_endian_and_bounded() -> None:
    parser = BinParser(bytes.fromhex("010203040506"))
    assert parser.read_uint16() == 0x0102
    with parser.subparser(2) as child:
        assert child.read_uint16() == 0x0304
    assert parser.read_uint16() == 0x0506
    with pytest.raises(ParseError):
        parser.read_uint8()
    with pytest.raises(ParseError):
        BinParser(b"\x00").subparser(2)


def test_tcp_transport_exact_reads_and_server_tcp_nodelay() -> None:
    connected = threading.Event()
    message_received = threading.Event()
    disconnected = threading.Event()
    accepted_descriptors: list[int] = []
    messages: list[tuple[int, bytes, int]] = []
    disconnected_descriptors: list[int] = []
    server = TCPServer(0, host="127.0.0.1", max_attempts=1)

    def on_connect(descriptor: int) -> None:
        accepted_descriptors.append(descriptor)
        connected.set()

    def on_message(descriptor: int, data: bytes, size: int) -> None:
        messages.append((descriptor, data, size))
        message_received.set()

    def on_disconnect(descriptor: int) -> None:
        disconnected_descriptors.append(descriptor)
        disconnected.set()

    server.set_connect_callback(on_connect)
    server.set_message_callback(on_message)
    server.set_disconnect_callback(on_disconnect)
    server.start()
    client = TCPSocket("127.0.0.1", server.port, receive_timeout=1.0)
    client.connect(max_attempts=1)
    assert connected.wait(1.0)
    accepted = server.clients[0]
    descriptor = accepted.fileno()
    assert accepted_descriptors == [descriptor]
    assert accepted.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) == 1

    accepted.send(b"ab")
    accepted.send(b"c")
    assert client.read_exact(3) == b"abc"
    assert client.write(b"payload") == 7
    assert message_received.wait(1.0)
    assert messages == [(descriptor, b"payload", 7)]

    client.close()
    assert disconnected.wait(1.0)
    assert disconnected_descriptors == [descriptor]
    client.close()
    server.close()
    server.close()
    with pytest.raises(LifecycleError):
        server.start()


class _PartialWriteSocket:
    def __init__(self) -> None:
        self.sent = bytearray()

    def settimeout(self, timeout: float | None) -> None:
        del timeout

    def setsockopt(self, level: int, option: int, value: int) -> None:
        del level, option, value

    def connect(self, address: object) -> None:
        del address

    def send(self, data: memoryview) -> int:
        count = min(2, len(data))
        self.sent.extend(data[:count])
        return count

    def shutdown(self, how: int) -> None:
        del how

    def close(self) -> None:
        pass


def test_tcp_transport_retries_partial_writes(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = _PartialWriteSocket()
    monkeypatch.setattr(
        socket,
        "getaddrinfo",
        lambda *args, **kwargs: [(socket.AF_INET, socket.SOCK_STREAM, 0, "", ("test", 1))],
    )
    transport = TCPSocket(
        "test",
        1,
        socket_factory=lambda *args: cast(socket.socket, fake),
    )
    transport.connect(max_attempts=1)
    assert transport.write(b"abcdefg") == 7
    assert fake.sent == b"abcdefg"
    transport.close()


class _BlockingConnectSocket(_PartialWriteSocket):
    def __init__(self) -> None:
        super().__init__()
        self.connect_started = threading.Event()
        self.closed = threading.Event()

    def connect(self, address: object) -> None:
        del address
        self.connect_started.set()
        self.closed.wait()
        raise OSError("connect cancelled")

    def close(self) -> None:
        self.closed.set()


def test_close_cancels_an_inflight_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    fake = _BlockingConnectSocket()
    monkeypatch.setattr(
        socket,
        "getaddrinfo",
        lambda *args, **kwargs: [(socket.AF_INET, socket.SOCK_STREAM, 0, "", ("test", 1))],
    )
    transport = TCPSocket(
        "test",
        1,
        socket_factory=lambda *args: cast(socket.socket, fake),
    )
    errors: list[BaseException] = []

    def connect() -> None:
        try:
            transport.connect(max_attempts=0, retry_interval=10.0)
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=connect)
    thread.start()
    assert fake.connect_started.wait(1.0)
    transport.close()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], URCLConnectionError)


def test_external_event_cancels_an_inflight_connection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake = _BlockingConnectSocket()
    monkeypatch.setattr(
        socket,
        "getaddrinfo",
        lambda *args, **kwargs: [(socket.AF_INET, socket.SOCK_STREAM, 0, "", ("test", 1))],
    )
    transport = TCPSocket(
        "test",
        1,
        socket_factory=lambda *args: cast(socket.socket, fake),
    )
    cancel = threading.Event()
    errors: list[BaseException] = []

    def connect() -> None:
        try:
            transport.connect(
                max_attempts=0,
                retry_interval=10.0,
                cancel_event=cancel,
            )
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=connect)
    thread.start()
    assert fake.connect_started.wait(1.0)
    cancel.set()
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], URCLConnectionError)
    transport.close()


def test_tcp_server_unchecked_write_accepts_an_arbitrary_descriptor() -> None:
    server = TCPServer(0)
    sender, receiver = socket.socketpair()
    try:
        assert server.write_unchecked(sender.fileno(), b"payload") == 7
        assert receiver.recv(7) == b"payload"
    finally:
        sender.close()
        receiver.close()
        server.close()


class _CountingProducer:
    def __init__(self) -> None:
        self.value = 0
        self.running = False

    def setup(self, *, max_attempts: int = 0, retry_interval: float = 10.0) -> None:
        del max_attempts, retry_interval

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def close(self) -> None:
        self.running = False

    def try_get(self) -> int | None:
        if not self.running:
            return None
        self.value += 1
        if self.value > 100:
            time.sleep(0.001)
        return self.value


def test_pipeline_is_bounded_and_drains_to_latest(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    overflowed = threading.Event()
    warnings: list[str] = []

    def capture_warning(message: str, *args: object) -> None:
        warnings.append(message % args)
        overflowed.set()

    monkeypatch.setattr(
        "ur_client_library.comm.pipeline.LOGGER.warning",
        capture_warning,
    )
    pipeline = Pipeline(_CountingProducer())
    pipeline.connect()
    latest: int | None = None
    try:
        pipeline.start()
        assert overflowed.wait(1.0)
        latest = pipeline.get_latest(0.1)
    finally:
        pipeline.close()

    assert latest is not None
    assert latest > 1
    assert any("overflowed" in warning for warning in warnings)


class _FailingProducer(_CountingProducer):
    def try_get(self) -> int | None:
        raise RuntimeError("boom")


def test_pipeline_retains_worker_failure() -> None:
    pipeline = Pipeline(_FailingProducer())
    pipeline.connect()
    pipeline.start()
    deadline = time.monotonic() + 1.0
    while pipeline.is_running and time.monotonic() < deadline:
        time.sleep(0.001)
    with pytest.raises(WorkerThreadError, match="boom"):
        pipeline.get_latest(0)
    pipeline.close()


class _BatchProducer:
    def __init__(self) -> None:
        self.running = False
        self.sent = False

    def setup(self, *, max_attempts: int = 0, retry_interval: float = 10.0) -> None:
        del max_attempts, retry_interval

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def close(self) -> None:
        self.running = False

    def try_get(self) -> range | None:
        if not self.running or self.sent:
            time.sleep(0.001)
            return None
        self.sent = True
        return range(1, 4)


class _CollectingConsumer:
    def __init__(self) -> None:
        self.values: list[int] = []
        self.complete = threading.Event()

    def consume(self, product: int) -> bool:
        self.values.append(product)
        if len(self.values) == 3:
            self.complete.set()
        return True

    def on_timeout(self) -> None:
        pass


def test_pipeline_expands_covariant_product_sequences() -> None:
    consumer = _CollectingConsumer()
    pipeline = Pipeline(_BatchProducer(), consumer)
    pipeline.connect()
    pipeline.start()

    assert consumer.complete.wait(1.0)
    assert consumer.values == [1, 2, 3]
    pipeline.close()


class _FramingServer:
    """Accept connections and send one 2-byte-length-prefixed frame on each."""

    FRAME = b"\x00\x05abc"

    def __init__(self) -> None:
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.listen()
        self.socket.settimeout(0.05)
        self.port = self.socket.getsockname()[1]
        self.accepted = 0
        self.connections: list[socket.socket] = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                connection, _ = self.socket.accept()
            except OSError:
                continue
            with self.lock:
                self.accepted += 1
                self.connections.append(connection)
            connection.sendall(self.FRAME)

    def wait_for_accepts(self, count: int, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.accepted >= count:
                    return True
            time.sleep(0.01)
        return False

    def drop_all(self) -> None:
        with self.lock:
            connections, self.connections = self.connections, []
        for connection in connections:
            with suppress(OSError):
                connection.shutdown(socket.SHUT_RDWR)
            connection.close()

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.socket.close()
        self.drop_all()


class _RawFrames:
    def parse(self, data: bytes) -> bytes:
        return data


def test_reconnect_does_not_reopen_a_deliberately_closed_socket() -> None:
    server = _FramingServer()
    transport = TCPSocket("127.0.0.1", server.port, receive_timeout=1.0)
    try:
        # reconnect() only recovers a lost connection; it does not replace connect().
        with pytest.raises(URCLConnectionError, match="not lost"):
            transport.reconnect(max_attempts=1, retry_interval=0.0)
        transport.connect(max_attempts=1)
        assert server.wait_for_accepts(1)
        assert transport.read_exact(5) == _FramingServer.FRAME
        server.drop_all()
        with pytest.raises(URCLConnectionError):
            transport.read_exact(1)
        assert transport.state is SocketState.DISCONNECTED
        assert not transport.stop_requested

        transport.close()
        assert transport.stop_requested
        with pytest.raises(URCLConnectionError, match="closed"):
            transport.reconnect(max_attempts=1, retry_interval=0.0)
        assert transport.state is SocketState.CLOSED
        time.sleep(0.1)
        assert server.accepted == 1
    finally:
        transport.close()
        server.close()


def test_producer_close_during_backoff_does_not_reconnect() -> None:
    server = _FramingServer()
    stream = URStream("127.0.0.1", server.port, receive_timeout=0.2)
    producer = URProducer(stream, _RawFrames())
    producer.setup(max_attempts=1)
    producer.start()
    results: list[bytes | None] = []
    worker = threading.Thread(target=lambda: results.append(producer.try_get()), daemon=True)
    try:
        assert producer.try_get() == _FramingServer.FRAME
        assert server.wait_for_accepts(1)

        server.drop_all()
        worker.start()
        # The producer is now in its 1 s back-off; close the stream during it.
        time.sleep(0.3)
        assert worker.is_alive()
        stream.close()
        worker.join(timeout=2.0)

        assert not worker.is_alive()
        assert results == [None]
        # Outlast the original back-off: no reconnect may happen afterwards either.
        time.sleep(1.2)
        assert server.accepted == 1
        assert stream.closed
    finally:
        producer.close()
        server.close()


def test_producer_reconnects_after_a_lost_connection() -> None:
    server = _FramingServer()
    stream = URStream("127.0.0.1", server.port, receive_timeout=0.2)
    producer = URProducer(stream, _RawFrames())
    try:
        producer.setup(max_attempts=1)
        producer.start()
        assert producer.try_get() == _FramingServer.FRAME
        assert server.wait_for_accepts(1)

        server.drop_all()
        assert producer.try_get() == _FramingServer.FRAME
        assert server.accepted == 2
        assert stream.is_connected
    finally:
        producer.close()
        server.close()
