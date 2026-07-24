from __future__ import annotations

import builtins
import socket
import threading
import time
from typing import cast

import pytest

from ur_client_library.comm import BinParser, Pipeline, TCPServer, TCPSocket
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
