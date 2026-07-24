"""Protocol-based bounded producer/consumer pipelines."""

from __future__ import annotations

import logging
import os
import queue
import threading
from collections.abc import Iterable, Sequence
from typing import Generic, Protocol, TypeVar, cast, runtime_checkable

from ur_client_library.exceptions import LifecycleError, WorkerThreadError

LOGGER = logging.getLogger(__name__)
T = TypeVar("T")
T_co = TypeVar("T_co", covariant=True)
T_contra = TypeVar("T_contra", contravariant=True)


@runtime_checkable
class Producer(Protocol[T_co]):
    def setup(self, *, max_attempts: int = 0, retry_interval: float = 10.0) -> None: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...

    def try_get(self) -> T_co | Sequence[T_co] | None: ...


@runtime_checkable
class Consumer(Protocol[T_contra]):
    def consume(self, product: T_contra) -> bool | None: ...

    def on_timeout(self) -> None: ...


@runtime_checkable
class Notifier(Protocol):
    def started(self, name: str) -> None: ...

    def stopped(self, name: str) -> None: ...


class NullNotifier:
    def started(self, name: str) -> None:
        del name

    def stopped(self, name: str) -> None:
        del name


class MultiConsumer(Generic[T]):
    """Dispatch each product to a snapshot of the registered consumers."""

    def __init__(self, consumers: Iterable[Consumer[T]] = ()) -> None:
        self._consumers = list(consumers)
        self._lock = threading.Lock()

    def add(self, consumer: Consumer[T]) -> None:
        with self._lock:
            self._consumers.append(consumer)

    def remove(self, consumer: Consumer[T]) -> None:
        with self._lock:
            self._consumers.remove(consumer)

    def _snapshot(self) -> tuple[Consumer[T], ...]:
        with self._lock:
            return tuple(self._consumers)

    def setup(self) -> None:
        for consumer in self._snapshot():
            callback = getattr(consumer, "setup", None)
            if callback is not None:
                callback()

    def close(self) -> None:
        for consumer in self._snapshot():
            callback = getattr(consumer, "close", None)
            if callback is not None:
                callback()

    def stop(self) -> None:
        for consumer in self._snapshot():
            callback = getattr(consumer, "stop", None)
            if callback is not None:
                callback()

    def on_timeout(self) -> None:
        for consumer in self._snapshot():
            callback = getattr(consumer, "on_timeout", None)
            if callback is not None:
                callback()

    def consume(self, product: T) -> bool:
        result = True
        for consumer in self._snapshot():
            if consumer.consume(product) is False:
                result = False
        return result


class Pipeline(Generic[T]):
    """Run a producer and optional consumer with a bounded 32-item queue."""

    CONSUMER_TIMEOUT = 0.008

    def __init__(
        self,
        producer: Producer[T],
        consumer: Consumer[T] | None = None,
        *,
        name: str = "pipeline",
        notifier: Notifier | None = None,
        producer_fifo_scheduling: bool = False,
    ) -> None:
        self._producer = producer
        self._consumer = consumer
        self._name = name
        self._notifier = notifier or NullNotifier()
        self._fifo = producer_fifo_scheduling
        self._queue: queue.Queue[T] = queue.Queue(maxsize=32)
        self._running = threading.Event()
        self._initialized = False
        self._producer_thread: threading.Thread | None = None
        self._consumer_thread: threading.Thread | None = None
        self._error: WorkerThreadError | None = None
        self._error_lock = threading.Lock()
        self._state_lock = threading.Lock()

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    def connect(self, *, max_attempts: int = 0, retry_interval: float = 10.0) -> None:
        self._raise_worker_error()
        with self._state_lock:
            if self._initialized:
                return
        self._producer.setup(max_attempts=max_attempts, retry_interval=retry_interval)
        if self._consumer is not None:
            callback = getattr(self._consumer, "setup", None)
            if callback is not None:
                callback()
        with self._state_lock:
            self._initialized = True

    init = connect

    def start(self) -> None:
        self._raise_worker_error()
        with self._state_lock:
            if not self._initialized:
                raise LifecycleError("Pipeline must be connected before it can be started")
            if self._running.is_set():
                return
            self._running.set()
            self._producer.start()
            self._producer_thread = threading.Thread(
                target=self._run_producer,
                name=f"{self._name}-producer",
                daemon=True,
            )
            self._producer_thread.start()
            if self._consumer is not None:
                self._consumer_thread = threading.Thread(
                    target=self._run_consumer,
                    name=f"{self._name}-consumer",
                    daemon=True,
                )
                self._consumer_thread.start()
        self._notifier.started(self._name)

    run = start

    def get_latest(self, timeout: float | None = None) -> T | None:
        self._raise_worker_error()
        latest: T | None = None
        while True:
            try:
                latest = self._queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            return latest
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            self._raise_worker_error()
            return None

    get_latest_product = get_latest

    def close(self) -> None:
        with self._state_lock:
            was_running = self._running.is_set()
            self._running.clear()
        try:
            self._producer.stop()
        finally:
            current = threading.current_thread()
            for thread in (self._producer_thread, self._consumer_thread):
                if thread is not None and thread is not current:
                    thread.join(timeout=2.0)
            try:
                self._producer.close()
            finally:
                if self._consumer is not None:
                    callback = getattr(self._consumer, "close", None)
                    if callback is not None:
                        callback()
                with self._state_lock:
                    self._initialized = False
        if was_running:
            self._notifier.stopped(self._name)

    stop = close

    def _run_producer(self) -> None:
        if self._fifo:
            self._try_fifo()
        try:
            while self._running.is_set():
                produced = self._producer.try_get()
                if produced is None:
                    continue
                products: Iterable[T]
                if isinstance(produced, Sequence) and not isinstance(
                    produced, (str, bytes, bytearray, memoryview)
                ):
                    products = cast(Sequence[T], produced)
                else:
                    products = (cast(T, produced),)
                for product in products:
                    try:
                        self._queue.put_nowait(product)
                    except queue.Full:
                        LOGGER.warning(
                            "Pipeline producer overflowed; dropping package <%s>", self._name
                        )
        except BaseException as exc:
            self._store_error("producer", exc)
        finally:
            self._running.clear()
            self._notifier.stopped(self._name)

    def _run_consumer(self) -> None:
        assert self._consumer is not None
        try:
            while self._running.is_set():
                try:
                    product = self._queue.get(timeout=self.CONSUMER_TIMEOUT)
                except queue.Empty:
                    callback = getattr(self._consumer, "on_timeout", None)
                    if callback is not None:
                        callback()
                    continue
                if self._consumer.consume(product) is False:
                    break
        except BaseException as exc:
            self._store_error("consumer", exc)
        finally:
            self._running.clear()
            callback = getattr(self._consumer, "stop", None)
            if callback is not None:
                callback()
            self._notifier.stopped(self._name)

    def _store_error(self, worker: str, cause: BaseException) -> None:
        with self._error_lock:
            if self._error is None:
                self._error = WorkerThreadError(worker, cause)

    def _raise_worker_error(self) -> None:
        with self._error_lock:
            error = self._error
        if error is not None:
            raise error from error.cause

    def _try_fifo(self) -> None:
        try:
            priority = os.sched_get_priority_max(os.SCHED_FIFO)
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        except (AttributeError, OSError, PermissionError) as exc:
            LOGGER.warning("Could not enable FIFO scheduling for %s: %s", self._name, exc)

    def __enter__(self) -> Pipeline[T]:
        self.connect()
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
