"""Offline tests for the PolyScope X readiness gate used by URSim CI."""

from __future__ import annotations

import importlib.util
from collections.abc import Iterable, Iterator
from pathlib import Path
from types import ModuleType

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "wait_for_ursim_ready.py"


def _load() -> ModuleType:
    spec = importlib.util.spec_from_file_location("wait_for_ursim_ready", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ready = _load()


def _arrivals(times: Iterable[float]) -> Iterator[float]:
    yield from times


def _stream(start: float, end: float, period: float = 0.002) -> list[float]:
    count = round((end - start) / period)
    return [start + index * period for index in range(count + 1)]


def test_steady_stream_returns_after_one_window() -> None:
    arrivals = _arrivals(_stream(0.0, 20.0))

    worst = ready.await_steady_stream(
        lambda: next(arrivals), window=10.0, max_gap=0.1, deadline=100.0
    )

    assert worst == pytest.approx(0.002)
    # The gate stops reading as soon as the window is satisfied.
    assert next(arrivals) == pytest.approx(10.002)


def test_a_stall_restarts_the_window() -> None:
    # A 190 ms stall at t=5 s, as seen right after PolyScope X boot, must not
    # count towards the steady window; the gate waits for 10 s after it.
    times = _stream(0.0, 5.0) + _stream(5.19, 30.0)
    arrivals = _arrivals(times)

    worst = ready.await_steady_stream(
        lambda: next(arrivals), window=10.0, max_gap=0.1, deadline=100.0
    )

    assert worst == pytest.approx(0.19)
    assert next(arrivals) == pytest.approx(15.192)


def test_gaps_below_threshold_do_not_restart_the_window() -> None:
    times = _stream(0.0, 5.0) + _stream(5.05, 20.0)
    arrivals = _arrivals(times)

    ready.await_steady_stream(lambda: next(arrivals), window=10.0, max_gap=0.1, deadline=100.0)

    assert next(arrivals) == pytest.approx(10.002, abs=0.003)


def test_a_never_steady_stream_times_out_with_the_worst_gap() -> None:
    # A 150 ms stall every second never leaves a 10 s gap-free window.
    times: list[float] = []
    for second in range(60):
        times += _stream(second + 0.15, second + 1.0)
    arrivals = _arrivals(times)

    with pytest.raises(TimeoutError) as raised:
        ready.await_steady_stream(
            lambda: next(arrivals), window=10.0, max_gap=0.1, deadline=30.0
        )

    assert raised.value.args[0] == pytest.approx(0.152, abs=0.003)
