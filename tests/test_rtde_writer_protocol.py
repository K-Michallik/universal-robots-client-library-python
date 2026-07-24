from __future__ import annotations

import threading
import time

import pytest

from ur_client_library.exceptions import InvalidRangeError, WorkerThreadError
from ur_client_library.rtde import DataPackage, RTDEParser, RTDEWriter


class _CaptureStream:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.messages: list[bytes] = []

    def write(self, data: bytes | bytearray | memoryview) -> int:
        message = bytes(data)
        with self.condition:
            self.messages.append(message)
            self.condition.notify_all()
        return len(message)

    def wait(self) -> bytes:
        with self.condition:
            assert self.condition.wait_for(lambda: bool(self.messages), timeout=1.0)
            return self.messages.pop(0)


def test_writer_serializes_typed_commands() -> None:
    recipe = (
        "speed_slider_mask",
        "speed_slider_fraction",
        "standard_digital_output_mask",
        "standard_digital_output",
        "external_force_torque",
    )
    stream = _CaptureStream()
    writer = RTDEWriter(stream, recipe)
    writer.start(1)
    writer.send_speed_slider(0.5)
    package = RTDEParser(recipe, 2).parse(stream.wait())
    assert isinstance(package, DataPackage)
    assert package.recipe_id == 1
    assert package["speed_slider_mask"] == 1
    assert package["speed_slider_fraction"] == 0.5

    writer.send_standard_digital_output(2, True)
    package = RTDEParser(recipe, 2).parse(stream.wait())
    assert isinstance(package, DataPackage)
    assert package["standard_digital_output_mask"] == 4
    assert package["standard_digital_output"] == 255

    writer.send_external_force_torque((1, 2, 3, 4, 5, 6))
    package = RTDEParser(recipe, 2).parse(stream.wait())
    assert isinstance(package, DataPackage)
    assert package["external_force_torque"] == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    writer.close()


def test_writer_validates_ranges_and_recipe_membership() -> None:
    stream = _CaptureStream()
    writer = RTDEWriter(stream, ("speed_slider_mask", "speed_slider_fraction"))
    writer.start(1)
    with pytest.raises(ValueError):
        writer.send_speed_slider(1.1)
    with pytest.raises(TypeError):
        writer.send_speed_slider(True)
    with pytest.raises(InvalidRangeError):
        writer.send_speed_slider(float("nan"))
    with pytest.raises(KeyError):
        writer.send_standard_digital_output(0, True)
    writer.close()


class _FailingStream:
    def write(self, data: bytes | bytearray | memoryview) -> int:
        del data
        raise OSError("write failed")


def test_writer_retains_worker_error() -> None:
    writer = RTDEWriter(_FailingStream(), ("speed_slider_mask", "speed_slider_fraction"))
    writer.start(1)
    writer.send_speed_slider(0.5)
    deadline = time.monotonic() + 1.0
    while writer.is_running and time.monotonic() < deadline:
        time.sleep(0.001)
    with pytest.raises(WorkerThreadError, match="write failed"):
        writer.send_speed_slider(0.25)
    writer.close()
