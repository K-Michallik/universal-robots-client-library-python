"""Coalescing asynchronous RTDE input writer."""

from __future__ import annotations

import math
import threading
from collections.abc import Sequence
from numbers import Real
from typing import Protocol

from ur_client_library.exceptions import InvalidRangeError, LifecycleError, WorkerThreadError
from ur_client_library.rtde.data_package import DataPackage
from ur_client_library.ur.datatypes import AnalogOutputType


class WritableStream(Protocol):
    def write(self, data: bytes | bytearray | memoryview) -> int: ...


class RTDEWriter:
    """Coalesce input updates and send the newest package on a worker thread."""

    def __init__(self, stream: WritableStream, recipe: Sequence[str]) -> None:
        self._stream = stream
        self._recipe = tuple(recipe)
        self._recipe_id = 0
        self._condition = threading.Condition()
        self._running = False
        self._pending = False
        self._thread: threading.Thread | None = None
        self._error: WorkerThreadError | None = None
        self._used_masks: tuple[str, ...] = ()
        self._store = DataPackage(self._recipe or ("timestamp",))
        self._send = DataPackage(self._recipe or ("timestamp",))
        self._configure_buffers(self._recipe)

    @property
    def recipe(self) -> tuple[str, ...]:
        return self._recipe

    @property
    def is_running(self) -> bool:
        with self._condition:
            return self._running

    def set_input_recipe(self, recipe: Sequence[str]) -> None:
        self._raise_worker_error()
        with self._condition:
            if self._running:
                raise LifecycleError("RTDE writer must be stopped before changing its recipe")
            self._configure_buffers(tuple(recipe))

    def _configure_buffers(self, recipe: tuple[str, ...]) -> None:
        self._recipe = recipe
        self._used_masks = tuple(key for key in recipe if key.endswith("_mask"))
        if recipe:
            self._store = DataPackage(recipe)
            self._send = DataPackage(recipe)
        self._pending = False

    def start(self, recipe_id: int) -> None:
        self._raise_worker_error()
        with self._condition:
            if self._running:
                raise LifecycleError("RTDE writer is already running")
            if not self._recipe:
                raise LifecycleError("RTDE writer has no input recipe")
            self._store.recipe_id = recipe_id
            self._send.recipe_id = recipe_id
            self._recipe_id = recipe_id
            self._running = True
            self._pending = False
            self._thread = threading.Thread(
                target=self._run,
                name="rtde-writer",
                daemon=True,
            )
            self._thread.start()

    init = start

    def close(self) -> None:
        with self._condition:
            self._running = False
            self._condition.notify_all()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    stop = close

    def send_package(self, package: DataPackage) -> None:
        if package.recipe != self._recipe:
            raise ValueError("Data package recipe does not match the RTDE writer recipe")
        with self._condition:
            self._check_running_locked()
            for key in self._recipe:
                self._store[key] = package[key]
            self._mark_pending_locked()

    def send_speed_slider(self, fraction: float) -> None:
        fraction = self._validate_fraction(fraction, "speed slider fraction")
        self._set_fields(
            {
                "speed_slider_mask": 1,
                "speed_slider_fraction": fraction,
            }
        )

    def send_standard_digital_output(self, pin: int, value: bool) -> None:
        self._send_digital(
            pin,
            value,
            maximum=7,
            mask_key="standard_digital_output_mask",
            output_key="standard_digital_output",
        )

    def send_configurable_digital_output(self, pin: int, value: bool) -> None:
        self._send_digital(
            pin,
            value,
            maximum=7,
            mask_key="configurable_digital_output_mask",
            output_key="configurable_digital_output",
        )

    def send_tool_digital_output(self, pin: int, value: bool) -> None:
        self._send_digital(
            pin,
            value,
            maximum=1,
            mask_key="tool_digital_output_mask",
            output_key="tool_digital_output",
        )

    def _send_digital(
        self,
        pin: int,
        value: bool,
        *,
        maximum: int,
        mask_key: str,
        output_key: str,
    ) -> None:
        self._validate_integer(pin, 0, maximum, "pin")
        if not isinstance(value, bool):
            raise TypeError("digital output value must be a bool")
        self._set_fields(
            {
                mask_key: 1 << pin,
                output_key: 0xFF if value else 0,
            }
        )

    def send_standard_analog_output(
        self,
        pin: int,
        value: float,
        output_type: AnalogOutputType | int = AnalogOutputType.SET_ON_TEACH_PENDANT,
    ) -> None:
        self._validate_integer(pin, 0, 1, "pin")
        value = self._validate_fraction(value, "analog output value")
        if isinstance(output_type, bool):
            raise TypeError("output_type must be an AnalogOutputType")
        try:
            normalized_type = AnalogOutputType(output_type)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Unknown analog output type {output_type!r}") from exc
        fields: dict[str, object] = {
            "standard_analog_output_mask": 1 << pin,
            # Upstream deliberately writes both values; the mask chooses the pin.
            "standard_analog_output_0": value,
            "standard_analog_output_1": value,
        }
        if normalized_type is not AnalogOutputType.SET_ON_TEACH_PENDANT:
            fields["standard_analog_output_type"] = int(normalized_type) << pin
        self._set_fields(fields)

    def send_input_bit_register(self, register_id: int, value: bool) -> None:
        self._validate_integer(register_id, 64, 127, "register_id")
        if not isinstance(value, bool):
            raise TypeError("input bit register value must be a bool")
        self._set_fields({f"input_bit_register_{register_id}": value})

    def send_input_int_register(self, register_id: int, value: int) -> None:
        self._validate_integer(register_id, 24, 47, "register_id")
        self._set_fields({f"input_int_register_{register_id}": value})

    def send_input_double_register(self, register_id: int, value: float) -> None:
        self._validate_integer(register_id, 24, 47, "register_id")
        self._set_fields({f"input_double_register_{register_id}": value})

    def send_external_force_torque(self, wrench: Sequence[float]) -> None:
        self._set_fields({"external_force_torque": wrench})

    def _set_fields(self, fields: dict[str, object]) -> None:
        self._raise_worker_error()
        with self._condition:
            self._check_running_locked()
            for key, value in fields.items():
                self._store[key] = value
            self._mark_pending_locked()

    def _mark_pending_locked(self) -> None:
        self._pending = True
        self._condition.notify()

    def _check_running_locked(self) -> None:
        if not self._running:
            raise LifecycleError("RTDE writer is not running")

    def _run(self) -> None:
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._pending or not self._running)
                    if not self._running:
                        return
                    self._store, self._send = self._send, self._store
                    self._pending = False
                self._stream.write(self._send.serialize())
                self._reset_masks(self._send)
        except BaseException as exc:
            with self._condition:
                self._error = WorkerThreadError("RTDE writer", exc)
                self._running = False
                self._condition.notify_all()

    def _reset_masks(self, package: DataPackage) -> None:
        for key in self._used_masks:
            package[key] = 0

    def _raise_worker_error(self) -> None:
        with self._condition:
            error = self._error
        if error is not None:
            raise error from error.cause

    @staticmethod
    def _validate_integer(value: int, minimum: int, maximum: int, name: str) -> None:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if not minimum <= value <= maximum:
            raise InvalidRangeError(f"{name} must be in [{minimum}, {maximum}]")

    @staticmethod
    def _validate_fraction(value: object, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError(f"{name} must be a real number")
        normalized = float(value)
        if not math.isfinite(normalized) or not 0.0 <= normalized <= 1.0:
            raise InvalidRangeError(f"{name} must be between 0 and 1")
        return normalized

    def __enter__(self) -> RTDEWriter:
        if not self.is_running:
            raise LifecycleError("Call start(recipe_id) before entering an RTDE writer context")
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
