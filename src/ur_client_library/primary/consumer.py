"""Thread-safe Primary package snapshots, queues, and subscriptions."""

from __future__ import annotations

import logging
import threading
from collections import deque
from collections.abc import Callable
from typing import Any, TypeVar

from ur_client_library.primary.packages import (
    ConfigurationData,
    ErrorCode,
    ErrorCodeMessage,
    KeyMessage,
    KinematicsInfo,
    MasterboardData,
    PrimaryPackage,
    RobotModeData,
    RuntimeExceptionMessage,
    SafetyModeMessage,
    VersionMessage,
)
from ur_client_library.ur.version_information import VersionInformation

_LOGGER = logging.getLogger(__name__)
_CallbackT = TypeVar("_CallbackT")


class PrimaryConsumer:
    """Maintain immutable latest-value snapshots and event queues."""

    def __init__(self) -> None:
        self._condition = threading.Condition(threading.RLock())
        self._robot_mode_data: RobotModeData | None = None
        self._configuration_data: ConfigurationData | None = None
        self._kinematics_info: KinematicsInfo | None = None
        self._masterboard_data: MasterboardData | None = None
        self._safety_mode_message: SafetyModeMessage | None = None
        self._version_message: VersionMessage | None = None
        self._errors: deque[ErrorCode] = deque()
        self._key_messages: deque[KeyMessage] = deque()
        self._runtime_exceptions: deque[RuntimeExceptionMessage] = deque()
        self._callbacks: dict[int, Callable[[PrimaryPackage], object]] = {}
        self._error_callbacks: dict[int, Callable[[ErrorCode], object]] = {}
        self._key_callbacks: dict[int, Callable[[KeyMessage], object]] = {}
        self._runtime_callbacks: dict[int, Callable[[RuntimeExceptionMessage], object]] = {}
        self._next_callback_id = 1

    def consume(self, package: PrimaryPackage) -> bool:
        typed_callbacks: tuple[Callable[[Any], object], ...] = ()
        typed_value: object | None = None
        with self._condition:
            if isinstance(package, RobotModeData):
                self._robot_mode_data = package
            elif isinstance(package, ConfigurationData):
                self._configuration_data = package
            elif isinstance(package, KinematicsInfo):
                self._kinematics_info = package
            elif isinstance(package, MasterboardData):
                self._masterboard_data = package
            elif isinstance(package, SafetyModeMessage):
                self._safety_mode_message = package
            elif isinstance(package, VersionMessage):
                self._version_message = package
            elif isinstance(package, ErrorCodeMessage):
                code = package.to_error_code()
                typed_value = code
                self._errors.append(code)
                typed_callbacks = tuple(self._error_callbacks.values())
            elif isinstance(package, KeyMessage):
                self._key_messages.append(package)
                typed_value = package
                typed_callbacks = tuple(self._key_callbacks.values())
            elif isinstance(package, RuntimeExceptionMessage):
                self._runtime_exceptions.append(package)
                typed_value = package
                typed_callbacks = tuple(self._runtime_callbacks.values())
            callbacks = tuple(self._callbacks.values())
            self._condition.notify_all()

        # User code is never invoked while the snapshot/queue lock is held.
        for callback in callbacks:
            try:
                callback(package)
            except Exception:
                _LOGGER.exception("Primary package callback failed")
        for callback in typed_callbacks:
            try:
                callback(typed_value)
            except Exception:
                _LOGGER.exception("Primary package callback failed")
        return True

    def subscribe(
        self, callback: Callable[[PrimaryPackage], object]
    ) -> Callable[[], None]:
        return self._subscribe(self._callbacks, callback)

    def subscribe_errors(self, callback: Callable[[ErrorCode], object]) -> Callable[[], None]:
        return self._subscribe(self._error_callbacks, callback)

    def subscribe_key_messages(
        self, callback: Callable[[KeyMessage], object]
    ) -> Callable[[], None]:
        return self._subscribe(self._key_callbacks, callback)

    def subscribe_runtime_exceptions(
        self, callback: Callable[[RuntimeExceptionMessage], object]
    ) -> Callable[[], None]:
        return self._subscribe(self._runtime_callbacks, callback)

    def _subscribe(
        self,
        callbacks: dict[int, Callable[[_CallbackT], object]],
        callback: Callable[[_CallbackT], object],
    ) -> Callable[[], None]:
        with self._condition:
            callback_id = self._next_callback_id
            self._next_callback_id += 1
            callbacks[callback_id] = callback

        def unsubscribe() -> None:
            with self._condition:
                callbacks.pop(callback_id, None)

        return unsubscribe

    @property
    def robot_mode_data(self) -> RobotModeData | None:
        with self._condition:
            return self._robot_mode_data

    @property
    def configuration_data(self) -> ConfigurationData | None:
        with self._condition:
            return self._configuration_data

    @property
    def kinematics_info(self) -> KinematicsInfo | None:
        with self._condition:
            return self._kinematics_info

    @property
    def masterboard_data(self) -> MasterboardData | None:
        with self._condition:
            return self._masterboard_data

    @property
    def safety_mode_message(self) -> SafetyModeMessage | None:
        with self._condition:
            return self._safety_mode_message

    @property
    def version_message(self) -> VersionMessage | None:
        with self._condition:
            return self._version_message

    @property
    def version(self) -> VersionInformation | None:
        message = self.version_message
        return None if message is None else message.version

    @property
    def latest_runtime_exception(self) -> RuntimeExceptionMessage | None:
        with self._condition:
            return self._runtime_exceptions[-1] if self._runtime_exceptions else None

    def pop_error_codes(self) -> tuple[ErrorCode, ...]:
        with self._condition:
            values = tuple(self._errors)
            self._errors.clear()
            return values

    def pop_key_messages(self) -> tuple[KeyMessage, ...]:
        with self._condition:
            values = tuple(self._key_messages)
            self._key_messages.clear()
            return values

    def pop_runtime_exceptions(self) -> tuple[RuntimeExceptionMessage, ...]:
        with self._condition:
            values = tuple(self._runtime_exceptions)
            self._runtime_exceptions.clear()
            return values

    def clear_script_feedback(self) -> None:
        with self._condition:
            self._errors.clear()
            self._key_messages.clear()
            self._runtime_exceptions.clear()

    def wait_for(self, predicate: Callable[[], bool], timeout: float | None) -> bool:
        with self._condition:
            return self._condition.wait_for(predicate, timeout)
