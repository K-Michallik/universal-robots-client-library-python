"""Opt-in standard-library logging configuration."""

from __future__ import annotations

import logging

LOGGER_NAME = "ur_client_library"
_logger = logging.getLogger(LOGGER_NAME)
_logger.addHandler(logging.NullHandler())
_configured_handler: logging.Handler | None = None


def configure_logging(
    level: int | str = logging.WARNING,
    *,
    handler: logging.Handler | None = None,
) -> logging.Logger:
    """Configure the package logger and return it.

    Applications remain in control of root logging. Repeated calls replace only
    handlers previously installed by this function.
    """
    global _configured_handler
    logger = logging.getLogger(LOGGER_NAME)
    if _configured_handler is not None:
        logger.removeHandler(_configured_handler)
    selected = handler or logging.StreamHandler()
    if selected.formatter is None:
        selected.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
        )
    logger.addHandler(selected)
    logger.setLevel(level)
    _configured_handler = selected
    return logger
