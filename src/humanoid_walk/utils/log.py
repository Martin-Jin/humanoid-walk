"""Logging setup shared by scripts."""

import logging


def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Return a console logger. Safe to call repeatedly: handlers are added once."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        logger.addHandler(handler)
        logger.propagate = False
    logger.setLevel(level)
    return logger
