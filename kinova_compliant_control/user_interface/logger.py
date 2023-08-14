import time
from dearpygui_ext.logger import mvLogger


class Logger:
    """Class used to log messages."""

    logger: mvLogger

    @staticmethod
    def log(message: str) -> None:
        """Log a message."""
        Logger.logger.log_info(f"{time.strftime('%X')}: {message}")
