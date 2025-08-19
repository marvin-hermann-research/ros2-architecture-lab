import logging
from logging.handlers import RotatingFileHandler
from pythonjsonlogger import jsonlogger
from datetime import datetime, timezone

class LoggingFactory:
    """
    ScientificLogger provides a unified JSON-based logging interface
    for all ROS2 nodes and components. Logs follow a fixed schema:

    {
      "timestamp": "...",
      "node": "...",
      "level": "...",
      "event": "...",
      "data": {...}
    }
    """

    def __init__(self, node_name: str, log_file: str = "robot_log.json"):
        self._node_name = node_name
        self._logger = logging.getLogger(node_name)
        self._logger.setLevel(logging.INFO)

        if not self._logger.handlers:
            try:
                handler = RotatingFileHandler(
                    log_file, maxBytes=5 * 10**6, backupCount=5, encoding="utf-8"
                )
                formatter = jsonlogger.JsonFormatter()  # type: ignore
                handler.setFormatter(formatter)
                self._logger.addHandler(handler)
            except Exception as e:
                print(f"Failed to initialize logger for {node_name}: {e}")

    def log(self, level: str, event: str, data: dict):
        """
        Create a structured JSON log entry.

        Args:
            level (str): Logging level ["DEBUG", "INFO", "WARNING", "ERROR"]
            event (str): Short event identifier
            data (dict): Structured data payload
        """
        # Log using JSON formatter; catch exceptions to prevent runtime crash
        try:
            log_method = getattr(self._logger, level.lower(), self._logger.info)
            log_method(
                event,
                extra={
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "node": self._node_name,
                    "level": level.upper(),
                    "event": event,
                    "data": data
                },
            )
        except Exception as e:
            print(f"Logging failed for node {self._node_name}, event {event}: {e}")