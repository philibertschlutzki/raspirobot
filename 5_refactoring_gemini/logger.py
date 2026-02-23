import json
import time
import os
from typing import Any, Dict, Optional
from threading import Lock

try:
    from .data_models import LogEntry, LogLevel
except ImportError:
    from data_models import LogEntry, LogLevel

class Logger:
    _instance = None
    _lock = Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(Logger, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.log_file = None
        self._initialized = True
        self._file_handle = None

    def setup(self, log_file_path: str):
        """Initializes the logger to write to a specific file."""
        with self._lock:
            if self._file_handle:
                self._file_handle.close()

            # Ensure directory exists
            os.makedirs(os.path.dirname(log_file_path), exist_ok=True)

            self.log_file = log_file_path
            self._file_handle = open(self.log_file, 'a', encoding='utf-8')

    def log(self, level: LogLevel, event_name: str, data: Optional[Dict[str, Any]] = None):
        """Logs an event to the JSONL file."""
        if not self._file_handle:
            # If setup() hasn't been called, we might want to print to stderr or ignore.
            # For now, let's print a warning once or just return.
            # But requirements say "Global JSONL-Logger".
            # Ideally setup is called early.
            return

        if data is None:
            data = {}

        entry = LogEntry(
            timestamp=time.time(),
            level=level,
            event_name=event_name,
            data=data
        )

        json_line = entry.model_dump_json()

        with self._lock:
            self._file_handle.write(json_line + '\n')
            self._file_handle.flush()

    def info(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.INFO, event_name, data)

    def warn(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.WARN, event_name, data)

    def error(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.ERROR, event_name, data)

    def close(self):
        with self._lock:
            if self._file_handle:
                self._file_handle.close()
                self._file_handle = None

# Global instance accessor
_logger = Logger()

def get_logger():
    return _logger
