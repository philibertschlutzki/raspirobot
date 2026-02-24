import json
import time
import os
import threading
import queue
from typing import Any, Dict, Optional, List

try:
    from .data_models import LogEntry, LogLevel
except ImportError:
    from data_models import LogEntry, LogLevel

class Logger:
    _instance = None
    _lock = threading.Lock()

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
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._worker_thread = None
        self._early_buffer: List[LogEntry] = []
        self._buffer_lock = threading.Lock()

        self._initialized = True
        self._start_worker()

    def _start_worker(self):
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

    def setup(self, log_file_path: str):
        """Initializes the logger to write to a specific file."""
        os.makedirs(os.path.dirname(log_file_path), exist_ok=True)

        with self._buffer_lock:
            self.log_file = log_file_path

        # Wake up worker to switch file
        self._queue.put(None)

    def _worker_loop(self):
        """Background thread loop to process log entries."""
        current_file_path = None
        file_handle = None
        last_flush_time = time.time()

        while True:
            try:
                entry = self._queue.get(timeout=0.1)
            except queue.Empty:
                if self._stop_event.is_set():
                    break
                # Flush if idle
                if file_handle and time.time() - last_flush_time > 0.5:
                    file_handle.flush()
                    last_flush_time = time.time()
                continue

            # Check for file change
            with self._buffer_lock:
                target_file = self.log_file

            if target_file != current_file_path:
                if file_handle:
                    file_handle.close()
                    file_handle = None

                if target_file:
                    try:
                        file_handle = open(target_file, 'a', encoding='utf-8')
                        current_file_path = target_file

                        # Flush early buffer
                        to_flush = []
                        with self._buffer_lock:
                            to_flush = list(self._early_buffer)
                            self._early_buffer.clear()

                        for buf_entry in to_flush:
                            file_handle.write(buf_entry.model_dump_json() + '\n')
                        file_handle.flush()
                    except Exception as e:
                        print(f"Logger: Failed to open {target_file}: {e}")

            # Write entry
            if entry:
                if file_handle:
                    try:
                        file_handle.write(entry.model_dump_json() + '\n')
                        # Flush periodically even if busy
                        if time.time() - last_flush_time > 0.5:
                            file_handle.flush()
                            last_flush_time = time.time()
                    except Exception as e:
                        print(f"Logger: Write error: {e}")
                else:
                    with self._buffer_lock:
                        self._early_buffer.append(entry)

            self._queue.task_done()

        if file_handle:
            file_handle.flush()
            file_handle.close()

    def log(self, level: LogLevel, event_name: str, data: Optional[Dict[str, Any]] = None):
        """Logs an event to the queue (non-blocking)."""
        if data is None:
            data = {}

        entry = LogEntry(
            timestamp=time.time(),
            level=level,
            event_name=event_name,
            data=data
        )
        self._queue.put(entry)

    def info(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.INFO, event_name, data)

    def warn(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.WARN, event_name, data)

    def error(self, event_name: str, data: Optional[Dict[str, Any]] = None):
        self.log(LogLevel.ERROR, event_name, data)

    def close(self):
        """Stops the worker."""
        self._stop_event.set()
        self._queue.put(None)
        if self._worker_thread:
            self._worker_thread.join()

# Global instance accessor
_logger = Logger()

def get_logger():
    return _logger
