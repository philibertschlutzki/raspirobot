import gzip
import json
import threading
import queue
import os
import time
from typing import Optional, Dict, Any

try:
    from .data_models import PathRecordingData, LidarFrame
except ImportError:
    from data_models import PathRecordingData, LidarFrame

class DataRecorder:
    def __init__(self, output_dir: str = "path_recordings"):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        self._write_queue = queue.Queue()
        self._is_recording = False
        self._thread = None
        self._file_handle = None

    def start_session(self, session_id: str, hardware_info: Dict[str, Any]):
        """Starts a new recording session."""
        if self._is_recording:
            self.stop_session()

        filename = f"{session_id}.jsonl.gz"
        filepath = os.path.join(self.output_dir, filename)

        try:
            self._file_handle = gzip.open(filepath, 'wt', encoding='utf-8')
            # Write metadata as first line
            metadata = {
                "session_id": session_id,
                "start_timestamp": time.time(),
                "hardware_info": hardware_info,
                "type": "metadata"
            }
            self._file_handle.write(json.dumps(metadata) + '\n')

            self._is_recording = True
            self._write_queue = queue.Queue()
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()
            print(f"Started recording to {filepath}")
        except Exception as e:
            print(f"Failed to start recording: {e}")
            if self._file_handle:
                self._file_handle.close()
                self._file_handle = None

    def log_frame(self, frame: LidarFrame):
        """Queues a frame for writing."""
        if self._is_recording:
            self._write_queue.put(frame)

    def stop_session(self):
        """Stops the recording session."""
        if not self._is_recording:
            return

        # Signal stop
        self._write_queue.put(None)
        if self._thread:
            self._thread.join(timeout=2.0)

        self._is_recording = False
        self._thread = None

        if self._file_handle:
            try:
                # Write end metadata?
                end_meta = {"type": "session_end", "end_timestamp": time.time()}
                self._file_handle.write(json.dumps(end_meta) + '\n')
                self._file_handle.close()
            except Exception as e:
                print(f"Error closing recording file: {e}")
            self._file_handle = None
        print("Recording stopped.")

    def _worker(self):
        while True:
            item = self._write_queue.get()
            if item is None:
                self._write_queue.task_done()
                break

            if self._file_handle:
                try:
                    # item is LidarFrame (Pydantic model)
                    if hasattr(item, 'model_dump_json'):
                        line = item.model_dump_json()
                    else:
                        line = json.dumps(item)
                    self._file_handle.write(line + '\n')
                except Exception as e:
                    print(f"Error writing frame: {e}")

            self._write_queue.task_done()
