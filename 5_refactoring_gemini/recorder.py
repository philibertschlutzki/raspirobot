import gzip
import json
import threading
import os

try:
    from .data_models import PathRecordingData
except ImportError:
    from data_models import PathRecordingData

class DataRecorder:
    def __init__(self, output_dir: str = "path_recordings"):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    def save_session_async(self, session: PathRecordingData, filename: str = None):
        """Saves the session to a .json.gz file in a separate thread."""
        if filename is None:
            filename = f"{session.session_id}.json.gz"

        filepath = os.path.join(self.output_dir, filename)

        t = threading.Thread(target=self._save_worker, args=(session, filepath))
        t.start()
        return t

    def _save_worker(self, session: PathRecordingData, filepath: str):
        try:
            json_str = session.model_dump_json()
            with gzip.open(filepath, 'wt', encoding='utf-8') as f:
                f.write(json_str)
            print(f"Session saved to {filepath}")
        except Exception as e:
            print(f"Error saving session: {e}")
