import os
import json
import gzip
from dataclasses import asdict
from .config import RECORDING_BASE_DIR
from .data_models import RecordingSession

class StorageManager:
    def __init__(self):
        self.base_dir = RECORDING_BASE_DIR
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def save_session(self, session: RecordingSession):
        """
        Speichert die Session in path_recordings/<session_id>.json.gz
        """
        filename = f"{session.session_id}.json.gz"
        filepath = os.path.join(self.base_dir, filename)

        # Convert dataclass to dict
        data = asdict(session)

        with gzip.open(filepath, 'wt', encoding='utf-8') as f:
            json.dump(data, f)
