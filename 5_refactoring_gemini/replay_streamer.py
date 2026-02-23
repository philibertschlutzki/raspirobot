import gzip
import json
from typing import Iterator, Union, Optional
import os
import sys

try:
    from .data_models import PathRecordingData, LidarFrame
except ImportError:
    # Handle direct execution or import issues
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from data_models import PathRecordingData, LidarFrame

class ReplayStreamer:
    """
    Decoupled data provider for SLAM replay.
    Reads PathRecordingData (file or object) and yields LidarFrames.
    """
    def __init__(self, source: Union[str, PathRecordingData]):
        self.session: Optional[PathRecordingData] = None

        if isinstance(source, str):
            try:
                with gzip.open(source, 'rt', encoding='utf-8') as f:
                    data = json.load(f)
                    self.session = PathRecordingData(**data)
            except Exception as e:
                raise ValueError(f"Failed to load replay file: {e}")
        elif isinstance(source, PathRecordingData):
            self.session = source
        else:
            raise TypeError("Source must be a file path (str) or PathRecordingData object")

    def stream_frames(self) -> Iterator[LidarFrame]:
        """Generator that yields frames AFAP (As Fast As Possible)."""
        if not self.session:
            return

        for frame in self.session.frames:
            yield frame
