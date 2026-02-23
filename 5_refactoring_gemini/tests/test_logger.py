import os
import sys
import pytest
import tempfile
import subprocess
import time

# Add parent directory to sys.path to allow importing logger
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from logger import get_logger, Logger

def test_logger_singleton():
    l1 = Logger()
    l2 = get_logger()
    assert l1 is l2

def test_logger_writes_jsonl():
    with tempfile.NamedTemporaryFile(suffix=".jsonl", delete=False) as tmp:
        log_path = tmp.name

    logger = get_logger()
    logger.setup(log_path)

    logger.info("TEST_INFO", {"key": "value"})
    logger.error("TEST_ERROR", {"code": 123})

    # Allow async write to flush? Logger writes synchronously with lock.
    # But let's close it to be sure buffers are flushed if needed,
    # though logger.log uses flush().

    with open(log_path, 'r') as f:
        lines = f.readlines()

    assert len(lines) >= 2 # Might be more if run multiple times in same process due to singleton

    import json
    last_line = json.loads(lines[-1])
    assert last_line["event_name"] == "TEST_ERROR"
    assert last_line["level"] == "ERROR"
    assert last_line["data"]["code"] == 123

    os.remove(log_path)

def test_log_validator_cli():
    # create a valid log file
    with tempfile.NamedTemporaryFile(mode='w+', suffix=".jsonl", delete=False) as tmp:
        tmp.write('{"timestamp": 123.45, "level": "INFO", "event_name": "REQUIRED_EVENT", "data": {}}\n')
        tmp.write('{"timestamp": 123.46, "level": "WARN", "event_name": "OTHER_EVENT", "data": {}}\n')
        log_path = tmp.name

    validator_script = os.path.join(os.path.dirname(__file__), 'log_validator.py')

    # 1. Test basic validation (Pass)
    result = subprocess.run(
        [sys.executable, validator_script, log_path],
        capture_output=True,
        text=True
    )
    assert result.returncode == 0

    # 2. Test required event present (Pass)
    result = subprocess.run(
        [sys.executable, validator_script, log_path, "--require", "REQUIRED_EVENT"],
        capture_output=True,
        text=True
    )
    assert result.returncode == 0

    # 3. Test required event missing (Fail)
    result = subprocess.run(
        [sys.executable, validator_script, log_path, "--require", "MISSING_EVENT"],
        capture_output=True,
        text=True
    )
    assert result.returncode == 1
    assert "Missing required events: MISSING_EVENT" in result.stdout

    os.remove(log_path)

def test_log_validator_invalid_json():
    with tempfile.NamedTemporaryFile(mode='w+', suffix=".jsonl", delete=False) as tmp:
        tmp.write('INVALID JSON\n')
        log_path = tmp.name

    validator_script = os.path.join(os.path.dirname(__file__), 'log_validator.py')

    result = subprocess.run(
        [sys.executable, validator_script, log_path],
        capture_output=True,
        text=True
    )
    assert result.returncode == 1

    os.remove(log_path)
