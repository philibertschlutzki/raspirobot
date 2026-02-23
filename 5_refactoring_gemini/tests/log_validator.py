import argparse
import json
import sys
import os
from typing import List

# Add parent directory (5_refactoring_gemini/) to sys.path
# calculate based on __file__ location
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from data_models import LogEntry
except ImportError:
    print("Error: Could not import data_models. Ensure you are running from the correct directory.")
    sys.exit(1)

def validate_log_file(filepath: str, required_events: List[str] = None) -> bool:
    if not os.path.exists(filepath):
        print(f"Error: Log file not found: {filepath}")
        return False

    valid_lines = 0
    found_events = set()

    with open(filepath, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            try:
                # 1. Parse JSON
                data = json.loads(line)

                # 2. Validate against Schema (LogEntry)
                # This throws ValidationError if schema is wrong
                entry = LogEntry(**data)

                # 3. Check for required events
                found_events.add(entry.event_name)

                valid_lines += 1

            except json.JSONDecodeError as e:
                print(f"Error line {line_num}: Invalid JSON - {e}")
                return False
            except Exception as e: # Pydantic ValidationError
                print(f"Error line {line_num}: Schema validation failed - {e}")
                return False

    # print(f"Validated {valid_lines} lines.")

    if required_events:
        missing = [evt for evt in required_events if evt not in found_events]
        if missing:
            print(f"Error: Missing required events: {', '.join(missing)}")
            return False
        # else:
            # print("All required events found.")

    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate JSONL log files.")
    parser.add_argument("logfile", help="Path to the JSONL log file")
    parser.add_argument("--require", nargs='+', help="List of required event names", default=[])

    args = parser.parse_args()

    success = validate_log_file(args.logfile, args.require)

    if success:
        sys.exit(0)
    else:
        sys.exit(1)
