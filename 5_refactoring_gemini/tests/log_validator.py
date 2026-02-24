import argparse
import json
import sys
import os

def validate_log_line(line_number, line, required_event=None):
    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        print(f"Line {line_number}: Invalid JSON")
        return False, False

    # Schema check
    required_keys = {"timestamp", "level", "event_name", "data"}
    if not required_keys.issubset(data.keys()):
        print(f"Line {line_number}: Missing keys. Found {list(data.keys())}, expected {list(required_keys)}")
        return False, False

    # Type checks
    if not isinstance(data["timestamp"], (int, float)):
        print(f"Line {line_number}: 'timestamp' must be float/int")
        return False, False

    if data["level"] not in ["INFO", "WARN", "ERROR"]:
        print(f"Line {line_number}: 'level' must be INFO, WARN, or ERROR. Found {data['level']}")
        return False, False

    if not isinstance(data["event_name"], str):
        print(f"Line {line_number}: 'event_name' must be string")
        return False, False

    if not isinstance(data["data"], dict):
        print(f"Line {line_number}: 'data' must be dict")
        return False, False

    found_required = False
    if required_event and data["event_name"] == required_event:
        found_required = True

    return True, found_required

def main():
    parser = argparse.ArgumentParser(description="Validate JSONL log file.")
    parser.add_argument("logfile", help="Path to the JSONL log file.")
    parser.add_argument("--require", help="Event name that must be present in the log.")
    args = parser.parse_args()

    if not os.path.exists(args.logfile):
        print(f"Error: File {args.logfile} not found.")
        sys.exit(1)

    valid_file = True
    found_required_event = False

    try:
        with open(args.logfile, 'r', encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue

                is_valid, found = validate_log_line(i, line, args.require)
                if not is_valid:
                    valid_file = False
                if found:
                    found_required_event = True
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)

    if not valid_file:
        print("Validation Failed: Invalid log format.")
        sys.exit(1)

    if args.require and not found_required_event:
        print(f"Validation Failed: Required event '{args.require}' not found.")
        sys.exit(1)

    print("Validation Passed.")
    sys.exit(0)

if __name__ == "__main__":
    main()
