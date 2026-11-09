"""Save the Pico's serial output to a file until Ctrl+C, and send keys to the Pico.

    python python/log_serial.py data/swing_01.csv
    python python/log_serial.py data/run_01.csv --port /dev/ttyACM1

Everything the Pico sends goes to the file. Data lines (they start with a digit) only go there;
every other line, such as the firmware's # messages and prompts and the column names, also shows
in the terminal.

Keys: type them and press Enter. They are sent without the Enter, so typing e sends only 'e'
(a newline would arrive as a second key). A bare Enter sends a newline, for "press any key".
"""
import argparse
import sys
import threading
from pathlib import Path

import serial


def send_keys(ser):
    """Runs next to the logging loop: each line typed in the terminal goes to the Pico."""
    for line in sys.stdin:
        keys = line.rstrip("\r\n")
        ser.write(keys.encode() if keys else b"\n")
        print(f"  sent {keys!r}" if keys else "  sent Enter")


parser = argparse.ArgumentParser()
parser.add_argument("file", type=Path)
parser.add_argument("--port", default="/dev/ttyACM0")
args = parser.parse_args()

args.file.parent.mkdir(parents=True, exist_ok=True)
n = 0
# buffering=1: every line reaches the disk at once, so a crash loses nothing
with serial.Serial(args.port) as ser, open(args.file, "w", buffering=1) as f:
    print(f"logging {args.port} -> {args.file}, Ctrl+C to stop. Type keys + Enter to send them")
    threading.Thread(target=send_keys, args=(ser,), daemon=True).start()
    try:
        while True:
            # Blocks until a whole line arrives, so a Ctrl+C never leaves half a line in the file
            line = ser.readline().decode(errors="replace")
            f.write(line)
            if line[:1].isdigit():
                n += 1
                if n % 1000 == 0:
                    print(f"  {n} data lines ({n / 100:.0f} s)")
            else:
                print(line, end="")
    except KeyboardInterrupt:
        pass
print(f"\nsaved {n} data lines to {args.file}")
