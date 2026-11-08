"""Save the Pico's serial output to a file until Ctrl+C.

    python python/log_serial.py data/swing_01.csv
    python python/log_serial.py data/swing_01.csv --port /dev/ttyACM1

The firmware prints its header (# comment lines, then the column names) when the port opens,
so this script only copies lines.
"""
import argparse
from pathlib import Path

import serial

parser = argparse.ArgumentParser()
parser.add_argument("file", type=Path)
parser.add_argument("--port", default="/dev/ttyACM0")
args = parser.parse_args()

args.file.parent.mkdir(parents=True, exist_ok=True)
n = 0
with serial.Serial(args.port) as ser, open(args.file, "w") as f:
    print(f"logging {args.port} -> {args.file}, Ctrl+C to stop")
    try:
        while True:
            # Blocks until a whole line arrives, so a Ctrl+C never leaves half a line in the file
            f.write(ser.readline().decode(errors="replace"))
            n += 1
            if n % 100 == 0:
                print(f"{n} lines ({n / 100:.0f} s)", end="\r")
    except KeyboardInterrupt:
        pass
print(f"\nsaved {n} lines to {args.file}")
