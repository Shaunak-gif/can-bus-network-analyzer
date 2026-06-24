#!/usr/bin/env python3
"""
can_dashboard.py
─────────────────────────────────────────────────────────────────
Live CAN Bus Analyzer Dashboard
Reads decoded CAN data from the STM32 Analyzer board via USB-UART
and displays a real-time dashboard in the terminal.

Requirements:
    pip install pyserial colorama

Usage:
    python can_dashboard.py                        # auto-detect port
    python can_dashboard.py --port COM3            # Windows
    python can_dashboard.py --port /dev/ttyUSB0    # Linux/Mac
    python can_dashboard.py --port COM3 --log      # also save to CSV
"""

import argparse
import csv
import os
import re
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not found.  Run:  pip install pyserial")
    sys.exit(1)

try:
    from colorama import Fore, Back, Style, init as colorama_init
    colorama_init(autoreset=True)
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = False
    class _Dummy:
        def __getattr__(self, _): return ""
    Fore = Back = Style = _Dummy()


# ─── Configuration ────────────────────────────────────────────────────
BAUD      = 115_200
REFRESH   = 0.5          # dashboard refresh rate (seconds)
LOG_FILE  = f"can_log_{datetime.now():%Y%m%d_%H%M%S}.csv"

# Maps signal name → unit, colour, bar scale (max value for % bar)
SIGNAL_META = {
    "Speed":        ("km/h",  Fore.CYAN,    200),
    "RPM":          ("RPM",   Fore.GREEN,   8000),
    "Fuel":         ("%",     Fore.YELLOW,  100),
    "Coolant Temp": ("°C",    Fore.RED,     150),
    "Battery":      ("V",     Fore.BLUE,    15),
    "Throttle":     ("%",     Fore.MAGENTA, 100),
    "FAULT":        ("",      Fore.RED,     0),
    "Unknown":      ("",      Fore.WHITE,   0),
}


# ─── Shared state ─────────────────────────────────────────────────────
state      = {}             # signal_name → (value_str, timestamp)
fault_log  = []             # list of fault strings
frame_cnt  = 0
lock       = threading.Lock()


# ─── UART reader thread ────────────────────────────────────────────────
# Pattern: [  0.123] Speed          = 80 km/h
LINE_RE = re.compile(
    r'\[\s*(\d+\.\d+)\]\s+(.+?)\s*=\s*(.+)'
)

def reader_thread(port_obj, csv_writer):
    global frame_cnt
    while True:
        try:
            line = port_obj.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            break

        if not line:
            continue

        # Fault line detection
        if "FAULT" in line or "TIMEOUT" in line or "RANGE ERR" in line:
            with lock:
                ts = datetime.now().strftime("%H:%M:%S")
                fault_log.append(f"[{ts}] {line}")
                if len(fault_log) > 10:
                    fault_log.pop(0)
            continue

        m = LINE_RE.match(line)
        if m:
            ts_str, sig, val = m.group(1), m.group(2).strip(), m.group(3).strip()
            with lock:
                state[sig] = (val, ts_str)
                frame_cnt += 1
            if csv_writer:
                csv_writer.writerow([datetime.now().isoformat(),
                                     ts_str, sig, val])


# ─── Progress bar helper ───────────────────────────────────────────────
def make_bar(value: float, max_val: float, width: int = 20) -> str:
    if max_val <= 0:
        return ""
    ratio = min(max(value / max_val, 0.0), 1.0)
    filled = int(ratio * width)
    bar = "█" * filled + "░" * (width - filled)
    return f"[{bar}] {ratio*100:5.1f}%"


def extract_number(val_str: str) -> float:
    """Pull the first float out of a value string like '80 km/h'."""
    m = re.search(r"[\d.]+", val_str)
    return float(m.group()) if m else 0.0


# ─── Dashboard renderer ────────────────────────────────────────────────
def clear():
    os.system("cls" if os.name == "nt" else "clear")


def render(port_name: str):
    with lock:
        snap       = dict(state)
        faults     = list(fault_log)
        total      = frame_cnt

    clear()
    now_str = datetime.now().strftime("%H:%M:%S")

    print(Fore.CYAN + Style.BRIGHT +
          "╔══════════════════════════════════════════════════╗")
    print(Fore.CYAN + Style.BRIGHT +
          f"║   CAN Bus Analyzer Dashboard   [{now_str}]     ║")
    print(Fore.CYAN + Style.BRIGHT +
          f"║   Port: {port_name:<10}  Frames: {total:<10}         ║")
    print(Fore.CYAN + Style.BRIGHT +
          "╚══════════════════════════════════════════════════╝")
    print()

    order = ["Speed", "RPM", "Throttle", "Fuel",
             "Coolant Temp", "Battery"]

    for sig in order:
        meta      = SIGNAL_META.get(sig, ("", Fore.WHITE, 0))
        color     = meta[1]
        max_val   = meta[2]
        val_str, ts = snap.get(sig, ("—", "—"))

        # Try to draw a bar
        bar = ""
        if max_val > 0 and val_str != "—":
            n = extract_number(val_str)
            bar = make_bar(n, max_val)

        print(color +
              f"  {sig:<16} {val_str:<14}  {bar}   @{ts}")

    # Unknown IDs
    known = set(order) | {"FAULT", "Unknown"}
    extras = {k: v for k, v in snap.items() if k not in known}
    if extras:
        print()
        print(Fore.WHITE + "  Other IDs:")
        for sig, (val_str, ts) in extras.items():
            print(Fore.WHITE + f"    {sig:<16} = {val_str}   @{ts}")

    # Faults section
    print()
    if faults:
        print(Fore.RED + Style.BRIGHT + "  ⚠  FAULTS / WARNINGS:")
        for f in faults[-5:]:
            print(Fore.RED + f"    {f}")
    else:
        print(Fore.GREEN + "  ✓  No faults detected")

    print()
    print(Style.DIM + "  [Ctrl-C to quit]  "
          "[L=log dump  C=clear  R=raw mode  S=stats]")


# ─── Port auto-detection ───────────────────────────────────────────────
def find_stm32_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    # Prefer known STM32 VID/PID or USB-Serial adapters
    for p in ports:
        desc = (p.description or "").lower()
        if any(kw in desc for kw in
               ["stm32", "st-link", "ch340", "cp210", "ftdi", "usb serial"]):
            return p.device
    if ports:
        return ports[0].device
    return None


# ─── Main ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CAN Bus Analyzer Dashboard")
    parser.add_argument("--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--log",  action="store_true",
                        help="Save decoded frames to CSV")
    args = parser.parse_args()

    port = args.port or find_stm32_port()
    if not port:
        print("No serial port found. Use --port <device>")
        sys.exit(1)

    print(f"Connecting to {port} at {args.baud} baud …")

    csv_fh     = None
    csv_writer = None
    if args.log:
        csv_fh     = open(LOG_FILE, "w", newline="")
        csv_writer = csv.writer(csv_fh)
        csv_writer.writerow(["wall_time", "can_time", "signal", "value"])
        print(f"Logging to {LOG_FILE}")

    try:
        with serial.Serial(port, args.baud, timeout=1) as ser:
            t = threading.Thread(target=reader_thread,
                                 args=(ser, csv_writer), daemon=True)
            t.start()

            while True:
                render(port)
                time.sleep(REFRESH)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if csv_fh:
            csv_fh.close()


if __name__ == "__main__":
    main()
