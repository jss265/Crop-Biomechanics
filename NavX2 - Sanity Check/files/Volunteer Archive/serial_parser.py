"""
navX2 Serial Parser
-------------------
Thread 1 (reader)   : reads lines from serial, validates '$' header, puts raw line in queue
Thread 2 (consumer) : gets lines from queue, parses into NavXFrame, processes them
"""

import re
import queue
import threading
import serial
from typing import Optional
import _Processor
from _navx_types import NavXFrame, ACTIVE_FIELDS

# ─── Configure ────────────────────────────────────────────────────────────────
PORT      = "COM26"  # 'COM26' for my computer
BAUD_RATE = 115200
HEADER    = "$"      # must match Serial.print("$") in the .cpp


# ─── Thread 1: Serial Reader ───────────────────────────────────────────────────
def reader_thread(ser: serial.Serial, pkt_queue: queue.Queue, stop: threading.Event):
    """Read lines from serial. Only enqueue lines that start with HEADER."""
    while not stop.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith(HEADER):
                pkt_queue.put(line)
        except serial.SerialException:
            break

# ─── Parser ────────────────────────────────────────────────────────────────────
def parse(line: str) -> Optional[NavXFrame]:
    line = line.lstrip(HEADER).strip()
    pattern = '|'.join(ACTIVE_FIELDS)
    tokens = re.split(pattern, line)
    
    if tokens and tokens[0] == '':
        tokens.pop(0)

    if len(tokens) != len(ACTIVE_FIELDS):
        return None

    frame = NavXFrame()
    try:
        for field, token in zip(ACTIVE_FIELDS, tokens):
            setattr(frame, field, float(token))
    except ValueError:
        return None

    return frame

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    pkt_queue = queue.Queue(maxsize=500)
    stop      = threading.Event()

    print(f"Opening {PORT} at {BAUD_RATE} baud...")
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
        t_reader = threading.Thread(target=reader_thread, args=(ser, pkt_queue, stop), daemon=True)
        t_reader.start()
        print("Connected. Press ESC to stop.\n")

        try:
            while not stop.is_set() and getattr(_Processor, 'is_running', True):
                try:
                    line = pkt_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

                frame = parse(line)
                if frame:
                    _Processor.process(frame)
                pkt_queue.task_done()
        except KeyboardInterrupt:
            pass
            
        stop.set()
        t_reader.join()
        print("\nStopped.")

if __name__ == "__main__":
    main()
