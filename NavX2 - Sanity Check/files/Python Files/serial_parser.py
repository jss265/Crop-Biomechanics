"""
navX2 Serial Parser
-------------------
Thread 1 (reader)   : reads lines from serial, validates '$' header, puts raw line in queue
Thread 2 (consumer) : gets lines from queue, parses into NavXFrame, processes them
"""

import queue
import threading
import serial
from typing import Optional
from navx_types import NavXFrame, ACTIVE_FIELDS
import Processor

# ─── Configure ────────────────────────────────────────────────────────────────
PORT      = "COM3"
BAUD_RATE = 921600
HEADER    = "$"        # must match Serial.print("$") in the .cpp


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

# ─── Thread 2: Consumer / Parser ──────────────────────────────────────────────
def consumer_thread(pkt_queue: queue.Queue, stop: threading.Event):
    """Parse lines from the queue into NavXFrame structs and process them."""
    while not stop.is_set():
        try:
            line = pkt_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        frame = parse(line)
        if frame:
            process(frame)

        pkt_queue.task_done()

# ─── Parser ────────────────────────────────────────────────────────────────────
def parse(line: str) -> Optional[NavXFrame]:
    """Strip header byte, split on whitespace, zip against ACTIVE_FIELDS."""
    tokens = line.lstrip(HEADER).strip().split()

    if len(tokens) != len(ACTIVE_FIELDS):
        return None  # wrong field count — dropped packet

    frame = NavXFrame()
    try:
        for field, token in zip(ACTIVE_FIELDS, tokens):
            setattr(frame, field, float(token))
    except ValueError:
        return None

    return frame

# ─── Processing hook ──────────────────────────────────────────────────────────
def process(frame: NavXFrame):
    """
    Process data from processor.py. 
    frame will be discarded after process call unless processor.py does something with it
    """
    Processor.process(frame)

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    pkt_queue = queue.Queue(maxsize=500)
    stop      = threading.Event()

    print(f"Opening {PORT} at {BAUD_RATE} baud...")
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
        print("Connected.\n")

        t_reader   = threading.Thread(target=reader_thread,   args=(ser, pkt_queue, stop), daemon=True)
        t_consumer = threading.Thread(target=consumer_thread, args=(pkt_queue, stop),      daemon=True)

        t_reader.start()
        t_consumer.start()

        try:
            t_reader.join()
            t_consumer.join()
        except KeyboardInterrupt:
            print("\nStopping...")
            stop.set()

if __name__ == "__main__":
    main()
