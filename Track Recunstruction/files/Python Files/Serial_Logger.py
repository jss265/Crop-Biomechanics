import serial
import threading
import msvcrt
import csv
import time
from datetime import datetime
import re

COM_PORT = 'COM26'
BAUD_RATE = 115200

# Regex to parse the incoming telemetry data
# Example: $X-0.1195Y-0.0146Z0.0456w32.7320x0.0200y0.3400z1.4560T29139
PATTERN = re.compile(
    r'\$X([+-]?[\d\.]+)Y([+-]?[\d\.]+)Z([+-]?[\d\.]+)[wW]([+-]?[\d\.]+)x([+-]?[\d\.]+)y([+-]?[\d\.]+)z([+-]?[\d\.]+)T(\d+)'
)

def main():
    running = True
    data_log = []
    
    # Metadata tracking
    start_datetime = None

    def read_serial():
        nonlocal running, start_datetime
        try:
            ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        except Exception as e:
            print(f"Error opening port {COM_PORT}: {e}")
            running = False
            return
            
        print(f"Connected to {COM_PORT} at {BAUD_RATE} baud.")
        print("Logging data... Press 'ESC' to stop and save.")

        while running:
            try:
                line = ser.readline()
                if line:
                    decoded = line.decode('utf-8', errors='ignore').strip()
                    if not decoded:
                        continue
                        
                    if start_datetime is None:
                        start_datetime = datetime.now()
                        
                    match = PATTERN.search(decoded)
                    if match:
                        x, y, z, w, qx, qy, qz, t_val = match.groups()
                        t_ms = int(t_val)
                        
                        data_log.append({
                            'X': x, 'Y': y, 'Z': z,
                            'w': w, 'x': qx, 'y': qy, 'z': qz,
                            'T': t_ms, 'raw': decoded
                        })
                    else:
                        data_log.append({'raw': decoded})
            except Exception as e:
                print(f"Serial read error: {e}")
                
        ser.close()

    # Start the background thread for serial reading
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    # Main thread listens for ESC key to stop program
    while running:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\x1b':  # ESC key
                print("\nESC pressed. Stopping and saving data...")
                running = False
                break
        time.sleep(0.05)
        
        # In case the serial thread dies unexpectedly
        if not serial_thread.is_alive():
            running = False

    serial_thread.join()

    # Save to CSV
    if data_log and start_datetime:
        filename = f"data/NavX2_{start_datetime.strftime('%Y%m%d_%H%M%S')}.csv"
        
        timestamps = [row['T'] for row in data_log if 'T' in row]
        duration_s = 0.0
        if timestamps:
            duration_s = (max(timestamps) - min(timestamps)) / 1000.0

        comment = input("Type short description: ").strip()

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # Write Metadata
            writer.writerow(['Date', start_datetime.strftime('%Y-%m-%d %H:%M:%S')])
            writer.writerow(['Duration (s)', f"{duration_s:.4f}"])
            writer.writerow(['Comment', comment])
            writer.writerow([])
            
            # Write Data Headers
            writer.writerow(['X', 'Y', 'Z', 'w', 'x', 'y', 'z', 'T'])
            
            # Write Data Rows
            for row in data_log:
                if 'T' in row:
                    writer.writerow([
                        row['X'], row['Y'], row['Z'], 
                        row['w'], row['x'], row['y'], row['z'], 
                        row['T']
                    ])
                    
        print(f"Successfully saved {len(data_log)} records to {filename}")
    else:
        print("No data collected!")

if __name__ == "__main__":
    main()
