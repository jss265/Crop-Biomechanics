import serial
import pandas as pd
import time
from pathlib import Path

# ============================================
# USER SETTINGS
# ============================================

PORT = 'COM26'          # Change to your port
BAUD = 115200
LOG_TIME_SEC = 60      # Duration to record

CSV_FILE = 'imu_mag_log.csv'

# ============================================
# RECORD SERIAL DATA
# ============================================

print(f"Opening {PORT}...")

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Recording for {LOG_TIME_SEC} seconds...")
print(f"Saving to {CSV_FILE}")

start_time = time.time()

with open(CSV_FILE, 'w') as f:

    while time.time() - start_time < LOG_TIME_SEC:

        try:
            line = ser.readline().decode('utf-8').strip()

            if not line:
                continue

            print(line)

            f.write(line + '\n')

        except UnicodeDecodeError:
            pass

ser.close()

print("\nRecording complete.")

# ============================================
# LOAD CSV
# ============================================

rows = []

with open(CSV_FILE, 'r') as f:

    for line in f:

        parts = line.strip().split(',')

        if len(parts) == 8 and parts[0] == 'IMU':

            rows.append({
                'sensor': 'IMU',
                'time_us': int(parts[1]),
                'ax': float(parts[2]),
                'ay': float(parts[3]),
                'az': float(parts[4]),
                'gx': float(parts[5]),
                'gy': float(parts[6]),
                'gz': float(parts[7]),
            })

        elif len(parts) == 5 and parts[0] == 'MAG':

            rows.append({
                'sensor': 'MAG',
                'time_us': int(parts[1]),
                'mx': float(parts[2]),
                'my': float(parts[3]),
                'mz': float(parts[4]),
            })

df = pd.DataFrame(rows)

print(f"\nTotal samples loaded: {len(df)}")

# ============================================
# SPLIT DATASETS
# ============================================

imu = df[df.sensor == 'IMU'].copy()
mag = df[df.sensor == 'MAG'].copy()

# ============================================
# ZERO TIMESTAMPS
# ============================================

if len(imu):
    imu['time_us'] -= imu['time_us'].iloc[0]

if len(mag):
    mag['time_us'] -= mag['time_us'].iloc[0]

# ============================================
# REMOVE DUPLICATES
# ============================================

imu_before = len(imu)
mag_before = len(mag)

imu = imu.drop_duplicates(
    subset=['ax','ay','az','gx','gy','gz']
)

mag = mag.drop_duplicates(
    subset=['mx','my','mz']
)

print("\nDuplicate Removal")
print("------------------")
print(f"IMU removed: {imu_before - len(imu)}")
print(f"MAG removed: {mag_before - len(mag)}")

# ============================================
# DATA RATE CALCULATIONS
# ============================================

def calc_rate(df):

    if len(df) < 2:
        return None

    df = df.sort_values('time_us')

    dt_us = df['time_us'].diff().dropna()

    rate_hz = 1e6 / dt_us

    return {
        'mean_rate': rate_hz.mean(),
        'std_rate': rate_hz.std(),
        'min_rate': rate_hz.min(),
        'max_rate': rate_hz.max(),
        'samples': len(df)
    }

imu_stats = calc_rate(imu)
mag_stats = calc_rate(mag)

# ============================================
# PRINT RESULTS
# ============================================

print("\nIMU Statistics")
print("--------------")

if imu_stats:
    for k, v in imu_stats.items():
        print(f"{k:12}: {v:.3f}" if isinstance(v, float) else f"{k:12}: {v}")

print("\nMAG Statistics")
print("--------------")

if mag_stats:
    for k, v in mag_stats.items():
        print(f"{k:12}: {v:.3f}" if isinstance(v, float) else f"{k:12}: {v}")

# ============================================
# SAVE CLEANED DATA
# ============================================

imu.to_csv('imu_clean.csv', index=False)
mag.to_csv('mag_clean.csv', index=False)

print("\nSaved:")
print("  imu_clean.csv")
print("  mag_clean.csv")