"""
Shared NavX types used by parser and processor.

ACTIVE_FIELDS must mirror the uncommented rows in the C++ fields[] array.
Comment out any entry here to match what you commented out in the .cpp file.
"""

from dataclasses import dataclass
from typing import Optional


# ─── Mirror active rows from C++ fields[] ─────────────────────────────────────
# Comment out any entry here to match what you commented out in the .cpp file.
ACTIVE_FIELDS = [
    "timestamp",   # timestamp (s)
    "yaw",         # yaw (deg)
    "pitch",       # pitch (deg)
    "roll",        # roll (deg)
    "gyro_x",      # gyroX (rad/s)
    "gyro_y",      # gyroY (rad/s)
    "gyro_z",      # gyroZ (rad/s)
    "accel_x",     # accelX (m/s^2)
    "accel_y",     # accelY (m/s^2)
    "accel_z",     # accelZ (m/s^2)
]


@dataclass
class NavXFrame:
    timestamp: Optional[float] = None  # s
    yaw:       Optional[float] = None  # deg
    pitch:     Optional[float] = None  # deg
    roll:      Optional[float] = None  # deg
    gyro_x:    Optional[float] = None  # rad/s
    gyro_y:    Optional[float] = None  # rad/s
    gyro_z:    Optional[float] = None  # rad/s
    accel_x:   Optional[float] = None  # m/s^2
    accel_y:   Optional[float] = None  # m/s^2
    accel_z:   Optional[float] = None  # m/s^2
