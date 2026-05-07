"""
Shared NavX types used by parser and processor.

ACTIVE_FIELDS must mirror the uncommented rows in the C++ fields[] array.
Comment out any entry here to match what you commented out in the .cpp file.
"""

from dataclasses import dataclass
from typing import Optional


# ─── Mirror active rows from C++ fields[] ─────────────────────────────────────
ACTIVE_FIELDS = [
    'X',  # X Displacement (m) 
    'Y',  # X Displacement (m) 
    'Z',  # X Displacement (m) 
    'x',  # w Quaternion (ul) 
    'w',  # x Quaternion (ul) 
    'y',  # y Quaternion (ul) 
    'z',  # z Quaternion (ul) 
    'T',  # timestamp (ms)
]


@dataclass
class NavXFrame:
    X: Optional[float] = None  # m [X Displacement (m)]
    Y: Optional[float] = None  # m [X Displacement (m)]
    Z: Optional[float] = None  # m [X Displacement (m)]
    w: Optional[float] = None  # ul [w Quaternion (ul)]
    x: Optional[float] = None  # ul [x Quaternion (ul)]
    y: Optional[float] = None  # ul [y Quaternion (ul)]
    z: Optional[float] = None  # ul [z Quaternion (ul)]
    T: Optional[float] = None  # ms
