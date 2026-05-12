import serial
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from collections import deque
import time


# ---------------- CONFIG ----------------
SERIAL_PORT = "COM26"
BAUD = 115200

trail_seconds = 2.0
axis_len = 0.05

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0)


# ---------------- ROTATION ----------------
def quat_to_rot(q):
    w, x, y, z = q

    n = np.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)

    w, x, y, z = w/n, x/n, y/n, z/n

    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
    ])


# ---------------- DATA ----------------
data = deque()


def parse(line):
    try:
        if not line.startswith("$"):
            return None

        line = line[1:]

        parts = {}
        key = ""
        val = ""

        for c in line:
            if c.isalpha() and c not in "eE":
                if key:
                    parts[key] = val
                key = c
                val = ""
            else:
                val += c

        if key:
            parts[key] = val

        return (
            time.time(),
            np.array([float(parts["X"]), float(parts["Y"]), float(parts["Z"])]),
            np.array([float(parts["w"]), float(parts["x"]), float(parts["y"]), float(parts["z"])])
        )
    except:
        return None


# ---------------- APP ----------------
app = pg.mkQApp()
w = gl.GLViewWidget()
w.show()
w.setWindowTitle("IMU Live (AUTO CENTER)")

w.opts['distance'] = 2.0
w.opts['elevation'] = 20
w.opts['azimuth'] = 45


# grid
grid = gl.GLGridItem()
grid.scale(0.1, 0.1, 0.1)
w.addItem(grid)


# axes
origin = np.array([0, 0, 0], dtype=float)

x_line = gl.GLLinePlotItem(color=(1, 0, 0, 1), width=3)
y_line = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=3)
z_line = gl.GLLinePlotItem(color=(0, 0, 1, 1), width=3)

w.addItem(x_line)
w.addItem(y_line)
w.addItem(z_line)


# trail
trail = gl.GLLinePlotItem(color=(0.7, 0.7, 0.7, 1), width=2)
w.addItem(trail)


# ---------------- UPDATE ----------------
def update():

    line = ser.readline().decode(errors="ignore").strip()
    sample = parse(line)

    if sample:
        data.append(sample)

    if not data:
        return

    now = data[-1][0]

    while data and (now - data[0][0]) > trail_seconds:
        data.popleft()

    positions = np.array([d[1] for d in data])

    pos = data[-1][1]
    quat = data[-1][2]

    # -------- TRAIL --------
    if len(positions) > 1:
        trail.setData(pos=positions)

    # -------- ROTATION --------
    R = quat_to_rot(quat)

    x_axis = R @ np.array([axis_len, 0, 0])
    y_axis = R @ np.array([0, axis_len, 0])
    z_axis = R @ np.array([0, 0, axis_len])

    x_line.setData(pos=np.array([pos, pos + x_axis]))
    y_line.setData(pos=np.array([pos, pos + y_axis]))
    z_line.setData(pos=np.array([pos, pos + z_axis]))

    # -------- CAMERA FOLLOWS IMU --------
    w.opts['center'] = pg.Vector(pos[0], pos[1], pos[2])


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(16)

app.exec()