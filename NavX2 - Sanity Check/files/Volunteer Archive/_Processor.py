"""
Simple processor module for parsed NavX frames.
ther modules can import `process`.
"""
from _navx_types import NavXFrame
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque

# Global state
history = deque(maxlen=100)
fig = None
ax = None
is_running = True

def on_key_press(event):
	global is_running
	if event.key == 'escape':
		is_running = False
		plt.close('all')

def on_close(event):
	global is_running
	is_running = False

def quaternion_to_matrix(w, x, y, z):
	# Convert quaternion to 3x3 rotation matrix
	R = np.array([
		[1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
		[2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
		[2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
	])
	return R

def init_plot():
	global fig, ax
	fig = plt.figure(figsize=(10, 8))
	
	# Bind event listeners for ESC key and Window X button
	fig.canvas.mpl_connect('key_press_event', on_key_press)
	fig.canvas.mpl_connect('close_event', on_close)
	
	ax = fig.add_subplot(111, projection='3d')
	ax.set_xlabel('X (m)', color='red')
	ax.set_ylabel('Y (m)', color='green')
	ax.set_zlabel('Z (m)', color='blue')
	ax.set_xlim(-0.5, 0.5)
	ax.set_ylim(-0.5, 0.5)
	ax.set_zlim(-0.5, 0.5)
	plt.ion()

def process(frame: NavXFrame):
	global fig, ax, history
	
	print(frame)
	if fig is None:
		init_plot()
	
	# Record position history
	history.append([frame.X, frame.Y, frame.Z])
	
	ax.clear()
	
	# Plot trajectory as transparent grey line
	# if len(history) > 1:
	# 	hist_array = np.array(list(history))
	# 	ax.plot(hist_array[:, 0], hist_array[:, 1], hist_array[:, 2], 
	# 			color='grey', alpha=0.3, linewidth=1)
	
	# Plot current position as red sphere
	# ax.scatter([frame.X], [frame.Y], [frame.Z], color='red', s=100, label='Current')
	
	# Build rotation matrix and plot colored axis lines
	R = quaternion_to_matrix(frame.w, frame.x, frame.y, frame.z)
	origin = np.array([frame.X, frame.Y, frame.Z])
	
	# Red (X), Green (Y), Blue (Z) unit axis vectors
	for i, color in enumerate(['red', 'green', 'blue']):
		endpoint = origin + R[:, i] * 0.5
		ax.plot([origin[0], endpoint[0]], 
				[origin[1], endpoint[1]], 
				[origin[2], endpoint[2]], 
				color=color, linewidth=2)

	ax.set_xlabel('X (m)', color='red')
	ax.set_ylabel('Y (m)', color='green')
	ax.set_zlabel('Z (m)', color='blue')
	ax.set_xlim(-0.5, 0.5)
	ax.set_ylim(-0.5, 0.5)
	ax.set_zlim(-0.5, 0.5)
	ax.legend(['History', 'Current'])
	
	plt.pause(0.001)