import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


data_folder = "data"
file_names = os.listdir(data_folder)
for filename in file_names:
	print(filename)
file_name = input('Copy and paste the file you want to visualize: ')
file_path = os.path.join(data_folder, file_name)

with open(file_path, "r") as f:
	meta_date = f.readline().strip().split(',', 1)[1]
	meta_duration = f.readline().strip().split(',', 1)[1]
	meta_comment = f.readline().strip().split(',', 1)[1]
df = pd.read_csv(file_path, skiprows=4)
positions = df[["X", "Y", "Z"]].to_numpy()
quats = df[["w", "x", "y", "z"]].to_numpy()

def quat_to_rot(q):
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

textstr = '\n'.join((
	f'Date: {meta_date}',
	f'Duration: {meta_duration} s',
	f'Comment: {meta_comment}'
))
ax.text2D(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))

# Path
ax.plot(
	positions[:, 0],
	positions[:, 1],
	positions[:, 2],
	color=(0.5, 0.5, 0.5, 0.3),
)

# First Position
ax.scatter(
	positions[0, 0], positions[0, 1], positions[0, 2], color="green", s=50
)
# Last Position
ax.scatter(
	positions[-1, 0], positions[-1, 1], positions[-1, 2], color="red", s=50
)

length = 0.05

for i, (pos, quat) in enumerate(zip(positions, quats)):
	R = quat_to_rot(quat)

	x_axis = R @ np.array([length, 0, 0])
	y_axis = R @ np.array([0, length, 0])
	z_axis = R @ np.array([0, 0, length])

	if i == 0 or i == len(positions) - 1:
		alpha = 1.0
	else:
		alpha = 0.08

	ax.plot(
		[pos[0], pos[0] + x_axis[0]],
		[pos[1], pos[1] + x_axis[1]],
		[pos[2], pos[2] + x_axis[2]],
		color=(1, 0, 0, alpha),
	)

	ax.plot(
		[pos[0], pos[0] + y_axis[0]],
		[pos[1], pos[1] + y_axis[1]],
		[pos[2], pos[2] + y_axis[2]],
		color=(0, 1, 0, alpha),
	)

	ax.plot(
		[pos[0], pos[0] + z_axis[0]],
		[pos[1], pos[1] + z_axis[1]],
		[pos[2], pos[2] + z_axis[2]],
		color=(0, 0, 1, alpha),
	)


def on_key(event):
	if event.key == "escape":
		plt.close(fig)


fig.canvas.mpl_connect("key_press_event", on_key)
ax.set_aspect('equal')

plt.show()