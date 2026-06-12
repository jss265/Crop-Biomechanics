import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

# Globals
reference_paths_dir = os.path.join(os.path.dirname(__file__), "Paths")
reference_paths_csv_files = sorted([f for f in os.listdir(reference_paths_dir) if f.endswith(".csv")])

IMU_paths_dir = r'C:\Users\joshu\Documents\School\Crop Bio Repo\Crop-Biomechanics\ISM330 and LIS3 - Sanity Check\Hi-STIFFS_2026_Winter\Raw Data'

ACCEL_SCALE_MG_PER_LSB = 0.122   # ISM330DHCX ±4 g setting
GRAVITY_M_S2           = 9.80665

# Functions
def plot_reference_paths():
    for filename in reference_paths_csv_files:
        df = pd.read_csv(os.path.join(reference_paths_dir, filename))

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d") if df["Z(mm)"].nunique() > 1 else None

        if ax:
            ax.plot(df["X(mm)"], df["Y(mm)"], df["Z(mm)"])
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("Z (mm)")
        else:
            plt.plot(df["X(mm)"], df["Y(mm)"])
            plt.xlabel("X (mm)")
            plt.ylabel("Y (mm)")
            plt.axis("equal")

        plt.title(filename.replace(".csv", ""))
        plt.tight_layout()
        plt.show()

def reconstruct_path(imu_path, mag_path):
    # Load CSVs (row 0 is metadata, row 1 is header)
    imu = pd.read_csv(imu_path, skiprows=1)
    mag = pd.read_csv(mag_path, skiprows=1)

    t_imu = imu["IMU_Time_sec"].values
    t_mag = mag["MAG_Time_sec"].values

    # Convert Signal from raw → m/s²
    ax = imu["Accel_X_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2
    ay = imu["Accel_Y_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2
    az = imu["Accel_Z_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2

    # Remove static bias (mean of first 4.5 s)
    static_mask = t_imu < 4.5
    ax -= ax[static_mask].mean()
    ay -= ay[static_mask].mean()
    az -= az[static_mask].mean()

    # Double integrate: accel → velocity → position (in m, then convert to mm)
    vx = cumulative_trapezoid(ax, t_imu, initial=0)
    vy = cumulative_trapezoid(ay, t_imu, initial=0)
    vz = cumulative_trapezoid(az, t_imu, initial=0)

    # # Known final position in metres
    # desired_x, desired_y, desired_z = 0.0, 1.1684, 0.0  # 1168.4 mm → m

    # T = t_imu[-1]

    # # Integrate once to see where we'd end up
    # px_raw = cumulative_trapezoid(vx, t_imu, initial=0)
    # py_raw = cumulative_trapezoid(vy, t_imu, initial=0)
    # pz_raw = cumulative_trapezoid(vz, t_imu, initial=0)

    # # Constant velocity correction to hit the known endpoint
    # vx -= (px_raw[-1] - desired_x) / T
    # vy -= (py_raw[-1] - desired_y) / T
    # vz -= (pz_raw[-1] - desired_z) / T

    # Final integration with corrected velocity
    px = cumulative_trapezoid(vx, t_imu, initial=0) * 1000  # m → mm
    py = cumulative_trapezoid(vy, t_imu, initial=0) * 1000
    pz = cumulative_trapezoid(vz, t_imu, initial=0) * 1000

    return px, py, pz

def plot_comparison(reference, date, time):
    # --- Load reference path ---
    ref_file = os.path.join(reference_paths_dir, f"{reference}.csv")
    ref = pd.read_csv(ref_file)

    # --- Find matching IMU / MAG files ---
    day_dir = os.path.join(IMU_paths_dir, date)
    tag = f"{date}_{time}"
    all_files = os.listdir(day_dir)
    imu_file = next(f for f in all_files if tag in f and f.endswith("_IMU.csv"))
    mag_file = next(f for f in all_files if tag in f and f.endswith("_MAG.csv"))

    imu_path = os.path.join(day_dir, imu_file)
    mag_path = os.path.join(day_dir, mag_file)

    # --- Reconstruct path ---
    imu_x, imu_y, imu_z = reconstruct_path(imu_path, mag_path)

    ref_z = np.zeros(len(ref))

    # --- Plot 3D ---
    fig_3d = plt.figure()
    ax_3d = fig_3d.add_subplot(111, projection="3d")
    ax_3d.plot(ref["X(mm)"], ref["Y(mm)"], ref_z, label=f"Reference: {reference}", linewidth=2)
    ax_3d.plot(imu_x, imu_y, imu_z, label="IMU (double integration)", linewidth=1.5, linestyle="--")
    ax_3d.set_xlabel("X (mm)")
    ax_3d.set_ylabel("Y (mm)")
    ax_3d.set_zlabel("Z (mm)")
    ax_3d.set_title(f"Path Comparison (3D) — {reference} vs IMU ({tag})")
    ax_3d.set_aspect('equal')
    ax_3d.view_init(elev=90, azim=-90)
    ax_3d.legend()
    fig_3d.tight_layout()

    # --- Plot 2D ---
    fig_2d, ax_2d = plt.subplots()
    ax_2d.plot(ref["X(mm)"], ref["Y(mm)"], label=f"Reference: {reference}", linewidth=2)
    ax_2d.plot(imu_x, imu_y, label="IMU (double integration)", linewidth=1.5, linestyle="--")
    ax_2d.set_xlabel("X (mm)")
    ax_2d.set_ylabel("Y (mm)")
    ax_2d.set_title(f"Path Comparison (2D) — {reference} vs IMU ({tag})")
    ax_2d.set_aspect("equal")
    ax_2d.legend()
    fig_2d.tight_layout()

    plt.show()

# Main
if __name__ == "__main__":
    # plot_reference_paths()

    reference_path = [
        'Track 1'
        # 'Track 2'
        # 'Track 3'
    ]
    test_paths = [
        ['2026-06-11', '154428'],
        # ['2026-06-11', '154506'],
        # ['2026-06-11', '154539'],
    ]
    for test_path in test_paths:
        plot_comparison(*reference_path, *test_path)
