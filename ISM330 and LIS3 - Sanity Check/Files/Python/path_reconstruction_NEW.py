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

def condition_IMU_signal(t, ax, ay, az):
    # Remove static bias (mean of first 0.5 s)
    static_mask = t < 0.5
    ax -= ax[static_mask].mean()
    ay -= ay[static_mask].mean()
    az -= az[static_mask].mean()
    return ax, ay, az

def reconstruct_path(imu_path, mag_path):
    # Load CSVs (row 0 is metadata, row 1 is header)
    imu = pd.read_csv(imu_path, skiprows=1)
    mag = pd.read_csv(mag_path, skiprows=1)

    t = imu["IMU_Time_sec"].values

    # Convert Signal from raw → m/s²
    ax = imu["Accel_X_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2
    ay = imu["Accel_Y_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2
    az = imu["Accel_Z_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2

    # Condition the signal
    ax, ay, az = condition_IMU_signal(t, ax, ay, az)

    # Double integrate: accel → velocity → position (in m, then convert to mm)
    vx = cumulative_trapezoid(ax, t, initial=0)
    vy = cumulative_trapezoid(ay, t, initial=0)
    vz = cumulative_trapezoid(az, t, initial=0)
    px = cumulative_trapezoid(vx, t, initial=0) * 1000  # m → mm
    py = cumulative_trapezoid(vy, t, initial=0) * 1000
    pz = cumulative_trapezoid(vz, t, initial=0) * 1000

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

    # --- Plot 2D ---
    fig2, ax2 = plt.subplots()
    ax2.plot(ref["X(mm)"], ref["Y(mm)"], label=f"Reference: {reference}", linewidth=2)
    ax2.plot(imu_x, imu_y, label="IMU (double integration)", linewidth=1.5, linestyle="--")
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.set_title(f"Path Comparison (2D) — {reference} vs IMU ({tag})")
    ax2.set_aspect("equal")
    ax2.legend()
    plt.tight_layout()
    plt.show()

    # --- Plot 3D ---
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111, projection="3d")
    ax3.plot(ref["X(mm)"], ref["Y(mm)"], ref_z, label=f"Reference: {reference}", linewidth=2)
    ax3.plot(imu_x, imu_y, imu_z, label="IMU (double integration)", linewidth=1.5, linestyle="--")
    ax3.set_xlabel("X (mm)")
    ax3.set_ylabel("Y (mm)")
    ax3.set_zlabel("Z (mm)")
    ax3.set_title(f"Path Comparison (3D) — {reference} vs IMU ({tag})")
    ax3.set_box_aspect([1, 1, 1])
    ax3.view_init(elev=90, azim=-90)
    ax3.legend()
    plt.tight_layout()
    plt.show()

# Main
if __name__ == "__main__":
    # plot_reference_paths()

    plot_comparison(
        # 'Track 1',
        # 'Track 2',
        'Track 3',
        '2026-06-11',
        '140933'
    )
