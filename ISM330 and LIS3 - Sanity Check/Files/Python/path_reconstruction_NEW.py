import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

# Globals
reference_paths_dir = os.path.join(os.path.dirname(__file__), "Paths")
reference_paths_csv_files = sorted([f for f in os.listdir(reference_paths_dir) if f.endswith(".csv")])

IMU_paths_dir = r'C:\Users\joshu\Documents\School\Crop Bio Repo\Crop-Biomechanics\ISM330 and LIS3 - Sanity Check\Hi-STIFFS_2026_Winter\Raw Data'

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

ACCEL_SCALE_MG_PER_LSB = 0.122   # ISM330DHCX ±4 g setting
GRAVITY_M_S2           = 9.80665

def reconstruct_path(imu_path, mag_path):
    # Load CSVs (row 0 is metadata, row 1 is header)
    imu = pd.read_csv(imu_path, skiprows=1)
    mag = pd.read_csv(mag_path, skiprows=1)

    t = imu["IMU_Time_sec"].values

    # Convert raw → m/s²
    ax = imu["Accel_X_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2
    ay = imu["Accel_Y_raw"].values * ACCEL_SCALE_MG_PER_LSB * 1e-3 * GRAVITY_M_S2

    # Remove static bias (mean of first 0.5 s)
    static_mask = t < 0.5
    ax -= ax[static_mask].mean()
    ay -= ay[static_mask].mean()

    # Double integrate: accel → velocity → position (in m, then convert to mm)
    vx = cumulative_trapezoid(ax, t, initial=0)
    vy = cumulative_trapezoid(ay, t, initial=0)
    px = cumulative_trapezoid(vx, t, initial=0) * 1000  # m → mm
    py = cumulative_trapezoid(vy, t, initial=0) * 1000

    final_position = np.column_stack((px, py))
    return final_position

def plot_comparison(reference, date, time):
    # --- Load reference path ---
    ref_file = os.path.join(reference_paths_dir, f"{reference}.csv")
    ref = pd.read_csv(ref_file)

    # --- Find matching IMU / MAG files ---
    day_dir = os.path.join(IMU_paths_dir, date)
    # Match files whose name contains date_time (e.g. 2026-06-11_140933)
    tag = f"{date}_{time}"
    all_files = os.listdir(day_dir)
    imu_file = next(f for f in all_files if tag in f and f.endswith("_IMU.csv"))
    mag_file = next(f for f in all_files if tag in f and f.endswith("_MAG.csv"))

    imu_path = os.path.join(day_dir, imu_file)
    mag_path = os.path.join(day_dir, mag_file)

    # --- Reconstruct path ---
    imu_xy = reconstruct_path(imu_path, mag_path)

    # --- Plot ---
    fig, ax = plt.subplots()
    ax.plot(ref["X(mm)"], ref["Y(mm)"], label=f"Reference: {reference}", linewidth=2)
    ax.plot(imu_xy[:, 0], imu_xy[:, 1], label="IMU (double integration)", linewidth=1.5, linestyle="--")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(f"Path Comparison — {reference} vs IMU ({tag})")
    ax.axis("equal")
    ax.legend()
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
