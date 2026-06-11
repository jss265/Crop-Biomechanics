import os
import pandas as pd
import matplotlib.pyplot as plt

paths_dir = os.path.join(os.path.dirname(__file__), "Paths")
csv_files = sorted([f for f in os.listdir(paths_dir) if f.endswith(".csv")])

for filename in csv_files:
    df = pd.read_csv(os.path.join(paths_dir, filename))

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
