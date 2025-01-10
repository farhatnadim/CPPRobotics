#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt

def plot_speed_profile(csv_file):
    indices = []
    speeds = []

    # 1. Read CSV
    with open(csv_file, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # each row: {"Index": "...", "Speed": "..."}
            index = int(row["Index"])
            speed = float(row["Speed"])
            indices.append(index)
            speeds.append(speed)

    # 2. Plot
    plt.figure(figsize=(8, 4))
    plt.plot(indices, speeds, marker='o', linewidth=2)
    plt.title("Speed Profile")
    plt.xlabel("Index")
    plt.ylabel("Speed (m/s)")
    plt.grid(True)

    # 3. Show or save
    plt.show()
    plt.savefig("speed_profile_plot.png")

if __name__ == "__main__":
    plot_speed_profile("build/speed_profile_output_part1.csv")
    plot_speed_profile("build/speed_profile_output_part2.csv")
