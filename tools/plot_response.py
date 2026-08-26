#!/usr/bin/env python3
"""Plots the CSV written by motor_sim: setpoint vs speed, and controller output."""

import argparse
import csv

import matplotlib.pyplot as plt


def read_csv(path):
    # Loads the four columns motor_sim writes: time, setpoint, speed, output.
    time, setpoint, speed, output = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            time.append(float(row["time"]))
            setpoint.append(float(row["setpoint"]))
            speed.append(float(row["speed"]))
            output.append(float(row["output"]))
    return time, setpoint, speed, output


def plot_response(time, setpoint, speed, out_path):
    # Setpoint against measured speed, so tracking and overshoot are visible.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(time, setpoint, label="setpoint", linestyle="--")
    ax.plot(time, speed, label="measured speed")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed")
    ax.set_title("setpoint vs measured speed")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_output(time, output, out_min, out_max, out_path):
    # Controller output with the saturation limits marked, so it is clear
    # when the controller is being asked for more than it can give.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(time, output, label="controller output")
    ax.axhline(out_max, color="red", linestyle=":", label="output limit")
    ax.axhline(out_min, color="red", linestyle=":")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("output (V)")
    ax.set_title("controller output with saturation limits")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot a motor_sim CSV")
    parser.add_argument("csv", nargs="?", default="step_response.csv", help="path to the CSV file")
    parser.add_argument("--out-dir", default="docs", help="directory to write the PNGs into")
    parser.add_argument("--out-min", type=float, default=-12.0, help="lower output limit to mark")
    parser.add_argument("--out-max", type=float, default=12.0, help="upper output limit to mark")
    args = parser.parse_args()

    time, setpoint, speed, output = read_csv(args.csv)

    plot_response(time, setpoint, speed, f"{args.out_dir}/response.png")
    plot_output(time, output, args.out_min, args.out_max, f"{args.out_dir}/output.png")

    print(f"wrote {args.out_dir}/response.png and {args.out_dir}/output.png")


if __name__ == "__main__":
    main()
