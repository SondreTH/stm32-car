#!/usr/bin/env python3
"""
plot_run.py - plot a telemetry log from car_console.py (or tools/drive_sim)

  python plot_run.py logs/run_20261111_101500.csv

A distance run (columns ref_cm, dist_cm, ...) is plotted as reference and
measured distance, the error between them, the speed and the motor %.
This is the plot to tune Kp/Ki/Kd with and to compare with Simulink.

A log with x_m / y_m columns (the gyro autopilot, not in the firmware yet)
is plotted as a map; --path draws a path_data.h underneath.

Saves a PNG next to the CSV and opens a window (--no-show: only save).
"""
import argparse, csv, os, re
import matplotlib.pyplot as plt


def load_path(fn):
    pts = []
    with open(fn) as f:
        for l in f:
            m = re.search(r"\{\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*,\s*(\d+)\s*\}", l)
            if m:
                pts.append((float(m.group(1)), float(m.group(2)), int(m.group(3))))
    return pts


def plot_distance_run(rows, title):
    t = [(float(r["t_ms"]) - float(rows[0]["t_ms"])) / 1000 for r in rows]
    col = lambda k: [float(r[k]) for r in rows]
    ref, dist = col("ref_cm"), col("dist_cm")
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    ax[0].plot(t, ref, "--", color="gray", label="reference Pos_Ref [cm]")
    ax[0].plot(t, dist, color="#1f77b4", label="measured Real_Dist [cm]")
    ax[0].set_ylabel("distance [cm]")
    ax[0].set_title(title)
    ax[1].plot(t, [a - b for a, b in zip(ref, dist)], color="#d62728", label="error = reference - measured [cm]")
    ax[1].axhline(0, color="gray", lw=0.8)
    ax[1].set_ylabel("error [cm]")
    ax[2].plot(t, col("sp_cms"), "--", color="gray", label="setpoint SP_Vel [cm/s]")
    ax[2].plot(t, col("speed_cms"), color="#2ca02c", label="measured AverageSpeed [cm/s]")
    ax[2].set_ylabel("speed [cm/s]")
    ax[3].step(t, col("motor_pct"), where="post", color="#9467bd", label="motor [%]")
    ax[3].set_ylabel("motor [%]")
    ax[3].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="best")
    return fig


def plot_map(rows, title, path):
    x = [float(r["x_m"]) for r in rows]
    y = [float(r["y_m"]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.3, 1]})
    if path:
        p = load_path(path)
        ax1.plot([q[0] for q in p], [q[1] for q in p], "-", color="#bbbbbb", lw=6, label="path", zorder=1)
        for i, q in enumerate(p):
            if q[2] & 1:
                ax1.plot(q[0], q[1], "o", color="orange", ms=10, zorder=3)
            if q[2] & 2:
                ax1.plot(q[0], q[1], "X", color="red", ms=12, zorder=3)
    ax1.plot(x, y, "-", color="#1f77b4", lw=1.5, label="car (odometry)", zorder=2)
    if "true_x" in rows[0]:
        ax1.plot([float(r["true_x"]) for r in rows], [float(r["true_y"]) for r in rows],
                 "--", color="green", lw=1, label="car (true, sim)", zorder=2)
    ax1.plot(0, 0, "ks", ms=8, label="start")
    ax1.set_aspect("equal")
    ax1.grid(alpha=0.3)
    ax1.set_xlabel("x [m]  (forward at start)")
    ax1.set_ylabel("y [m]  (left at start)")
    ax1.legend(loc="best")
    ax1.set_title(title)

    t = [(float(r["t_ms"]) - float(rows[0]["t_ms"])) / 1000 for r in rows]
    v_key = "v_mps" if "v_mps" in rows[0] else "v"
    ax2.plot(t, [float(r[v_key]) for r in rows], label="speed [m/s]")
    if "set_mps" in rows[0]:
        ax2.plot(t, [float(r["set_mps"]) for r in rows], "--", label="setpoint [m/s]")
    ax2b = ax2.twinx()
    s_key = "steer_deg" if "steer_deg" in rows[0] else "steer"
    ax2b.plot(t, [float(r[s_key]) for r in rows], color="gray", alpha=0.5, label="steer [deg]")
    ax2b.set_ylabel("steer [deg]")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("speed [m/s]")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper left")
    ax2b.legend(loc="upper right")
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--path", help="path_data.h to draw underneath (map logs)")
    ap.add_argument("--no-show", action="store_true", help="only save the PNG")
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    if not rows:
        raise SystemExit("empty CSV")
    title = os.path.basename(a.csv)
    if "x_m" in rows[0]:
        fig = plot_map(rows, title, a.path)
    elif "ref_cm" in rows[0]:
        fig = plot_distance_run(rows, title)
    else:
        raise SystemExit("no ref_cm or x_m column: not a telemetry log")

    fig.tight_layout()
    out = os.path.splitext(a.csv)[0] + ".png"
    fig.savefig(out, dpi=120)
    print("saved", out)
    if not a.no_show:
        plt.show()


if __name__ == "__main__":
    main()
