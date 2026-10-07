#!/usr/bin/env python3
"""
plot_run.py - plot a run: where the car thinks it drove vs. the path

  python plot_run.py logs/run_20261001_101500.csv
  python plot_run.py logs/run_....csv --path ../cubeide/Team7_Car/Core/Inc/path_data.h
  python plot_run.py sim_out.csv --path ../cubeide/Team7_Car/Core/Inc/path_data.h

Saves a PNG next to the CSV and opens a window.
Also works for a TEACH drive (just the x/y trace).
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--path", help="path_data.h to draw underneath")
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    if not rows:
        raise SystemExit("empty CSV")
    x = [float(r["x_m"]) for r in rows]
    y = [float(r["y_m"]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.3, 1]})
    if a.path:
        p = load_path(a.path)
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
    ax1.set_title(os.path.basename(a.csv))

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

    fig.tight_layout()
    out = os.path.splitext(a.csv)[0] + ".png"
    fig.savefig(out, dpi=120)
    print("saved", out)
    plt.show()


if __name__ == "__main__":
    main()
