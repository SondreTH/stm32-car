#!/usr/bin/env python3
"""
plot_openloop.py - compare open-loop runs ("od" recordings saved by car_console.py)

  python plot_openloop.py logs/run_A.csv logs/run_B.csv
  python plot_openloop.py "full battery=logs/run_A.csv" "carpet=logs/run_B.csv"
  python plot_openloop.py --title="Tuning the turns" logs/run_A.csv logs/run_B.csv

Prints one line per run (time, distance, turn, where it ended) and saves two
1920x1080 pictures to logs/plots/ (for a video or slides):
  openloop_paths.png    the path of every run seen from above, estimated
                        from the encoder distance and the gyro heading
  openloop_signals.png  motor %, measured speed and heading over time
The path is an estimate (wheel slip and gyro drift are in it); measure the
real end point on the floor with a tape and say so next to the picture.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
# categorical slots in fixed order (validated palette, light mode)
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, comments="#")
    notes = [l[2:].rstrip() for l in open(path) if l.startswith("# ")]
    return d, notes


def style(ax):
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=12)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    title = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--title=")),
                 "Same open-loop plan, different runs")
    if not args:
        sys.exit(__doc__)
    runs = []
    for a in args:
        label, path = a.split("=", 1) if "=" in a else (os.path.basename(a)[:-4], a)
        d, notes = load(path)
        runs.append((label, d, notes))

    print(f"{'run':24s} {'time':>6s} {'dist':>7s} {'turn':>7s} {'end x':>7s} {'end y':>7s} {'off':>6s} {'batt':>6s}")
    for label, d, _ in runs:
        x, y = d["x_cm"][-1], d["y_cm"][-1]
        print(f"{label:24s} {d['t_ms'][-1]/1000:5.1f}s {d['dist_cm'][-1]:6.0f}cm {d['yaw_deg'][-1]:6.0f}deg "
              f"{x:6.0f}cm {y:6.0f}cm {np.hypot(x, y):5.0f}cm {d['batt_v'][0]:5.2f}V")

    out = os.path.join(HERE, "logs", "plots")
    os.makedirs(out, exist_ok=True)
    plt.rcParams.update({"font.size": 13, "axes.labelcolor": INK2, "text.color": INK})

    # ---- paths from above
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=150)
    style(ax)
    placed = []   # end points already labelled: move a label down if it would sit on another
    for i, (label, d, _) in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        ax.plot(d["y_cm"] * -1, d["x_cm"], color=c, lw=2, label=label)   # right = +, forward = up
        ex, ey = -d["y_cm"][-1], d["x_cm"][-1]
        ax.plot(ex, ey, "o", ms=9, color=c, mec="white", mew=2, zorder=5)
        near = sum(1 for px, py in placed if np.hypot(px - ex, py - ey) < 40)
        placed.append((ex, ey))
        ax.annotate(f"{label}: {np.hypot(ex, ey):.0f} cm off", (ex, ey), xytext=(10, 6 - 20 * near),
                    textcoords="offset points", fontsize=12, color=INK)
    ax.plot(0, 0, "s", ms=11, color=INK, zorder=6)
    ax.annotate("start", (0, 0), xytext=(-46, -4), textcoords="offset points", fontsize=12, color=INK)
    ax.annotate("", xy=(0, 25), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=INK, lw=2))
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("sideways (cm), right +")
    ax.set_ylabel("forward at the start (cm)")
    ax.set_title(f"{title} (path estimated from encoder + gyro)", loc="left", fontsize=16)
    if len(runs) > 1:
        ax.legend(frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "openloop_paths.png"))

    # ---- signals over time
    fig, axs = plt.subplots(3, 1, figsize=(12.8, 7.2), dpi=150, sharex=True)
    for ax in axs:
        style(ax)
    for i, (label, d, _) in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        t = d["t_ms"] / 1000
        axs[0].step(t, d["motor_pct"], where="post", color=c, lw=2, label=label)
        axs[1].plot(t, d["speed_cms"], color=c, lw=2)
        axs[2].plot(t, d["yaw_deg"], color=c, lw=2)
    axs[0].set_ylabel("motor (%)")
    axs[1].set_ylabel("speed (cm/s)")
    axs[2].set_ylabel("heading (deg)")
    axs[2].set_xlabel("time since the start (s)")
    axs[0].set_title(f"{title}: command, speed and heading", loc="left", fontsize=16)
    if len(runs) > 1:
        axs[0].legend(frameon=False, ncol=min(len(runs), 4), loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "openloop_signals.png"))
    print(f"pictures in {out}")


if __name__ == "__main__":
    main()
