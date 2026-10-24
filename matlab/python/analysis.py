"""Python port of step1_identify ... step4_validate, run on the virtual car's logs
(matlab/data_sim). Writes the figures for the report and results.json.

    ../../../stm32-car-cmake/.venv/bin/python analysis.py

The MATLAB scripts do the same steps with the toolboxes (tfest, pidtune); the
tuning target (phase margin 60 deg at the crossover chosen here) is copied into
step2_tune_pid.m so both give the same gains.
"""
import json
import math
import re
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from virtual_car import Car, Firmware, closedloop_run

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "data_sim"
FIG = HERE.parents[1] / "report" / "figures"
FIG.mkdir(parents=True, exist_ok=True)

FW = Firmware()
TS = FW.Ts

# chart style: categorical slots in fixed order, recessive grid, neutral reference
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
REF = "#555555"
plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 200, "font.size": 9, "axes.titlesize": 10,
    "axes.grid": True, "grid.color": "#dddddd", "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.6,
    "legend.frameon": False, "axes.edgecolor": "#888888",
})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(FIG / name)
    plt.close(fig)


# ----------------------------------------------------------------- telemetry
ROW = re.compile(r"-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){7}")
COLS = ["ms", "cnt", "dist_cm", "spd_cms", "sp_cms", "mot_pct", "ref_cm", "sonar_cm"]


def load_telemetry(path):
    """Same as load_telemetry.m: keep only the 8-number data rows."""
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("#"):
            continue
        m = ROW.search(line)
        if m:
            rows.append([float(v) for v in m.group(0).split(",")])
    a = np.array(rows)
    tel = {c: a[:, i] for i, c in enumerate(COLS)}
    tel["t"] = (tel["ms"] - tel["ms"][0]) / 1000
    return tel


# ----------------------------------------------------------------- models
# A model is {"K": gain, "taus": [time constants], "L": dead time}:
#   G(s) = K e^(-L s) / prod(tau_i s + 1)
def model_str(m):
    den = " ".join(f"({t:.3f}s+1)" for t in m["taus"])
    d = f" e^(-{m['L']:.3f}s)" if m["L"] > 0 else ""
    return f"{m['K']:.3f}{d} / {den}"


class Plant:
    """G(s) simulated with step dt (exact discretisation of each first-order lag)."""

    def __init__(self, m, dt):
        self.K = m["K"]
        self.a = [math.exp(-dt / t) for t in m["taus"]]
        self.buf = [0.0] * max(int(round(m["L"] / dt)), 0)
        self.y = [0.0] * len(self.a)

    def step(self, u):
        if self.buf:
            self.buf.append(u)
            u = self.buf.pop(0)
        x = self.K * u
        for i, a in enumerate(self.a):
            self.y[i] = a * self.y[i] + (1 - a) * x
            x = self.y[i]
        return x


def model_response(m, u, Ts=TS, sub=10):
    """Output on the Ts grid for an input held constant over each Ts (ZOH)."""
    pl = Plant(m, Ts / sub)
    y = np.zeros(len(u))
    for k, uk in enumerate(u):
        y[k] = pl.y[-1]
        for _ in range(sub):
            pl.step(uk)
    return y


def nrmse_fit(y, yhat):
    """Fit in % as MATLAB compare(): 100 (1 - |y - yhat| / |y - mean(y)|)."""
    return 100 * (1 - np.linalg.norm(y - yhat) / np.linalg.norm(y - y.mean()))


# ----------------------------------------------------------------- step 1
def segments():
    segs = []
    for f in sorted(DATA.glob("openloop_P*.csv")):
        tel = load_telemetry(f)
        on = int(np.argmax(tel["mot_pct"] > 0))
        P = tel["mot_pct"][on]
        rest = np.nonzero(tel["mot_pct"][on:] != P)[0]
        off = on + rest[0] - 1 if len(rest) else len(tel["t"]) - 1
        first = int(np.argmax(tel["t"] >= tel["t"][on] - 1))
        t = tel["t"][first:off + 1]
        tg = np.arange(t[0], t[-1] + 1e-9, TS)
        idx = np.searchsorted(t, tg, side="right") - 1
        u = tel["mot_pct"][first:off + 1][idx]                    # 'previous'
        v = np.interp(tg, t, tel["spd_cms"][first:off + 1])         # 'linear'
        segs.append({"name": f.name, "P": P, "t": tg - tg[0], "u": u, "v": v})
    return segs


def fit_model(segs, p0, kind):
    def unpack(x):
        if kind == "1st":
            return {"K": x[0], "taus": [x[1]], "L": 0.0}
        if kind == "1st+delay":
            return {"K": x[0], "taus": [x[1]], "L": x[2]}
        return {"K": x[0], "taus": [x[1], x[2]], "L": 0.0}

    x0 = {"1st": [1.5, 0.3], "1st+delay": [1.5, 0.3, 0.05], "2nd": [1.5, 0.3, 0.05]}[kind]
    lo = {"1st": [0, 0.01], "1st+delay": [0, 0.01, 0], "2nd": [0, 0.01, 0.001]}[kind]
    hi = {"1st": [10, 5], "1st+delay": [10, 5, 1], "2nd": [10, 5, 5]}[kind]

    def resid(x):
        m = unpack(x)
        return np.concatenate([model_response(m, np.maximum(s["u"] - p0, 0)) - s["v"] for s in segs])

    m = unpack(least_squares(resid, x0, bounds=(lo, hi)).x)
    m["fit"] = float(np.mean([nrmse_fit(s["v"], model_response(m, np.maximum(s["u"] - p0, 0))) for s in segs]))
    return m


def step1(R):
    segs = segments()
    P = np.array([s["P"] for s in segs])
    vss = np.array([s["v"][s["t"] >= s["t"][-1] - 1].mean() for s in segs])
    k_static, b = np.polyfit(P, vss, 1)
    p0 = -b / k_static

    models = {k: fit_model(segs, p0, k) for k in ("1st", "1st+delay", "2nd")}
    best = "1st"
    for k in ("1st+delay", "2nd"):
        if models[k]["fit"] > models[best]["fit"] + 3:
            best = k
    G = models[best]

    # hold-out check: identify without P 70, then predict P 70
    train = [s for s in segs if s["P"] != 70]
    test = [s for s in segs if s["P"] == 70][0]
    mh = fit_model(train, p0, best)
    hold = nrmse_fit(test["v"], model_response(mh, np.maximum(test["u"] - p0, 0)))

    R["step1"] = {"P": P.tolist(), "vss": vss.round(2).tolist(), "p0": p0, "k_static": k_static,
                  "fw_equiv": {"MOTOR_MIN_PCT": p0, "MOTOR_PCT_PER_CMS": 1 / k_static},
                  "models": {k: {**v, "str": model_str(v)} for k, v in models.items()},
                  "best": best, "holdout_fit_P70": hold, "holdout_model": model_str(mh)}

    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.9))
    for i, s in enumerate(segs):
        ax[0].plot(s["t"], s["v"], color=C[i], label=f"P {s['P']:.0f}")
    ax[0].set(xlabel="t (s)", ylabel="speed (cm/s)", title="Open-loop step responses (simulated)")
    ax[0].legend(loc="upper left")
    Pf = np.linspace(p0, P.max(), 50)
    ax[1].plot(Pf, k_static * (Pf - p0), color=REF, lw=1.2,
               label=f"fit: {k_static:.2f} (P - {p0:.1f})")
    ax[1].plot(P, vss, "o", color=C[0], ms=6, label="steady speed")
    ax[1].set(xlabel="motor %", ylabel="steady speed (cm/s)", title="Static characteristic")
    ax[1].legend(loc="upper left")
    save(fig, "openloop_steps.png")

    fig, ax = plt.subplots(figsize=(7.2, 3.0))
    t0 = 0.0
    names = {"1st": "1st order", "1st+delay": "1st order + delay", "2nd": "2nd order"}
    for j, s in enumerate(segs):
        tt = s["t"] + t0
        ax.plot(tt, s["v"], color="#222222", lw=2.4, label="log (virtual car)" if j == 0 else None)
        for i, k in enumerate(("1st", "1st+delay", "2nd")):
            ax.plot(tt, model_response(models[k], np.maximum(s["u"] - p0, 0)), color=C[i], lw=1.2,
                    ls="--" if k == "2nd" else "-",
                    label=f"{names[k]} ({models[k]['fit']:.1f} %)" if j == 0 else None)
        ax.text(tt[0] + 0.1, s["v"].max() + 2, f"P {s['P']:.0f}", fontsize=8, color="#444444")
        t0 = tt[-1] + 0.5
    ax.set(xlabel="time, the four tests one after another (s)", ylabel="speed (cm/s)",
           title="Identification: logged speed vs models (fit %)")
    ax.legend(loc="upper left", ncol=2, fontsize=8)
    save(fig, "identification.png")
    return G, p0


# ----------------------------------------------------------------- step 2
def freq_L(G, Kp, Ki, w, Ts=None, Kd=0.0):
    """Open-loop L(jw) of PI x slope*G/s. Ts=None: continuous; else discrete, ZOH,
    PI with backward Euler integral as ControlPID()."""
    s = 1j * w
    G0 = G["K"] * np.prod([1 / (t * s + 1) for t in G["taus"]], axis=0) * np.exp(-G["L"] * s)
    if Ts is None:
        return (Kp + Ki / s + Kd * s) * FW.motor_slope * G0 / s
    z = np.exp(s * Ts)
    # ZOH equivalent of slope*G/s: exact for one first-order lag, otherwise
    # approximated by the ZOH half-sample delay
    if len(G["taus"]) == 1:
        tau = G["taus"][0]
        a = math.exp(-Ts / tau)
        K = FW.motor_slope * G["K"]
        Pz = K * ((Ts - tau * (1 - a)) * z + (tau * (1 - a) - a * Ts)) / ((z - 1) * (z - a))
        Pz = Pz * np.exp(-G["L"] * s)
    else:
        Pz = FW.motor_slope * G0 / s * np.exp(-s * Ts / 2)
    return (Kp + Ki * Ts * z / (z - 1) + Kd * (z - 1) / (Ts * z)) * Pz


def pi_for(G, wc, pm_deg, Ts=None):
    """Kp, Ki that put the crossover at wc with the given phase margin."""
    P = freq_L(G, 1, 0, np.array([wc]), Ts)[0]           # plant alone (Kp=1, Ki=0)
    Ct = np.exp(1j * math.radians(-180 + pm_deg)) / P   # the controller we need there
    s = 1j * wc
    A = 1 / s if Ts is None else Ts * np.exp(s * Ts) / (np.exp(s * Ts) - 1)
    Ki = Ct.imag / A.imag
    Kp = Ct.real - Ki * A.real
    return Kp, Ki


def margins(G, Kp, Ki, Ts=None, Kd=0.0):
    w = np.logspace(-2, math.log10(math.pi / (Ts or 0.01)) - 1e-3, 4000)
    L = freq_L(G, Kp, Ki, w, Ts, Kd)
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    i = np.nonzero(np.diff(np.sign(mag - 1)))[0]
    wc = w[i[0]] if len(i) else float("nan")
    pm = math.degrees(np.interp(wc, w, ph)) + 180 if len(i) else float("nan")
    j = np.nonzero(np.diff(np.sign(ph + math.pi)))[0]
    gm = -20 * math.log10(np.interp(w[j[0]], w, mag)) if len(j) else float("inf")
    return {"wc": wc, "pm": pm, "gm_db": gm}


# ----------------------------------------------------------------- simulation
def trapezoid(D, T, ta, td, t):
    Sn = D / (T - (ta + td) / 2)
    t = np.clip(t, 0, T)
    t2 = T - td
    x1, x2 = Sn * ta / 2, Sn * ta / 2 + Sn * (t2 - ta)
    acc, cru, dec = t < ta, (t >= ta) & (t < t2), t >= t2
    v = acc * (Sn * t / ta) + cru * Sn + dec * (Sn * (T - t) / td)
    x = (acc * (Sn * t ** 2 / (2 * ta)) + cru * (x1 + Sn * (t - ta))
         + dec * (x2 + Sn * (t - t2) - Sn * (t - t2) ** 2 / (2 * td)))
    return v, x


RUN = {"D": 100.0, "T": 8.0, "ta": 2.0, "td": 2.0}


def sim_loop(G, p0, gains, mode, Ts=TS, run=RUN):
    """Port of sim_loop.m (the model-based simulation, not the virtual car)."""
    dt = 1e-3
    t = np.arange(0, run["T"] + 2 + 1e-9, dt)
    spd_ref, pos_ref = trapezoid(run["D"], run["T"], run["ta"], run["td"], t)
    pl = Plant(G, dt)
    nTs = max(1, round(Ts / dt))
    disc = mode == "discrete"
    pos, spd, pct_log = np.zeros(len(t)), np.zeros(len(t)), np.zeros(len(t))
    p_now = I = e_last = pct = 0.0
    for k in range(len(t)):
        if not disc or k % nTs == 0:
            h = nTs * dt if disc else dt
            meas = math.floor(p_now * FW.cnt_cm) / FW.cnt_cm if disc else p_now
            if t[k] <= run["T"]:
                e = pos_ref[k] - meas
                I = min(max(I + e * h, -FW.i_limit), FW.i_limit)
                D = (e - e_last) / h
                e_last = e
                u = max(spd_ref[k] + gains["Kp"] * e + gains["Ki"] * I + gains["Kd"] * D, 1.0)
                pct = min(FW.motor_min + FW.motor_slope * u, 100.0)
            else:
                pct = 0.0
        v = pl.step(max(pct - p0, 0.0))
        p_now += v * dt
        pos[k], spd[k], pct_log[k] = p_now, v, pct
    run_ = t <= run["T"]
    return {"t": t, "pos": pos, "spd": spd, "pct": pct_log, "pos_ref": pos_ref, "spd_ref": spd_ref,
            "rms": float(np.sqrt(np.mean((pos_ref[run_] - pos[run_]) ** 2))),
            "max": float(np.max(np.abs(pos_ref[run_] - pos[run_]))),
            "final_err": float(run["D"] - pos[-1])}


def linear_step(G, gains, Ts=None, t_end=6.0):
    """1 cm position step on the linear loop (no feed-forward, no dead zone)."""
    dt = 1e-3
    t = np.arange(0, t_end, dt)
    pl = Plant(G, dt)
    nTs = round(Ts / dt) if Ts else 1
    y = np.zeros(len(t))
    pos = I = u = e_last = 0.0
    for k in range(len(t)):
        if k % nTs == 0:
            h = nTs * dt
            e = 1.0 - pos
            I += e * h
            D = (e - e_last) / h if k else 0.0     # no kick from the step itself
            e_last = e
            u = gains["Kp"] * e + gains["Ki"] * I + gains.get("Kd", 0.0) * D
        pos += pl.step(FW.motor_slope * u) * dt
        y[k] = pos
    return t, y


def step2(R, G):
    # Phase margin target. 60 deg limits the crossover to ~1.2 rad/s and leaves
    # almost no integral action; 35 deg is the compromise between tracking and
    # margin (see the report, table of PM targets).
    pm = 35.0
    wcs = np.linspace(0.3, 4.0, 75)
    # mismatch cases for the robustness check: the car's p0 and K are off from
    # the feed-forward in the firmware
    cases = [(dp, kk) for dp in (-5, 0, 5) for kk in (0.8, 1.0, 1.2)]
    p0 = R["step1"]["p0"]
    sweep = []
    for wc in wcs:
        Kp, Ki = pi_for(G, wc, pm, TS)
        if Kp <= 0 or Ki <= 0:
            continue
        mg = margins(G, Kp, Ki, TS)
        if not mg["gm_db"] > 6:
            continue
        g = {"Kp": Kp, "Ki": Ki, "Kd": 0.0}
        worst = max(sim_loop({**G, "K": G["K"] * kk}, p0 + dp, g, "discrete")["rms"] for dp, kk in cases)
        sweep.append({"wc": float(wc), "Kp": Kp, "Ki": Ki, "gm_db": mg["gm_db"], "worst_rms": worst})
    best = min(sweep, key=lambda r: r["worst_rms"])
    wc = best["wc"]
    Kpc, Kic = pi_for(G, wc, pm, None)
    Kpd, Kid = pi_for(G, wc, pm, TS)
    gc = {"Kp": Kpc, "Ki": Kic, "Kd": 0.0}
    gd = {"Kp": Kpd, "Ki": Kid, "Kd": 0.0}
    gfw = {"Kp": FW.Kp, "Ki": FW.Ki, "Kd": FW.Kd}

    table = {
        "continuous": {**gc, **margins(G, Kpc, Kic, None)},
        "discrete": {**gd, **margins(G, Kpd, Kid, TS)},
        "firmware": {**gfw, **margins(G, FW.Kp, FW.Ki, TS, FW.Kd)},
    }
    # closed-loop bandwidth (-3 dB) of the continuous loop
    w = np.logspace(-2, 2, 4000)
    L = freq_L(G, Kpc, Kic, w, None)
    T = np.abs(L / (1 + L))
    wb = float(w[np.argmax(T < 1 / math.sqrt(2))])
    R["step2"] = {"pm_target": pm, "wc": wc, "sweep": sweep, "gains": table,
                  "bandwidth": wb, "ws": 2 * math.pi / TS, "ws_over_wb": 2 * math.pi / TS / wb}

    fig, ax = plt.subplots(figsize=(6.0, 2.9))
    for i, (lab, g, ts) in enumerate([("continuous", gc, None), (f"discrete Ts = {TS:.1f} s", gd, TS),
                                      ("firmware gains, discrete", gfw, TS)]):
        t, y = linear_step(G, g, ts)
        ax.plot(t, y, color=C[i], label=f"{lab}: Kp {g['Kp']:.2f}, Ki {g['Ki']:.2f}")
    ax.axhline(1, color=REF, lw=0.8, ls="--")
    ax.set(xlabel="t (s)", ylabel="position (cm)", title="Position step of 1 cm (no feed-forward)")
    ax.legend(loc="lower right", fontsize=8)
    save(fig, "step_response.png")

    fig, ax = plt.subplots(2, 1, figsize=(6.0, 4.2), sharex=True)
    w = np.logspace(-1, math.log10(math.pi / TS) - 1e-3, 1500)
    for i, (lab, g, ts) in enumerate([("continuous", gc, None), ("discrete", gd, TS), ("firmware", gfw, TS)]):
        L = freq_L(G, g["Kp"], g["Ki"], w, ts, g["Kd"])
        mg = margins(G, g["Kp"], g["Ki"], ts, g["Kd"])
        ax[0].semilogx(w, 20 * np.log10(np.abs(L)), color=C[i], label=f"{lab}: PM {mg['pm']:.0f}°, wc {mg['wc']:.2f} rad/s")
        ax[1].semilogx(w, np.degrees(np.unwrap(np.angle(L))), color=C[i])
    ax[0].axhline(0, color=REF, lw=0.8, ls="--"); ax[1].axhline(-180, color=REF, lw=0.8, ls="--")
    ax[0].set(ylabel="|L| (dB)", title="Open-loop Bode: PI x plant"); ax[0].legend(fontsize=8)
    ax[1].set(xlabel="w (rad/s)", ylabel="phase (deg)")
    save(fig, "bode.png")
    return gc, gd, gfw


# ----------------------------------------------------------------- step 3
def step3(R, G, p0, gc, gd):
    rc = sim_loop(G, p0, gc, "continuous")
    rd = sim_loop(G, p0, gd, "discrete")
    Sn = RUN["D"] / (RUN["T"] - (RUN["ta"] + RUN["td"]) / 2)
    ts_rows = []
    fig2, ax2 = plt.subplots(figsize=(6.4, 2.9))
    ax2.plot(rc["t"], rc["pos_ref"] - rc["pos"], color="#222222", lw=2.2, label="continuous")
    for i, Ts in enumerate((0.02, 0.05, 0.1, 0.2, 0.4)):
        r = sim_loop(G, p0, gd, "discrete", Ts)
        ts_rows.append({"Ts": Ts, "rms": r["rms"], "max": r["max"], "final_err": r["final_err"]})
        ax2.plot(r["t"], r["pos_ref"] - r["pos"], color=C[i], lw=1.2, label=f"Ts = {Ts:.2f} s")
    ax2.set(xlabel="t (s)", ylabel="position error (cm)", title="Tracking error for different sampling times")
    ax2.legend(fontsize=8, ncol=2)
    save(fig2, "ts_sweep.png")
    R["step3"] = {"Sn": Sn, "continuous": {k: rc[k] for k in ("rms", "max", "final_err")},
                  "discrete": {k: rd[k] for k in ("rms", "max", "final_err")}, "ts_sweep": ts_rows}

    fig, ax = plt.subplots(3, 1, figsize=(6.4, 5.6), sharex=True)
    ax[0].plot(rc["t"], rc["pos_ref"], color=REF, ls="--", lw=1, label="reference Pos_Ref")
    ax[0].plot(rc["t"], rc["pos"], color=C[0], label="continuous")
    ax[0].plot(rd["t"], rd["pos"], color=C[1], label=f"discrete Ts = {TS:.1f} s")
    ax[0].set(ylabel="position (cm)", title=f"Timed run: {RUN['D']:.0f} cm in {RUN['T']:.0f} s (ta = td = 2 s)")
    ax[0].legend(loc="lower right", fontsize=8)
    ax[1].plot(rc["t"], rc["spd_ref"], color=REF, ls="--", lw=1, label="SP_Vel")
    ax[1].plot(rc["t"], rc["spd"], color=C[0]); ax[1].plot(rd["t"], rd["spd"], color=C[1])
    ax[1].set(ylabel="speed (cm/s)"); ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(rc["t"], rc["pct"], color=C[0]); ax[2].plot(rd["t"], rd["pct"], color=C[1])
    ax[2].set(ylabel="motor %", xlabel="t (s)")
    save(fig, "cont_vs_disc.png")


# ----------------------------------------------------------------- step 4
def step4(R, G, p0, gd):
    tel = load_telemetry(DATA / "closedloop_run1.csv")
    st = max(int(np.argmax(tel["ref_cm"] > 0)) - 1, 0)
    rl = {k: v[st:] for k, v in tel.items()}
    rl["t"] = rl["t"] - rl["t"][0]
    gfw = {"Kp": FW.Kp, "Ki": FW.Ki, "Kd": FW.Kd}
    rs = sim_loop(G, p0, gfw, "discrete")
    inrun = rl["t"] <= RUN["T"]
    car_rms = float(np.sqrt(np.mean((rl["ref_cm"][inrun] - rl["dist_cm"][inrun]) ** 2)))
    # the simulated position at the log's sample times
    sim_at = np.interp(rl["t"], rs["t"], rs["pos"])
    R["step4"] = {"car_final": float(rl["dist_cm"][-1]), "sim_final": float(rs["pos"][-1]),
                  "car_rms": car_rms, "sim_rms": rs["rms"],
                  "car_vs_sim_rms": float(np.sqrt(np.mean((rl["dist_cm"] - sim_at) ** 2)))}

    fig, ax = plt.subplots(3, 1, figsize=(6.4, 5.6), sharex=True)
    ax[0].plot(rs["t"], rs["pos_ref"], color=REF, ls="--", lw=1, label="reference")
    ax[0].plot(rl["t"], rl["dist_cm"], "o", color=C[0], ms=3, label="virtual car log (Real_Dist)")
    ax[0].plot(rs["t"], rs["pos"], color=C[1], lw=1.2, label="simulation, identified model")
    ax[0].set(ylabel="position (cm)", title="Validation: virtual car vs simulation (Kp 3.8, Ki 2.5, Kd 0.05)")
    ax[0].legend(loc="lower right", fontsize=8)
    ax[1].plot(rs["t"], rs["spd_ref"], color=REF, ls="--", lw=1, label="SP_Vel")
    ax[1].plot(rl["t"], rl["spd_cms"], "o", color=C[0], ms=3, label="AverageSpeed")
    ax[1].plot(rs["t"], rs["spd"], color=C[1], lw=1.2)
    ax[1].set(ylabel="speed (cm/s)"); ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(rl["t"], rl["mot_pct"], "o", color=C[0], ms=3)
    ax[2].plot(rs["t"], rs["pct"], color=C[1], lw=1.2)
    ax[2].set(ylabel="motor %", xlabel="t (s)")
    save(fig, "validation.png")


def sensitivity(R, gd):
    """Tuned vs firmware gains on the firmware-exact virtual car, when the real
    car differs from the assumptions."""
    rows = []
    base = Car()
    variants = [("nominal", base),
                ("tau 0.10 s", replace(base, tau=0.10)), ("tau 0.50 s", replace(base, tau=0.50)),
                ("p0 45 %", replace(base, p0=45)), ("p0 55 %", replace(base, p0=55)),
                ("K -20 %", replace(base, K=base.K * 0.8)), ("K +20 %", replace(base, K=base.K * 1.2))]
    for name, car in variants:
        row = {"case": name}
        for lab, g in (("tuned", gd), ("firmware", {"Kp": FW.Kp, "Ki": FW.Ki, "Kd": FW.Kd})):
            fw = replace(FW, Kp=g["Kp"], Ki=g["Ki"], Kd=g["Kd"])
            log, final = closedloop_run(car, fw)
            tel = load_telemetry_lines(log)
            inrun = (tel["ref_cm"] > 0) & (tel["ref_cm"] < 100)
            err = tel["ref_cm"][inrun] - tel["dist_cm"][inrun]
            row[lab] = {"final": final, "rms": float(np.sqrt(np.mean(err ** 2))), "max": float(np.max(np.abs(err)))}
        rows.append(row)
    R["sensitivity"] = rows


def load_telemetry_lines(lines):
    p = FIG / "_tmp.csv"
    p.write_text("\n".join(lines))
    tel = load_telemetry(p)
    p.unlink()
    return tel


def main():
    R = {}
    G, p0 = step1(R)
    gc, gd, gfw = step2(R, G)
    step3(R, G, p0, gc, gd)
    step4(R, G, p0, gd)
    sensitivity(R, gd)
    for k in list(R["step2"]):
        if k == "sweep":
            R["step2"][k] = [{kk: round(v, 4) for kk, v in r.items()} for r in R["step2"][k]]
    (HERE / "results.json").write_text(json.dumps(R, indent=1, default=float))
    s1, s2 = R["step1"], R["step2"]
    print(f"p0 {s1['p0']:.2f} %, k_static {s1['k_static']:.3f}")
    for k, m in s1["models"].items():
        print(f"  {k:10s} fit {m['fit']:.1f} %  {m['str']}")
    print(f"chosen {s1['best']}, hold-out P70 fit {s1['holdout_fit_P70']:.1f} %")
    print(f"wc {s2['wc']:.2f} rad/s; bandwidth {s2['bandwidth']:.2f}; ws/wb {s2['ws_over_wb']:.0f}")
    for k, g in s2["gains"].items():
        print(f"  {k:10s} Kp {g['Kp']:.3f} Ki {g['Ki']:.3f} Kd {g['Kd']:.3f} PM {g['pm']:.1f} wc {g['wc']:.2f} GM {g['gm_db']:.1f} dB")
    print("step3", json.dumps(R["step3"], default=float))
    print("step4", json.dumps(R["step4"], default=float))
    for r in R["sensitivity"]:
        print(f"  {r['case']:11s} tuned final {r['tuned']['final']:.1f} rms {r['tuned']['rms']:.2f} max {r['tuned']['max']:.2f} | "
              f"fw final {r['firmware']['final']:.1f} rms {r['firmware']['rms']:.2f} max {r['firmware']['max']:.2f}")


if __name__ == "__main__":
    main()
