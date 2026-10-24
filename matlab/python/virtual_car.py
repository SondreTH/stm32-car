"""Virtual car: a stand-in for the real car, used because the real tests could
not be recorded (battery flat at 9.4 V, labs closed; simulation agreed with the
professor). Everything it writes is marked SIMULATED.

It reproduces the firmware as closely as possible:
  - plant: speed = K / (tau s + 1) * max(motor% - p0, 0), integrated every 1 ms
  - encoder: whole counts only, 1170 counts per metre (COUNTS_PER_METER)
  - TIM3 interrupt every 100 ms: drive_update() in car_drive.c, including
    Speed_CM_SEC = delta counts / 11.7 / 0.1 and AverageSpeed = 0.5*prev + 0.5*raw,
    the trapezoid state machine, Pos_Ref, ControlPID() and Apply_Motor_PWM()
  - telemetry (T 10) every 100 ms from the main loop, same CSV columns

The plant parameters are the ASSUMPTIONS of this work. p0 and K come from the
feed-forward Hakon tuned on the real car (commit b95dbe3, "Driving exactly 1
meter"): offset 7200/14400 = 50 % PWM and 80 counts per cm/s = 0.556 % per cm/s,
i.e. speed = (motor% - 50) / 0.556 = 1.8 (motor% - 50). tau was not measured.
"""
import math
from dataclasses import dataclass, field, replace
from pathlib import Path


@dataclass
class Car:
    p0: float = 50.0          # % : motor % where the car starts to move   (from Hakon's MOTOR_MIN_PCT)
    K: float = 1.8            # cm/s per % above p0                        (from 1 / MOTOR_PCT_PER_CMS)
    tau: float = 0.25         # s : mechanical time constant               (ASSUMED, typical small gear motor)
    tau_brake: float = 0.05   # s : how fast the speed dies on X (brake)   (ASSUMED)


@dataclass
class Firmware:
    """Values from car_config.h / car_drive.c."""
    Ts: float = 0.100
    cpm: float = 1170.0
    motor_min: float = 7200 / 144
    motor_slope: float = 80 / 144
    Kp: float = 3.8
    Ki: float = 2.5
    Kd: float = 0.05
    i_limit: float = 25.0

    @property
    def cnt_cm(self):
        return self.cpm / 100


HEADER = "ms,cnt,dist_cm,spd_cms,sp_cms,mot_pct,ref_cm,sonar_cm"


class Sim:
    """One power-on of the virtual car. Call step() every 1 ms."""
    DT = 0.001

    def __init__(self, car: Car, fw: Firmware, tick0_ms=5000, isr_phase_ms=37):
        self.car, self.fw = car, fw
        self.ms = tick0_ms                 # HAL_GetTick()
        self.isr_phase = isr_phase_ms      # TIM3 is not in phase with telemetry
        self.x = 0.0                       # true position (cm)
        self.v = 0.0                       # true speed (cm/s)
        self.motor = 0.0                   # motor %, 0 = coast
        self.brake = False
        # firmware variables
        self.last_pos = 0
        self.real_dist = 0.0
        self.avg_speed = 0.0
        self.state = 0
        self.time_ctl = 0.0
        self.sp_vel = 0.0
        self.pos_ref = 0.0
        self.err_acum = 0.0
        self.last_err = 0.0
        self.log = []
        self.telem = False
        self.next_telem = None

    # ---- commands (as handle_command in car_app.c) ----
    def cmd_P(self, pct):
        self.state = 0
        self.brake = False
        self.motor = min(max(pct, 0.0), 100.0)

    def cmd_X(self):
        self.state = 0
        self.motor = 0.0
        self.brake = True

    def cmd_T(self, hz):
        self.telem = hz > 0
        if self.telem:
            self.log.append(HEADER)
            self.next_telem = self.ms

    def cmd_timed_run(self, dist_m, T, ta, td):
        """Start_Timed_Profile() + reset_run()."""
        self.D = dist_m * 100
        self.Sn = self.D / (T - 0.5 * (ta + td))
        self.t1, self.T, self.t2 = ta, T, T - td
        self.time_ctl = 0.0
        self.sp_vel = 0.0
        self.pos_ref = 0.0
        self.err_acum = 0.0
        self.last_err = 0.0
        self.x = 0.0                       # the encoder is cleared at the start of a run
        self.last_pos = 0
        self.avg_speed = 0.0
        self.brake = False
        self.state = 1

    # ---- firmware ----
    def counts(self):
        return math.floor(self.x * self.fw.cnt_cm)

    def isr(self):
        """drive_update(), every Ts."""
        fw = self.fw
        h = fw.Ts
        position = abs(self.counts())
        self.real_dist = position / fw.cnt_cm
        delta = position - self.last_pos
        self.last_pos = position
        speed = (delta / fw.cnt_cm) / h
        self.avg_speed = 0.5 * self.avg_speed + 0.5 * speed
        if self.state == 0:
            return
        self.time_ctl += h
        if self.state == 1:
            self.sp_vel = self.time_ctl * self.Sn / self.t1
            if self.time_ctl >= self.t1 - 1e-9:
                self.state = 2
        elif self.state == 2:
            self.sp_vel = self.Sn
            if self.time_ctl >= self.t2 - 1e-9:
                self.state = 3
        elif self.state == 3:
            self.sp_vel = self.Sn * (self.T - self.time_ctl) / (self.T - self.t2)
            if self.time_ctl >= self.T - 1e-9:
                self.state = 0
                self.sp_vel = 0.0
        self.pos_ref += self.sp_vel * h
        if self.state == 0:
            self.sp_vel = 0.0
            self.pos_ref = self.D
            self.motor = 0.0               # Apply_Motor_PWM(0): coast
            return
        e = self.pos_ref - self.real_dist
        self.err_acum = min(max(self.err_acum + e * h, -fw.i_limit), fw.i_limit)
        rate = (e - self.last_err) / h
        self.last_err = e
        u = self.sp_vel + fw.Kp * e + fw.Ki * self.err_acum + fw.Kd * rate
        u = max(u, 1.0)
        self.motor = 0.0 if u <= 0.2 else min(fw.motor_min + u * fw.motor_slope, 100.0)

    def step(self):
        c = self.car
        if self.brake:
            self.v += (-self.v) * self.DT / c.tau_brake
        else:
            drive = c.K * max(self.motor - c.p0, 0.0)
            self.v += (drive - self.v) * self.DT / c.tau
        self.v = max(self.v, 0.0)
        self.x += self.v * self.DT
        self.ms += 1
        if (self.ms - self.isr_phase) % round(self.fw.Ts * 1000) == 0:
            self.isr()
        if self.telem and self.ms >= self.next_telem:
            self.next_telem += 100
            self.log.append(f"{self.ms},{self.counts()},{self.real_dist:.1f},{self.avg_speed:.1f},"
                            f"{self.sp_vel:.1f},{self.motor:.1f},{self.pos_ref:.1f},400")

    def wait(self, s):
        for _ in range(round(s / self.DT)):
            self.step()


def note(car: Car, what: str):
    return [f"# SIMULATED DATA - virtual car (matlab/python/virtual_car.py), NOT measured on the real car.",
            f"# {what}",
            f"# plant: speed = {car.K:.3f}/({car.tau:.3f} s + 1) * max(motor% - {car.p0:.1f}, 0); "
            f"p0 and K from the feed-forward tuned on the real car, tau assumed."]


def openloop_test(car: Car, fw: Firmware, pct: float):
    """README procedure: T 10, wait 1 s, P xx, wait 3 s, X, T 0."""
    s = Sim(car, fw)
    s.wait(0.5)
    s.cmd_T(10); s.wait(1.0)
    s.cmd_P(pct); s.wait(3.0)
    s.cmd_X(); s.wait(0.5)
    s.cmd_T(0)
    return note(car, f"Open-loop test: T 10, wait 1 s, P {pct:.0f}, wait 3 s, X, T 0") + s.log


def closedloop_run(car: Car, fw: Firmware, dist_m=1.0, T=8.0, ta=2.0, td=2.0):
    """README procedure: tt 8, ta 2, td 2, T 10, d1, wait, T 0. Returns (log, final cm)."""
    s = Sim(car, fw)
    s.wait(0.5)
    s.cmd_T(10); s.wait(0.3)
    s.cmd_timed_run(dist_m, T, ta, td)
    s.wait(T + 2.0)
    s.cmd_T(0)
    hdr = note(car, f"Closed-loop timed run: tt {T:g}, ta {ta:g}, td {td:g}, d{dist_m:g}, "
                    f"Kp {fw.Kp:g} Ki {fw.Ki:g} Kd {fw.Kd:g}; final position {s.x:.1f} cm")
    return hdr + s.log, s.x


def main():
    out = Path(__file__).resolve().parents[1] / "data_sim"
    out.mkdir(exist_ok=True)
    car, fw = Car(), Firmware()
    for pct in (60, 70, 80, 90):
        (out / f"openloop_P{pct}.csv").write_text("\n".join(openloop_test(car, fw, pct)) + "\n")
    log, final = closedloop_run(car, fw)
    (out / "closedloop_run1.csv").write_text("\n".join(log) + "\n")
    print(f"Wrote {out}: 4 open-loop tests and 1 timed run (final position {final:.1f} cm)")


if __name__ == "__main__":
    main()
