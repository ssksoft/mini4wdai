# -*- coding: utf-8 -*-
"""On-board speed controller of the Mini 4WD, driven ONLY by the 6-axis IMU (no unreal import: testable offline).

  ImuSensor   : virtual LSM6DS3 - body-frame specific force [g] and angular rate [dps] from the car motion
  Estimator   : strapdown attitude (gyro integration), heading psi, forward speed v_hat (accel integration)
  Controller  : PWM duty 0..1 (single low-side switch: no brake, no reverse) from heading-based course phase

Body axes: x forward, y left, z up (right-handed).  World: right-handed, z up.  Clockwise running -> psi decreases.

Course phase from the heading only:
  start (far straight, heading 0) -> 4 right curves (psi 0 -> -180) -> LANE CHANGE (S-bend, net 0)
  -> 4 right curves (-180 -> -360) -> 2 straights -> next lap.
  The n-th lane-change crossing happens around psi = -180 - 360 n.
  The car alternates lanes every lap, so crossings 0, 2, 4 ... go OVER the bridge (inner-lane start),
  crossings 1, 3 ... go UNDER it.
"""
import math

G = 9.80665


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_t(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def _mat_vec(A, v):
    return [A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3)]


def _orthonormalize(R):
    # columns = body axes in world
    x = [R[0][0], R[1][0], R[2][0]]
    z = [R[0][2], R[1][2], R[2][2]]
    n = math.sqrt(sum(c * c for c in x)); x = [c / n for c in x]
    d = sum(a * b for a, b in zip(x, z)); z = [a - d * b for a, b in zip(z, x)]
    n = math.sqrt(sum(c * c for c in z)); z = [c / n for c in z]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[i], y[i], z[i]] for i in range(3)]


def body_matrix(f, l, u):
    """3x3 matrix whose columns are the body axes (forward, left, up) in world coordinates"""
    return [[f[i], l[i], u[i]] for i in range(3)]


class ImuSensor:
    """6-axis IMU sampled at RATE_HZ like the real logger (LSM6DS3, 200 Hz).
    Each sample = mean specific force / mean angular rate over the sample period (sensor low-pass),
    range +-16 g / +-2000 dps, quantised to 0.001 g / 0.1 dps."""
    RATE_HZ = 200.0

    def __init__(self):
        self.R = None
        self.v = None
        self.acc_t = 0.0

    def update(self, dt, R, v_world):
        """feed the true body motion every physics tick; returns (dt_sample, acc_g, gyro_dps) or None"""
        if self.R is None:
            self.R, self.v, self.acc_t = R, v_world, 0.0
            return None
        self.acc_t += dt
        if self.acc_t < 1.0 / self.RATE_HZ - 1e-6:
            return None
        h = self.acc_t
        a = [(v_world[i] - self.v[i]) / h for i in range(3)]
        a[2] += G
        Rm = _orthonormalize([[0.5 * (self.R[i][j] + R[i][j]) for j in range(3)] for i in range(3)])
        f = _mat_vec(_mat_t(Rm), a)
        M = _mat_mul(_mat_t(self.R), R)
        w = [0.5 * (M[2][1] - M[1][2]) / h, 0.5 * (M[0][2] - M[2][0]) / h, 0.5 * (M[1][0] - M[0][1]) / h]
        self.R, self.v, self.acc_t = R, v_world, 0.0
        acc_g = [max(-16.0, min(16.0, round(c / G, 3))) for c in f]
        gyro = [max(-2000.0, min(2000.0, round(math.degrees(c), 1))) for c in w]
        return h, acc_g, gyro


class Estimator:
    """strapdown navigation from IMU samples only"""

    def __init__(self):
        self.R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]   # start: level, heading 0
        self.vw = [0.0, 0.0, 0.0]   # velocity estimate in the navigation frame [m/s]
        self.v = 0.0            # speed estimate [m/s]
        self.psi = 0.0          # unwrapped heading [deg]
        self._yaw_prev = 0.0
        self.pitch = 0.0        # [deg], nose up positive

    def update(self, dt, acc_g, gyro_dps):
        w = [math.radians(c) * dt for c in gyro_dps]
        dR = [[1.0, -w[2], w[1]], [w[2], 1.0, -w[0]], [-w[1], w[0], 1.0]]
        self.R = _orthonormalize(_mat_mul(self.R, dR))
        yaw = math.degrees(math.atan2(self.R[1][0], self.R[0][0]))
        d = (yaw - self._yaw_prev + 180.0) % 360.0 - 180.0
        self.psi += d
        self._yaw_prev = yaw
        self.pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.R[2][0]))))
        a_nav = _mat_vec(self.R, [c * G for c in acc_g])  # specific force -> navigation frame
        a_nav[2] -= G                                      # + gravity
        self.vw = [self.vw[i] + a_nav[i] * dt for i in range(3)]
        self.v = math.sqrt(sum(c * c for c in self.vw))


DEFAULT_CTRL = dict(
    v_over=2.85,       # (legacy) target speed at the foot of the up-ramp
    v_bridge=3.0,      # "virtual motor" top speed used from emu_from deg before the lane change to the bridge exit [m/s]
    kp_bridge=7.6,     # gain of that law: F = F0/vmax*(1+kp)*(v_bridge - v) = same force curve as a 3.0 m/s-geared motor
    emu_from=-1.0,     # heading before the lane change where the virtual-motor law starts; <0: only on the bridge
    a_coast=4.0,       # deceleration assumed for planning the coasting point [m/s^2] (1st crossing)
    a_coast_late=-1.0, # same for the later over crossings (car arrives much faster); <0: = a_coast
    r_eff=0.37,        # effective curve radius (heading -> distance) [m]
    l_str=0.93,        # length of the two-straight section [m]
    phi_post=35.0,     # deg after the lane-change heading (into the next curve) where full throttle resumes
    v_under=99.0,      # speed limit when crossing UNDER the bridge (99 = none)
    kp=4.0,            # proportional gain of the speed loop elsewhere
    first_over=1,      # 1: the first crossing goes over the bridge (inner-lane start)
)


class Controller:
    def __init__(self, vmax, params=None):
        self.p = dict(DEFAULT_CTRL)
        if params:
            self.p.update({k: v for k, v in params.items() if k in DEFAULT_CTRL})
        self.vmax = vmax
        self.imu = ImuSensor()
        self.est = Estimator()
        self.n = 0              # index of the next lane-change crossing
        self.zone = False
        self.duty = 1.0
        self.max_pitch = 0.0
        self.rem = 180.0
        self.v_lim = 99.0
        self.on_bridge = False
        self.s_str = 0.5 * self.p["l_str"]     # the start line is in the middle of the straight section

    def over(self, n):
        return (n % 2 == 0) == bool(self.p["first_over"])

    def speed_duty(self, v_target, kp=None):
        if v_target >= 90.0:
            return 1.0
        kp = self.p["kp"] if kp is None else kp
        d = v_target / self.vmax + kp * (v_target - self.est.v) / self.vmax
        return max(0.0, min(1.0, d))

    def distance_to_go(self, rem, h):
        """distance [m] to the foot of the next up-ramp, from the heading still to turn (+ odometry on the straight)"""
        p, d = self.p, 5.0
        if rem > 180.0 + d:                              # curves after the previous lane change
            self.s_str = 0.0
            return p["r_eff"] * math.radians(rem - 180.0) + p["l_str"] + p["r_eff"] * math.pi
        if rem >= 180.0 - d:                             # on the straights: heading constant -> integrate speed
            self.s_str += self.est.v * h
            return max(0.0, p["l_str"] - self.s_str) + p["r_eff"] * math.radians(max(0.0, rem))
        return p["r_eff"] * math.radians(max(0.0, rem))   # last four curves

    def step(self, dt, R, v_world):
        """call every physics tick; the estimator and the duty update at the IMU rate (zero-order hold between)"""
        sample = self.imu.update(dt, R, v_world)
        if sample is None:
            return self.duty, None
        h, acc, gyro = sample
        self.est.update(h, acc, gyro)
        p = self.p
        rem = self.est.psi - (-180.0 - 360.0 * self.n)        # heading still to turn before the n-th lane change
        if rem < -p["phi_post"]:                              # lane change passed -> next one
            self.n += 1
            rem = self.est.psi - (-180.0 - 360.0 * self.n)
            self.on_bridge = False
        over = self.over(self.n)
        self.rem = rem
        if over and rem <= 0.0:
            self.on_bridge = True
        kp = None
        emu = p["emu_from"] >= 0.0
        if over and (self.on_bridge or (emu and rem <= p["emu_from"])):
            # virtual low-geared motor: same force-speed curve as the robust 3.0 m/s gearing
            v_lim, kp = p["v_bridge"], p["kp_bridge"]
        elif over:                                            # braking-point planning (coast only, no brake)
            s_go = self.distance_to_go(rem, h)
            ac = p["a_coast"] if (self.n == 0 or p["a_coast_late"] < 0) else p["a_coast_late"]
            if emu:                                           # reach v_bridge where the virtual motor takes over
                v_lim = math.sqrt(p["v_bridge"] ** 2 + 2.0 * ac * max(0.0, s_go - p["r_eff"] * math.radians(p["emu_from"])))
            else:
                v_lim = math.sqrt(p["v_over"] ** 2 + 2.0 * ac * s_go)
        else:
            v_lim = p["v_under"] if rem < 60.0 else 99.0
        self.zone = over and (self.on_bridge or (emu and rem <= p["emu_from"]))
        self.v_lim = v_lim
        self.duty = self.speed_duty(v_lim, kp)
        return self.duty, (acc, gyro)
