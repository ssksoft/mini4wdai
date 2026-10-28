# -*- coding: utf-8 -*-
"""Virtual IMU (6-axis, like the LSM6DS3 on the real car) from the UE body-motion log.

  python imu_from_log.py imu_raw_log.csv imu_sim_200Hz.csv [rate_hz]

Input : MiniYonkuSim/Saved/IMU/imu_raw_log.csv  (written by ue/record_imu.py; right-handed world, metres)
Output: same columns as the real logger (get_imu/get_imu.ino -> Mini4AI_*.csv)
        time_s,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g   + speed_mps,lap,grounded (extra, for analysis)

Sensor model
  * mounted at the car origin, axes: x = forward, y = left, z = up (right-handed)
  * accelerometer = specific force (a - g) in the body frame, in g  -> 0 0 +1 at rest
  * gyroscope     = body angular rate, deg/s
  * each sample is the average over the previous sample period (like the sensor's internal low-pass)
  * range clipped to +-16 g / +-2000 dps, resolution 0.001 g / 0.1 dps (as stored by the real logger)
"""
import sys
import numpy as np

G = 9.80665
ACC_RANGE_G, GYRO_RANGE_DPS = 16.0, 2000.0


def load(path):
    head = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("#"):
                head.append(line[1:].strip())
            else:
                cols = line.strip().split(",")
                break
    d = np.genfromtxt(path, delimiter=",", comments="#", skip_header=len(head) + 1)
    return head, {c: d[:, i] for i, c in enumerate(cols)}


def interp(t, tq, y):
    return np.stack([np.interp(tq, t, y[:, i]) for i in range(y.shape[1])], axis=1)


def rot(t, tq, D):
    """body-to-world rotation matrices at times tq (columns forward, left, up), re-orthonormalised"""
    f = interp(t, tq, np.c_[D["fx"], D["fy"], D["fz"]])
    u = interp(t, tq, np.c_[D["ux"], D["uy"], D["uz"]])
    f /= np.linalg.norm(f, axis=1, keepdims=True)
    u -= f * np.sum(u * f, axis=1, keepdims=True)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    l = np.cross(u, f)
    return np.stack([f, l, u], axis=2)          # R[:, :, k] = k-th body axis in world


def compute(D, rate=200.0):
    t = D["t_s"]
    h = 1.0 / rate
    tk = np.arange(h, t[-1] + 1e-9, h)
    v = np.c_[D["vx_mps"], D["vy_mps"], D["vz_mps"]]
    v1, v0 = interp(t, tk, v), interp(t, tk - h, v)
    a_world = (v1 - v0) / h + np.array([0.0, 0.0, G])          # specific force (world)
    Rm, R0, R1 = rot(t, tk - h / 2, D), rot(t, tk - h, D), rot(t, tk, D)
    acc = np.einsum("nji,nj->ni", Rm, a_world) / G               # R^T * f  -> body, in g
    M = np.einsum("nji,njk->nik", R0, R1)                        # R0^T R1 = small body rotation
    w = 0.5 * np.c_[M[:, 2, 1] - M[:, 1, 2], M[:, 0, 2] - M[:, 2, 0], M[:, 1, 0] - M[:, 0, 1]] / h
    gyro = np.degrees(w)
    acc = np.clip(np.round(acc, 3), -ACC_RANGE_G, ACC_RANGE_G)
    gyro = np.clip(np.round(gyro, 1), -GYRO_RANGE_DPS, GYRO_RANGE_DPS)
    speed = np.linalg.norm(v1, axis=1)
    idx = np.clip(np.searchsorted(t, tk), 0, len(t) - 1)
    return tk, gyro, acc, speed, D["laps"][idx].astype(int), D["grounded"][idx].astype(int)


def main(src, dst, rate=200.0):
    head, D = load(src)
    tk, gyro, acc, speed, lap, gnd = compute(D, rate)
    with open(dst, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("time_s,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,speed_mps,lap,grounded\n")
        for i in range(len(tk)):
            fh.write("%.6f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%d,%d\n" % (
                tk[i], gyro[i, 0], gyro[i, 1], gyro[i, 2], acc[i, 0], acc[i, 1], acc[i, 2], speed[i], lap[i], gnd[i]))
    print("%d samples at %g Hz -> %s" % (len(tk), rate, dst))
    for line in head:
        print("  " + line)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 200.0)
