# -*- coding: utf-8 -*-
"""Mini 4WD runtime - runs while you Play In Editor (PIE).

The car body, rollers and tyres collide with the course through Chaos physics
(all frictionless, so the walls guide the car like the real rollers do).
This script adds, every frame:
  * motor drive force  (linear torque-speed curve, 3.5:1 gear, ~24 mm tyres)
  * tyre side grip     (per axle, limited by friction mu * load)
  * keyboard input, cameras, HUD (speed / lap times)

Two modes, chosen automatically from the level:
  * manual   (level Mini4WD_Oval)     - drive with the keyboard
  * scenario (level Mini4WD_CourseOut, has an actor tagged M4WD_SCENARIO)
      re-enacts the real test video "duty100 madraser":
        1) standard kit motor, full throttle, a few laps  -> no course-out
        2) Mach-Dash Motor PRO, full throttle             -> flies off after the lane change

Manual keys:   W/Up throttle  S/Down brake/reverse  R reset  1/2 lane  C camera  M motor  T slow motion
Scenario keys: R restart scenario  N next phase  C camera  T slow motion on/off
"""
import math
import os
import json
import builtins
import unreal
import m4wd_ctrl as CTRL

# ---------------------------------------------------------------- parameters (UE units: cm, s, kg)
# Motor model: a = A0 * (1 - v/VMAX) while the tyres touch the track (linear torque-speed curve).
# VMAX / A0 are calibrated so the lap times match the real video (standard ~1.9 s, Mach-Dash ~1.2 s per lap).
MOTORS = [  # name, VMAX (cm/s), A0 (cm/s^2), real-test note
    ("標準モーター（キット付属）", 400.0, 800.0, "実機: ラップ約1.9 s・コースアウトなし"),
    ("マッハダッシュモーターPRO", 700.0, 1300.0, "実機: ラップ約1.2 s・レーンチェンジ後にコースアウト"),
]
MU_TIRE = 1.0            # tyre / track friction
G = 980.0
ROLL_DECEL = 0.25        # coasting drag  (1/s)
BRAKE = 1200.0           # cm/s^2
REVERSE_SPEED = -80.0
AXLE_HALF = 4.0          # wheelbase 80 mm / 2
AXLE_H = 1.2             # tyre radius
CHASE_BACK, CHASE_UP = 32.0, 13.0

# scenario
SCN_STD_LAPS = 4                 # laps with the standard motor
SCN_MAX_LAPS = 12                # give up the Mach-Dash phase after this many laps without course-out
SCN_BANNER_TIME = 4.0            # seconds the course-out banner stays before the scenario restarts
SLOWMO = 0.3                     # time dilation near the lane-change exit (Mach-Dash phase)
SLOWMO_RADIUS = 45.0             # cm around the lane-change exit
OFFTRACK_Z = 0.35                # car origin below this (cm) = lying on the room floor, not on the track

TAG_CAR, TAG_CAM, TAG_FIN, TAG_SCN = "M4WD_CAR", "M4WD_CAM", "M4WD_FINISH", "M4WD_SCENARIO"
TAG_START = ["M4WD_START_0", "M4WD_START_1"]

S = {}


# ---------------------------------------------------------------- helpers
def _v(x, y, z):
    return unreal.Vector(float(x), float(y), float(z))


def _t(v):
    return (v.x, v.y, v.z)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _make_key(name):
    try:
        return unreal.Key(key_name=name)
    except Exception:
        pass
    k = unreal.Key()
    try:
        k.set_editor_property("key_name", name)
        return k
    except Exception:
        pass
    k.import_text('(KeyName="%s")' % name)
    return k


KEYS = {}


def _down(pc, name):
    if name not in KEYS:
        KEYS[name] = _make_key(name)
    try:
        return bool(pc.is_input_key_down(KEYS[name]))
    except Exception:
        return False


def _edge(pc, name):
    d = _down(pc, name)
    was = S["prev"].get(name, False)
    S["prev"][name] = d
    return d and not was


def _first(world, tag):
    a = unreal.GameplayStatics.get_all_actors_with_tag(world, tag)
    return a[0] if a else None


def _trace_down(world, start, end, ignore):
    try:
        r = unreal.SystemLibrary.line_trace_single(world, start, end, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                                                   False, [ignore], unreal.DrawDebugTrace.NONE, True)
    except Exception:
        return None
    hit = r[0] if isinstance(r, tuple) else r
    if hit is None:
        return None
    try:
        tup = hit.to_tuple()
        if not tup[0]:
            return None
        return tup[3]  # distance
    except Exception:
        return None


def _hud(world, text, color=(1, 1, 1), key="M4WD_HUD"):
    c = unreal.LinearColor(color[0], color[1], color[2], 1.0)
    try:
        unreal.SystemLibrary.print_string(world, text, True, False, c, 0.0, key)
    except Exception:
        unreal.SystemLibrary.print_string(world, text, True, False, c, 0.0)


def _dilate(v):
    """logical time dilation (slow motion). While recording, the engine runs slower still (see _rec_tick)."""
    if REC is not None and REC["state"] == "run":
        S["dilation"] = v
        return
    if IMU is not None and IMU["state"] == "run":
        S["dilation"] = v
        return
    if abs(S.get("dilation", 1.0) - v) > 1e-3:
        unreal.GameplayStatics.set_global_time_dilation(S["world"], v)
        S["dilation"] = v


# ---------------------------------------------------------------- lap-time optimisation trials
# Physical motor model used in the trials (Mach-Dash Motor PRO through the gear train):
#   F = F0 * max(0, duty - v / VMAX)        (PWM on a low-side switch: no brake, no reverse; coasts when F = 0)
#   F0 * VMAX = MOTOR_K  (same motor: changing the gear ratio trades top speed for pulling force)
#   a = min(F / m, MU_TIRE * g)             (4WD traction limit)
# MOTOR_K comes from the Mach-Dash calibration against the real video: 13.0 m/s^2 * 0.1259 kg * 7.0 m/s.
MOTOR_K = 13.0 * 0.1259 * 7.0           # [N m/s]
OPT = None
OPT_LAPS = 3


def start_trials(trials, results_path=None, slow=1.0, timeout=12.0, record=False):
    """trials: list of dicts {mass, vmax, <controller params>, trace(optional)}; call before Play."""
    global OPT
    base = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()), "Opt")
    os.makedirs(base, exist_ok=True)
    OPT = dict(queue=list(trials), n_total=len(trials), done=0, path=results_path or os.path.join(base, "results.csv"),
               base=base, slow=float(slow), timeout=float(timeout), record=record, cur=None, state="run", ended_at=None)
    if not os.path.exists(OPT["path"]):
        with open(OPT["path"], "w", encoding="utf-8") as fh:
            fh.write("tag,outcome,total_s,lap1,lap2,lap3,mass_kg,vmax_mps,f0_N,vpeak_mps,vlc_over_mps,zmax_cm,params\n")
    unreal.log("[M4WD] %d trials queued -> %s" % (len(trials), OPT["path"]))


def _opt_begin(world, now):
    tr = OPT["queue"].pop(0)
    m = float(tr.get("mass", 0.1259)); vmax = float(tr.get("vmax", 7.0))
    S["comp"].set_mass_override_in_kg("None", m, True)
    S["mass"] = max(0.01, S["comp"].get_mass())
    _reset(0)
    dy, dyaw = float(tr.get("dy", 0.0)), float(tr.get("dyaw", 0.0))   # start perturbation (cm, deg) for robustness checks
    if dy or dyaw:
        car = S["car"]
        loc0 = car.get_actor_location(); rot0 = car.get_actor_rotation(); rv = car.get_actor_right_vector()
        car.set_actor_location_and_rotation(loc0 + rv * dy, unreal.Rotator(roll=rot0.roll, pitch=rot0.pitch, yaw=rot0.yaw + dyaw), False, True)
    S["t_lap"] = now
    S["motor"] = 1
    if not REC:
        unreal.GameplayStatics.set_global_time_dilation(world, float(tr.get("slow", OPT["slow"])))   # per-trial time step
    OPT["cur"] = dict(tr=tr, m=m, vmax=vmax, f0=MOTOR_K / vmax, t0=now, ctrl=None, vpeak=0.0, zmax=0.0,
                      vlc=0.0, slow_t=None, trace=[] if tr.get("trace") else None, outcome=None,
                      traj=[] if tr.get("save_traj") else None)


def _opt_update(world, now, loc, vel, f, r, u, offtrack, raw_dt):
    """returns duty 0..1 for this tick"""
    if OPT["state"] != "run":
        return 0.0
    cur = OPT["cur"]
    if cur is None:
        if not OPT["queue"]:
            OPT["state"] = "done"
            unreal.log("[M4WD] all trials done")
            if not REC:
                unreal.GameplayStatics.set_global_time_dilation(world, 1.0)
                unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_request_end_play()
            return 0.0
        _opt_begin(world, now)
        return 0.0
    if cur["outcome"] is not None:                     # finished trial shown in the video: hold for a moment
        if _real_now() - cur["ended_real"] > 2.5:
            OPT["cur"] = None
            if REC and REC["state"] == "run":
                REC["state"] = "finish"
                OPT["state"] = "done"
        return 0.0
    if cur["ctrl"] is None:                            # one settle tick after the teleport
        cur["ctrl"] = CTRL.Controller(cur["vmax"], cur["tr"])
        cur["ctrl"].step(0.0, _rh_body(f, r, u), [vel[0] / 100, -vel[1] / 100, vel[2] / 100])
        return 0.0
    ctrl = cur["ctrl"]
    v_rh = [vel[0] / 100, -vel[1] / 100, vel[2] / 100]
    duty, _ = ctrl.step(raw_dt, _rh_body(f, r, u), v_rh)
    spd = math.sqrt(_dot(vel, vel)) / 100
    cur["vpeak"] = max(cur["vpeak"], spd)
    cur["zmax"] = max(cur["zmax"], loc[2])
    if ctrl.over(ctrl.n) and -5.0 < ctrl.rem <= 0.0:   # speed at the foot of the up-ramp (over crossings)
        cur["vlc"] = max(cur["vlc"], spd)
    t = now - cur["t0"]
    if cur["trace"] is not None:
        cur["trace"].append((t, duty, spd, ctrl.est.v, ctrl.est.psi, ctrl.est.pitch, int(ctrl.zone), loc[0] / 100, -loc[1] / 100, loc[2] / 100))
    if cur["traj"] is not None:
        rot = S["car"].get_actor_rotation()
        cur["traj"].append([round(t, 5), round(loc[0], 3), round(loc[1], 3), round(loc[2], 3),
                            round(rot.pitch, 3), round(rot.yaw, 3), round(rot.roll, 3),
                            round(duty, 3), round(spd, 3), round(ctrl.est.v, 3), round(ctrl.est.psi, 1),
                            len(S["laps"]), int(ctrl.zone), round(min(ctrl.v_lim, 99.0), 2)])
    c, ext = S["course"][0], S["course"][1]
    outside = abs(loc[0] - c[0]) > ext[0] + 8.0 or abs(loc[1] - c[1]) > ext[1] + 8.0
    if spd < 0.05 and t > 1.0:
        cur["slow_t"] = cur["slow_t"] or now
    else:
        cur["slow_t"] = None
    outcome = None
    if offtrack or outside:
        outcome = "courseout"
    elif len(S["laps"]) >= OPT_LAPS:
        outcome = "ok"
    elif t > OPT["timeout"]:
        outcome = "timeout"
    elif cur["slow_t"] is not None and now - cur["slow_t"] > 1.0:
        outcome = "stuck"
    if outcome:
        _opt_finish(cur, outcome)
        return 0.0
    return duty


def _rh_body(f, r, u):
    return CTRL.body_matrix((f[0], -f[1], f[2]), (-r[0], r[1], -r[2]), (u[0], -u[1], u[2]))


def _opt_finish(cur, outcome):
    cur["outcome"] = outcome
    cur["ended_real"] = _real_now()
    laps = list(S["laps"])[:OPT_LAPS]
    total = sum(laps) if (outcome == "ok" and len(laps) >= OPT_LAPS) else ""
    tr = cur["tr"]
    params = {k: v for k, v in tr.items() if k not in ("trace",)}
    row = [tr.get("tag", ""), outcome, ("%.4f" % total) if total != "" else ""] + \
          [("%.4f" % laps[i]) if i < len(laps) else "" for i in range(3)] + \
          ["%.4f" % cur["m"], "%.3f" % cur["vmax"], "%.4f" % cur["f0"], "%.3f" % cur["vpeak"], "%.3f" % cur["vlc"],
           "%.2f" % cur["zmax"], '"' + json.dumps(params).replace('"', "'") + '"']
    with open(OPT["path"], "a", encoding="utf-8") as fh:
        fh.write(",".join(str(x) for x in row) + "\n")
    if cur["trace"] is not None:
        tp = os.path.join(OPT["base"], "trace_%s.csv" % (tr.get("tag") or "trial"))
        with open(tp, "w", encoding="utf-8") as fh:
            fh.write("t_s,duty,v_true_mps,v_est_mps,psi_deg,pitch_deg,lc_zone,x_m,y_m,z_m\n")
            for rr in cur["trace"]:
                fh.write(",".join("%.5f" % x for x in rr) + "\n")
    if cur["traj"] is not None:
        with open(os.path.join(OPT["base"], "traj_%s.json" % (tr.get("tag") or "trial")), "w", encoding="utf-8") as fh:
            json.dump(dict(params=params, outcome=outcome, laps=laps, mass=cur["m"], vmax=cur["vmax"],
                           cols=["t", "x", "y", "z", "pitch", "yaw", "roll", "duty", "v", "v_est", "psi", "laps", "bridge", "v_lim"],
                           rows=cur["traj"]), fh)
    OPT["done"] += 1
    txt = "試行 %s: %s  3周 %s s  ラップ %s" % (tr.get("tag", ""), {"ok": "完走", "courseout": "コースアウト", "timeout": "時間切れ", "stuck": "停止"}[outcome],
                                     ("%.3f" % total) if total != "" else "-", " / ".join("%.3f" % x for x in laps))
    S["opt_banner"] = (txt, (0.4, 1.0, 0.4) if outcome == "ok" else (1.0, 0.2, 0.1))
    unreal.log("[M4WD] trial " + txt)
    if not REC:
        OPT["cur"] = None                              # next trial right away


# ---------------------------------------------------------------- replay of a logged trial (for the video)
# The validated run is logged pose-by-pose (2 ms steps); the video replays exactly that run, so the
# frame-capture timing cannot change the physics outcome.
REPLAY = None


def start_replay(path, rate=1.0):
    global REPLAY
    d = json.load(open(path, encoding="utf-8"))
    REPLAY = dict(data=d, t0=None, idx=0, state="run", end_real=None, rate=float(rate))
    unreal.log("[M4WD] replay armed: %s (%d samples)" % (path, len(d["rows"])))


def _replay_step(world, now):
    R = REPLAY
    d = R["data"]; rows = d["rows"]
    car, comp = S["car"], S["comp"]
    if R["t0"] is None:
        comp.set_simulate_physics(False)
        R["t0"] = now
    t = (now - R["t0"]) * R["rate"]
    while R["idx"] + 1 < len(rows) and rows[R["idx"] + 1][0] <= t:
        R["idx"] += 1
    r = rows[R["idx"]]
    car.set_actor_location_and_rotation(_v(r[1], r[2], r[3]), unreal.Rotator(roll=r[6], pitch=r[4], yaw=r[5]), False, True)
    loc = (r[1], r[2], r[3])
    f = _t(car.get_actor_forward_vector())
    _camera(loc, f, 1.0 / 60.0)
    laps = d["laps"]; done = r[11]
    t_start = sum(laps[:done])
    cur_lap = t - t_start
    finished = R["idx"] >= len(rows) - 1
    best = min(laps[:done]) if done else None
    slow_txt = ("   [スロー x%.2f]" % R["rate"]) if R["rate"] < 0.99 else ""
    lines = ["ラップタイム最適化 結果%s  重量 %.0f g  最高速度 %.2f m/s  マッハダッシュモーターPRO + IMU速度制御" % (slow_txt, d["mass"] * 1000, d["vmax"]),
             "デューティ %3.0f%%  推定速度(IMU) %.2f m/s  %s" % (r[7] * 100, r[9],
                 "立体レーンチェンジ: 速度保持 %.2f m/s" % r[13] if r[12] else ("惰走で減速中 (上限 %.2f m/s)" % r[13] if r[7] < 0.99 and r[13] < 90 else "全開")),
             "速度 %.2f m/s (%.1f km/h)" % (r[8], r[8] * 3.6),
             "ラップ %d/3   現在 %.2f s   前回 %s   合計 %.3f s" % (done, cur_lap if not finished else 0.0,
                 "%.3f s" % laps[done - 1] if done else "-", min(t, sum(laps)))]
    _hud(world, "\n".join(lines), (1.0, 0.95, 0.3))
    if finished:
        _hud(world, "3周完走  合計 %.3f s  ( %s )  コースアウトなし" % (sum(laps), " / ".join("%.3f" % x for x in laps)), (0.4, 1.0, 0.4), key="M4WD_BANNER")
        if R["end_real"] is None:
            R["end_real"] = _real_now()
        elif _real_now() - R["end_real"] > 3.0 and R["state"] == "run":
            R["state"] = "done"
            if REC and REC["state"] == "run":
                REC["state"] = "finish"


# ---------------------------------------------------------------- video recording
# Frame-exact capture: the game runs slowed down and is held while each screenshot is written,
# so the frames are exactly 1/fps apart in "viewer time" (slow motion included), whatever the PC speed.
REC = None
REC_DIR = None


def _shot_dir():
    return os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()),
                        "Screenshots", "WindowsEditor")


def _shot_files():
    try:
        return sorted(f for f in os.listdir(REC["dir"]) if f.startswith("ScreenShot") and f.endswith(".png"))
    except OSError:
        return []


def start_recording(fps=30, slow=0.2, max_seconds=90.0):
    """call from the editor (not during Play); then start Play. Frames go to Saved/Screenshots/WindowsEditor."""
    global REC
    d = _shot_dir()
    os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        if f.startswith("ScreenShot") and f.endswith(".png"):
            os.remove(os.path.join(d, f))
    REC = dict(dir=d, fps=float(fps), slow=float(slow), max=float(max_seconds), t=0.0, k=0, next=0.0,
               state="run", saw_mach=False, act=None)
    unreal.log("[M4WD] recording armed: %s" % d)


# ---------------------------------------------------------------- IMU log (state of the car body every tick)
# The game runs slowed down (IMU["slow"]) so the physics ticks are short (~4 ms of game time);
# accelerometer / gyro values are derived from this log offline (see generator/imu_from_log.py).
IMU = None


def start_imu_log(slow=0.25, phase=0, path=None):
    """call from the editor, then start Play. Logs the scenario phase `phase` (0 = standard motor) and stops Play."""
    global IMU
    if path is None:
        path = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()), "IMU", "imu_raw_log.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    IMU = dict(path=path, slow=float(slow), phase=phase, state="run", rows=[], act=None, t0=None)
    unreal.log("[M4WD] IMU log armed: %s" % path)


def _imu_tick(world, now, loc, vel, f, r, u, grounded, throttle):
    want = IMU["slow"] * S.get("dilation", 1.0)
    if IMU["act"] is None or abs(IMU["act"] - want) > 1e-6:
        unreal.GameplayStatics.set_global_time_dilation(world, want)
        IMU["act"] = want
    if S.get("phase") != IMU["phase"]:
        return
    if S.get("phase_end") is not None:              # phase finished (all laps or course-out): write and stop
        IMU["state"] = "done"
        unreal.GameplayStatics.set_global_time_dilation(world, 1.0)
        cols = ["t_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
                "fx", "fy", "fz", "lx", "ly", "lz", "ux", "uy", "uz", "grounded", "throttle", "laps"]
        with open(IMU["path"], "w", encoding="utf-8") as fh:
            fh.write("# UE world converted to right-handed (x fwd-of-world, y = -UE y, z up); body axes f=forward l=left u=up\n")
            fh.write("# motor=%s  laps=%s  result=%s\n" % (MOTORS[S["motor"]][0], ["%.3f" % x for x in S["laps"]], S["phase_end"][1]))
            fh.write(",".join(cols) + "\n")
            for row in IMU["rows"]:
                fh.write(",".join("%.6f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
        unreal.log("[M4WD] IMU log done: %d samples, laps %s -> %s" % (len(IMU["rows"]), S["laps"], IMU["path"]))
        unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_request_end_play()
        return
    if IMU["t0"] is None:
        IMU["t0"] = now
    # right-handed conversion: flip UE y
    rh = lambda v: (v[0], -v[1], v[2])
    p, v_, fw, up = rh(loc), rh(vel), rh(f), rh(u)
    lf = (-r[0], r[1], -r[2])                       # left = -right, then y flipped
    IMU["rows"].append((now - IMU["t0"], p[0] / 100, p[1] / 100, p[2] / 100, v_[0] / 100, v_[1] / 100, v_[2] / 100,
                        fw[0], fw[1], fw[2], lf[0], lf[1], lf[2], up[0], up[1], up[2],
                        1 if grounded else 0, 1 if throttle else 0, len(S["laps"])))


def _rec_set(world, v):
    if REC["act"] is None or abs(REC["act"] - v) > 1e-6:
        unreal.GameplayStatics.set_global_time_dilation(world, v)
        REC["act"] = v


def _rec_tick(world, raw_dt):
    logical = max(S.get("dilation", 1.0), 1e-3)
    REC["t"] += raw_dt / logical                       # viewer time (what a real-time viewer would see)
    ready = len(_shot_files()) >= REC["k"]             # previous screenshot written?
    if REC["state"] == "finish":
        if ready:
            REC["state"] = "done"
            _rec_set(world, 1.0)
            with open(os.path.join(REC["dir"], "m4wd_frames.txt"), "w") as fh:
                fh.write("%d %g\n" % (REC["k"], REC["fps"]))
            unreal.log("[M4WD] recording done: %d frames (%.1f s) in %s" % (REC["k"], REC["k"] / REC["fps"], REC["dir"]))
            unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_request_end_play()
        else:
            _rec_set(world, 0.001)
        return
    if not ready:
        _rec_set(world, 0.001)                         # hold the game until the frame is on disk
        return
    if REC["t"] >= REC["max"]:
        REC["state"] = "finish"
        return
    if REC["t"] >= REC["next"]:
        unreal.SystemLibrary.execute_console_command(world, "shot showui")
        REC["k"] += 1
        REC["next"] += 1.0 / REC["fps"]
        _rec_set(world, 0.001)
    else:
        _rec_set(world, REC["slow"] * logical)


# ---------------------------------------------------------------- lifecycle
def _course_info(world):
    """centre / extent of the course and the lane-change exit, from the placed course pieces."""
    lo, hi, lc_exit = [1e9, 1e9, 1e9], [-1e9, -1e9, -1e9], None
    lc_pts = []
    for a in unreal.GameplayStatics.get_all_actors_with_tag(world, "M4WD"):
        try:
            origin, ext = a.get_actor_bounds(False)
        except Exception:
            continue
        o, e = _t(origin), _t(ext)
        for i in range(3):
            lo[i] = min(lo[i], o[i] - e[i]); hi[i] = max(hi[i], o[i] + e[i])
        try:
            mesh = a.static_mesh_component.static_mesh.get_name()
        except Exception:
            mesh = ""
        if mesh == "SM_LaneChange_Base":
            lc_pts.append(o)
            if "M4WD_REV" in [str(t) for t in a.tags]:
                lc_exit = _t(a.get_actor_location())       # reversed base: its In socket is where cars leave
    c = tuple((lo[i] + hi[i]) / 2 for i in range(3))
    ext = tuple((hi[i] - lo[i]) / 2 for i in range(3))
    lc_c = tuple(sum(p[i] for p in lc_pts) / len(lc_pts) for i in range(3)) if lc_pts else None
    return c, ext, lc_c, lc_exit


def _init(world):
    S.clear()
    S["world"] = world
    S["prev"] = {}
    S["dilation"] = 1.0
    car = _first(world, TAG_CAR)
    S["car"] = car
    if car is None:
        return
    S["comp"] = car.static_mesh_component
    S["mass"] = max(0.01, S["comp"].get_mass())
    S["cam"] = _first(world, TAG_CAM)
    S["starts"] = []
    for t in TAG_START:
        a = _first(world, t)
        if a:
            S["starts"].append((a.get_actor_location(), a.get_actor_rotation()))
    fin = _first(world, TAG_FIN)
    S["fin"] = (_t(fin.get_actor_location()), _t(fin.get_actor_forward_vector()), _t(fin.get_actor_right_vector())) if fin else None
    S["pc"] = unreal.GameplayStatics.get_player_controller(world, 0)
    S["course"] = _course_info(world)
    S["scenario"] = _first(world, TAG_SCN) is not None
    S["lane"] = 0
    S["cam_mode"] = 0
    S["motor"] = 0
    S["slowmo_on"] = True
    S["vmax_seen"] = 0.0
    try:
        unreal.SystemLibrary.execute_console_command(world, "r.MotionBlurQuality 0")
    except Exception:
        pass
    if S["cam"] and S["pc"]:
        S["pc"].set_view_target_with_blend(S["cam"], 0.0)
        S["cam_pos"] = _t(car.get_actor_location())
    _reset_laps()
    if S["scenario"]:
        _scn_start_phase(0)
    unreal.log("[M4WD] runtime attached to %s (%s mode)" % (car.get_name(), "scenario" if S["scenario"] else "manual"))


def _reset_laps():
    S["laps"] = []
    S["t_lap"] = None
    S["s_prev"] = None
    S["best"] = None
    S["vmax_seen"] = 0.0
    S["offtrack_t"] = None


def _reset(lane=None):
    if lane is not None:
        S["lane"] = lane
    car, comp = S["car"], S["comp"]
    if not S["starts"]:
        return
    loc, rot = S["starts"][min(S["lane"], len(S["starts"]) - 1)]
    car.set_actor_location_and_rotation(loc, rot, False, True)
    comp.set_physics_linear_velocity(_v(0, 0, 0))
    comp.set_physics_angular_velocity_in_degrees(_v(0, 0, 0))
    _reset_laps()


# ---------------------------------------------------------------- scenario
def _now():
    return unreal.GameplayStatics.get_time_seconds(S["world"])


def _real_now():
    if REC is not None and REC["state"] in ("run", "finish"):
        return REC["t"]
    return unreal.GameplayStatics.get_real_time_seconds(S["world"])


def _scn_start_phase(i):
    if REC is not None and REC["state"] == "run":
        if i == 1:
            REC["saw_mach"] = True
        elif REC["saw_mach"]:
            REC["state"] = "finish"                    # one full cycle (standard -> Mach-Dash) recorded
    S["phase"] = i
    S["phase_t0"] = _now()
    S["phase_end"] = None          # (real time when the phase result banner started, text, colour)
    S["motor"] = 0 if i == 0 else 1
    _dilate(1.0)
    _reset(0)
    S["t_lap"] = _now()            # timer runs from the release, like the stopwatch in the video
    S["courseout_at"] = None


def _scn_update(offtrack):
    """returns throttle on/off for this frame"""
    if S["phase_end"] is not None:
        if _real_now() - S["phase_end"][0] > SCN_BANNER_TIME:
            _scn_start_phase(1 if S["phase"] == 0 else 0)
        return False
    if offtrack:
        S["courseout_at"] = _t(S["car"].get_actor_location())
        txt = "コースアウト発生！  (%s, %d周目)" % (MOTORS[S["motor"]][0], len(S["laps"]) + 1)
        col = (1.0, 0.15, 0.1)
        if S["phase"] == 0:
            txt += "  ※実機ではコースアウトなし"
        S["phase_end"] = (_real_now(), txt, col)
        unreal.log("[M4WD] scenario: course-out in phase %d at %s, laps %s" % (S["phase"], S["courseout_at"], S["laps"]))
        return False
    if S["phase"] == 0 and len(S["laps"]) >= SCN_STD_LAPS:
        S["phase_end"] = (_real_now(), "標準モーター: %d周 完走（コースアウトなし）→ 次はマッハダッシュモーターPRO" % len(S["laps"]), (0.4, 1.0, 0.4))
        unreal.log("[M4WD] scenario: standard motor laps %s" % S["laps"])
        return False
    if S["phase"] == 1 and len(S["laps"]) >= SCN_MAX_LAPS:
        S["phase_end"] = (_real_now(), "マッハダッシュ: %d周してもコースアウトせず" % len(S["laps"]), (1.0, 0.8, 0.2))
        return False
    return True


# ---------------------------------------------------------------- main step
def _step(_slate_dt):
    ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    world = ues.get_game_world()
    if world is None:
        if S:
            S.clear()
        return
    if S.get("world") is not world:
        _init(world)
    car = S.get("car")
    if car is None:
        return
    pc, comp = S["pc"], S["comp"]
    dt = unreal.GameplayStatics.get_world_delta_seconds(world)
    if dt <= 0.0:
        return
    raw_dt = dt
    dt = min(max(dt, 1.0 / 1000.0), 1.0 / 20.0)
    if REPLAY is not None and S.get("scenario"):
        if REPLAY["state"] == "run" or (REC is not None and REC["state"] in ("run", "finish")):
            _replay_step(world, unreal.GameplayStatics.get_time_seconds(world))
            if REC is not None and REC["state"] in ("run", "finish"):
                _rec_tick(world, raw_dt)
            return

    loc = _t(car.get_actor_location())
    f = _t(car.get_actor_forward_vector())
    r = _t(car.get_actor_right_vector())
    u = _t(car.get_actor_up_vector())
    vel = _t(comp.get_physics_linear_velocity())
    vf = _dot(vel, f)
    mass = S["mass"]
    now = _now()

    # ---- off-track detection (on the room floor, or lying on its side / upside down for a while)
    offtrack = loc[2] < OFFTRACK_Z
    if u[2] < 0.3:
        S["offtrack_t"] = S["offtrack_t"] or now
        offtrack = offtrack or (now - S["offtrack_t"] > 0.5)
    else:
        S["offtrack_t"] = None

    # ---- input
    if _edge(pc, "C"):
        S["cam_mode"] = (S["cam_mode"] + 1) % 4
    if _edge(pc, "T"):
        S["slowmo_on"] = not S["slowmo_on"]
        if not S["slowmo_on"]:
            _dilate(1.0)
    if S["scenario"]:
        if _edge(pc, "R"):
            _scn_start_phase(0)
        if _edge(pc, "N"):
            _scn_start_phase(1 if S["phase"] == 0 else 0)
        if OPT is not None and OPT["state"] == "run":
            throttle = _opt_update(world, now, loc, vel, f, r, u, offtrack, raw_dt)
        else:
            throttle = _scn_update(offtrack)
        brake = False
    else:
        throttle = _down(pc, "W") or _down(pc, "Up")
        brake = _down(pc, "S") or _down(pc, "Down")
        if _edge(pc, "R"):
            _reset()
        if _edge(pc, "One"):
            _reset(0)
        if _edge(pc, "Two"):
            _reset(1)
        if _edge(pc, "M"):
            S["motor"] = (S["motor"] + 1) % len(MOTORS)

    # ---- slow motion around the lane-change exit (scenario, Mach-Dash phase)
    lc_exit = S["course"][3]
    if S["scenario"] and S["slowmo_on"] and lc_exit is not None:
        near = math.dist(loc, lc_exit) < SLOWMO_RADIUS and S["phase"] == 1
        recent_out = S["phase_end"] is not None and S["courseout_at"] is not None and _real_now() - S["phase_end"][0] < 1.5
        _dilate(SLOWMO if (near or recent_out) else 1.0)

    # ---- ground contact (tyres touching something below the chassis)
    start = _v(loc[0] + u[0] * 2.5, loc[1] + u[1] * 2.5, loc[2] + u[2] * 2.5)
    end = _v(loc[0] - u[0] * 1.0, loc[1] - u[1] * 1.0, loc[2] - u[2] * 1.0)
    grounded = _trace_down(world, start, end, car) is not None

    if grounded:
        name, vmax, a0 = MOTORS[S["motor"]][:3]
        if OPT is not None and OPT["state"] == "run" and S["scenario"]:
            cur = OPT["cur"]
            F = 0.0
            if cur is not None and throttle > 0.0:
                F = cur["f0"] * max(0.0, float(throttle) - max(vf, 0.0) / 100.0 / cur["vmax"])   # N
            if F > 0.0:
                a = min(F / S["mass"] * 100.0, MU_TIRE * G)  # cm/s^2, traction limit (4WD)
            else:
                a = -ROLL_DECEL * vf                       # free-wheeling (low-side PWM switch: no brake)
        elif throttle and not brake:
            a = a0 * max(0.0, 1.0 - max(vf, 0.0) / vmax)
            a = min(a, MU_TIRE * G)                       # traction limit (4WD)
        elif brake:
            if vf > 15.0:
                a = -BRAKE
            else:
                a = -0.6 * a0 * max(0.0, 1.0 - vf / REVERSE_SPEED) if vf > REVERSE_SPEED else 0.0
        else:
            a = -ROLL_DECEL * vf
        comp.add_force(_v(f[0] * a, f[1] * a, f[2] * a), "None", True)

        # tyre side grip, per axle (lets the car yaw when the rollers push it)
        fmax = MU_TIRE * mass * G * 0.5
        for ax in (AXLE_HALF, -AXLE_HALF):
            p = _v(loc[0] + f[0] * ax + u[0] * AXLE_H, loc[1] + f[1] * ax + u[1] * AXLE_H, loc[2] + f[2] * ax + u[2] * AXLE_H)
            vp = _t(comp.get_physics_linear_velocity_at_point(p))
            vl = _dot(vp, r)
            F = -0.5 * mass * vl / dt * 0.8
            F = max(-fmax, min(fmax, F))
            comp.add_force_at_location(_v(r[0] * F, r[1] * F, r[2] * F), p, "None")

    # ---- lap timer
    if S["t_lap"] is None and throttle:
        S["t_lap"] = now
    if S["fin"] and S["t_lap"] is not None:
        fp, ff, fr = S["fin"]
        d = (loc[0] - fp[0], loc[1] - fp[1], loc[2] - fp[2])
        s, lat = _dot(d, ff), _dot(d, fr)
        if S["s_prev"] is not None and S["s_prev"] < 0.0 <= s and abs(lat) < 15.0 and _dot(vel, ff) > 0:
            lap = now - S["t_lap"]
            if lap > 0.5:              # ignore the jump across the line right after a reset
                S["laps"].append(lap)
                S["best"] = lap if S["best"] is None else min(S["best"], lap)
            S["t_lap"] = now
        S["s_prev"] = s

    _camera(loc, f, dt)
    if IMU is not None and IMU["state"] == "run" and S["scenario"]:
        _imu_tick(world, now, loc, vel, f, r, u, grounded, throttle)
    if REC is not None and REC["state"] in ("run", "finish"):
        _rec_tick(world, raw_dt)

    # ---- HUD
    spd = math.sqrt(_dot(vel, vel))
    S["vmax_seen"] = max(S["vmax_seen"], spd)
    name = MOTORS[S["motor"]][0]
    last = S["laps"][-1] if S["laps"] else None
    cur = (now - S["t_lap"]) if S["t_lap"] is not None else 0.0
    laps_txt = "ラップ %d   現在 %.2f s   前回 %s   ベスト %s" % (
        len(S["laps"]), cur, "%.2f s" % last if last else "-", "%.2f s" % S["best"] if S["best"] else "-")
    spd_txt = "速度 %.2f m/s (%.1f km/h)   最高 %.2f m/s   %s%s" % (
        spd / 100.0, spd * 0.036, S["vmax_seen"] / 100.0, "" if grounded else "（浮いている）",
        "   [スロー x%.1f]" % S["dilation"] if S["dilation"] < 0.99 else "")
    if S["scenario"] and OPT is not None and OPT["cur"] is not None and OPT["cur"]["ctrl"] is not None:
        cur = OPT["cur"]; ctrl = cur["ctrl"]
        lines = ["ラップタイム最適化  試行 %s (%d/%d)  重量 %.0f g  最高速度 %.2f m/s  マッハダッシュPRO" % (
                     cur["tr"].get("tag", ""), OPT["done"] + 1, OPT["n_total"], cur["m"] * 1000, cur["vmax"]),
                 "IMU速度制御: デューティ %3.0f%%  推定速度 %.2f m/s  方位 %.0f°  %s" % (
                     ctrl.duty * 100, ctrl.est.v, ctrl.est.psi,
                     ("レーンチェンジ区間(%s)" % ("立体の上" if ctrl.over(ctrl.n) else "下")) if ctrl.zone else ""),
                 spd_txt, laps_txt]
        _hud(world, "\n".join(lines), (1.0, 0.95, 0.3))
        if S.get("opt_banner") and cur["outcome"] is not None:
            _hud(world, S["opt_banner"][0], S["opt_banner"][1], key="M4WD_BANNER")
    elif S["scenario"]:
        lines = ["再現シナリオ  %d/2  %s  デューティ100%%（フルスロットル）" % (S["phase"] + 1, name),
                 MOTORS[S["motor"]][3] + "   [R]最初から [N]次へ [C]カメラ [T]スロー",
                 spd_txt, laps_txt]
        _hud(world, "\n".join(lines), (1.0, 0.95, 0.3))
        if S["phase_end"] is not None:
            _hud(world, S["phase_end"][1], S["phase_end"][2], key="M4WD_BANNER")
    else:
        lines = ["MAD RACER (MA)  %s   [W]加速 [S]ブレーキ/後退 [R]リセット [1/2]レーン [C]カメラ [M]モーター" % name,
                 spd_txt, laps_txt]
        _hud(world, "\n".join(lines), (1.0, 0.95, 0.3))


def _camera(loc, f, dt):
    cam = S.get("cam")
    if not cam:
        return
    c, ext, lc_c, _ = S["course"]
    mode = S["cam_mode"]
    # scenario starts with the overview that matches the test video; manual starts with the chase camera
    views = ["overview", "chase", "top", "trackside"] if S["scenario"] else ["chase", "top", "trackside", "overview"]
    view = views[mode]
    if view == "chase":
        hf = (f[0], f[1], 0.0)
        n = math.hypot(hf[0], hf[1]) or 1.0
        want = (loc[0] - hf[0] / n * CHASE_BACK, loc[1] - hf[1] / n * CHASE_BACK, loc[2] + CHASE_UP)
        k = 1.0 - math.exp(-dt * 9.0)
        cp = S.get("cam_pos", want)
        cp = tuple(cp[i] + (want[i] - cp[i]) * k for i in range(3))
        S["cam_pos"] = cp
        look = (loc[0], loc[1], loc[2] + 3.0)
    elif view == "top":
        cp, look = (c[0], c[1] + 0.1, 330.0), (c[0], c[1], 0.0)
    elif view == "overview":
        # like the video: camera stands on the lane-change side, high, looking across the oval
        if lc_c is not None:
            dx, dy = lc_c[0] - c[0], lc_c[1] - c[1]
        else:
            dx, dy = 0.0, -1.0
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        cp = (c[0] + dx * 150.0, c[1] + dy * 150.0, 215.0)
        look = (c[0] + dx * 8.0, c[1] + dy * 8.0, 0.0)
    else:
        lc = S["course"][3] or c
        cp, look = (lc[0] + 25.0, lc[1] - 45.0, 22.0), (loc[0], loc[1], loc[2] + 2.0)
    dx, dy, dz = look[0] - cp[0], look[1] - cp[1], look[2] - cp[2]
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    cam.set_actor_location_and_rotation(_v(*cp), unreal.Rotator(roll=0.0, pitch=pitch, yaw=yaw), False, True)


def _tick(dt):
    try:
        _step(dt)
    except Exception as e:  # never let an exception spam every frame
        if not S.get("err"):
            unreal.log_error("[M4WD] runtime error: %r" % e)
            import traceback
            unreal.log_error(traceback.format_exc())
        S["err"] = True


def register():
    h = getattr(builtins, "_m4wd_tick_handle", None)
    if h is not None:
        try:
            unreal.unregister_slate_post_tick_callback(h)
        except Exception:
            pass
    builtins._m4wd_tick_handle = unreal.register_slate_post_tick_callback(_tick)
    unreal.log("[M4WD] keyboard drive runtime registered - press Play")


def unregister():
    h = getattr(builtins, "_m4wd_tick_handle", None)
    if h is not None:
        unreal.unregister_slate_post_tick_callback(h)
        builtins._m4wd_tick_handle = None


if __name__ == "__main__":
    register()
