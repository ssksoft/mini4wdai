# -*- coding: utf-8 -*-
"""Mini 4WD runtime (keyboard driving) - runs while you Play In Editor (PIE).

The car body, rollers and tyres collide with the course through Chaos physics
(all frictionless, so the walls guide the car like the real rollers do).
This script adds, every frame:
  * motor drive force  (linear torque-speed curve of the motor, 3.5:1 gear, ~24 mm tyres)
  * tyre side grip     (per axle, limited by friction mu * load)
  * keyboard input, chase camera, HUD (speed / lap times)

Keys:  W / Up = throttle    S / Down = brake, then reverse
       R = back to the start  1 / 2 = start lane (outer / inner) + reset
       C = camera (chase / top / trackside)    M = motor (normal / tuned / high speed)
"""
import math
import builtins
import unreal

# ---------------------------------------------------------------- parameters (UE units: cm, s, kg)
MOTORS = [  # name, no-load car speed (cm/s), stall acceleration (cm/s^2)
    ("ノーマルモーター", 500.0, 900.0),
    ("トルクチューン相当", 620.0, 1050.0),
    ("ハイスピード相当", 760.0, 1100.0),
]
MU_TIRE = 1.0            # tyre / track friction
G = 980.0
ROLL_DECEL = 0.25        # coasting drag  (1/s)
BRAKE = 1200.0           # cm/s^2
REVERSE_SPEED = -80.0
AXLE_HALF = 4.0          # wheelbase 80 mm / 2
AXLE_H = 1.2             # tyre radius
CHASE_BACK, CHASE_UP = 32.0, 13.0

TAG_CAR, TAG_CAM, TAG_FIN = "M4WD_CAR", "M4WD_CAM", "M4WD_FINISH"
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
    for f in (lambda: unreal.Key(key_name=name),):
        try:
            return f()
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


KEYS = {n: None for n in ("W", "Up", "S", "Down", "R", "One", "Two", "C", "M")}


def _down(pc, name):
    if KEYS[name] is None:
        KEYS[name] = _make_key(name)
    try:
        return bool(pc.is_input_key_down(KEYS[name]))
    except Exception:
        return False


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


# ---------------------------------------------------------------- lifecycle
def _init(world):
    S.clear()
    S["world"] = world
    car = _first(world, TAG_CAR)
    if car is None:
        S["car"] = None
        return
    S["car"] = car
    S["comp"] = car.get_component_by_class(unreal.PrimitiveComponent) if not hasattr(car, "static_mesh_component") else car.static_mesh_component
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
    S["lane"] = 0
    S["cam_mode"] = 0
    S["motor"] = 0
    S["prev"] = {}
    S["laps"] = []
    S["t_lap"] = None                 # lap clock starts with the first throttle press
    S["s_prev"] = None
    S["best"] = None
    try:
        unreal.SystemLibrary.execute_console_command(world, "r.MotionBlurQuality 0")
    except Exception:
        pass
    S["vmax_seen"] = 0.0
    if S["cam"] and S["pc"]:
        S["pc"].set_view_target_with_blend(S["cam"], 0.0)
        loc = car.get_actor_location()
        S["cam_pos"] = _t(loc)
    unreal.log("[M4WD] drive runtime attached to %s" % car.get_name())


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
    S["t_lap"] = None
    S["s_prev"] = None


def _edge(pc, name):
    d = _down(pc, name)
    was = S["prev"].get(name, False)
    S["prev"][name] = d
    return d and not was


def _step(dt):
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
    dt = min(max(dt, 1.0 / 240.0), 1.0 / 20.0)

    # ---- input
    throttle = _down(pc, "W") or _down(pc, "Up")
    brake = _down(pc, "S") or _down(pc, "Down")
    if _edge(pc, "R"):
        _reset()
    if _edge(pc, "One"):
        _reset(0)
    if _edge(pc, "Two"):
        _reset(1)
    if _edge(pc, "C"):
        S["cam_mode"] = (S["cam_mode"] + 1) % 3
    if _edge(pc, "M"):
        S["motor"] = (S["motor"] + 1) % len(MOTORS)

    loc = _t(car.get_actor_location())
    f = _t(car.get_actor_forward_vector())
    r = _t(car.get_actor_right_vector())
    u = _t(car.get_actor_up_vector())
    vel = _t(comp.get_physics_linear_velocity())
    vf = _dot(vel, f)
    mass = S["mass"]

    # ---- ground contact (tyres touching something below the chassis)
    start = _v(loc[0] + u[0] * 2.5, loc[1] + u[1] * 2.5, loc[2] + u[2] * 2.5)
    end = _v(loc[0] - u[0] * 1.0, loc[1] - u[1] * 1.0, loc[2] - u[2] * 1.0)
    grounded = _trace_down(world, start, end, car) is not None

    if grounded:
        name, vmax, a0 = MOTORS[S["motor"]]
        if throttle and not brake:
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
    now = unreal.GameplayStatics.get_time_seconds(world)
    if S["t_lap"] is None and throttle:
        S["t_lap"] = now
    if S["fin"] and S["t_lap"] is not None:
        fp, ff, fr = S["fin"]
        d = (loc[0] - fp[0], loc[1] - fp[1], loc[2] - fp[2])
        s, lat = _dot(d, ff), _dot(d, fr)
        if S["s_prev"] is not None and S["s_prev"] < 0.0 <= s and abs(lat) < 15.0 and _dot(vel, ff) > 0:
            lap = now - S["t_lap"]
            S["laps"].append(lap)
            S["best"] = lap if S["best"] is None else min(S["best"], lap)
            S["t_lap"] = now
        S["s_prev"] = s

    # ---- camera
    cam = S.get("cam")
    if cam:
        mode = S["cam_mode"]
        if mode == 0:
            hf = (f[0], f[1], 0.0)
            n = math.hypot(hf[0], hf[1]) or 1.0
            want = (loc[0] - hf[0] / n * CHASE_BACK, loc[1] - hf[1] / n * CHASE_BACK, loc[2] + CHASE_UP)
            k = 1.0 - math.exp(-dt * 9.0)
            cp = S.get("cam_pos", want)
            cp = tuple(cp[i] + (want[i] - cp[i]) * k for i in range(3))
            S["cam_pos"] = cp
            look = (loc[0], loc[1], loc[2] + 3.0)
        elif mode == 1:
            cp, look = (46.5, -49.7, 330.0), (46.5, -49.6, 0.0)
        else:
            cp, look = (40.0, 40.0, 25.0), (loc[0], loc[1], loc[2] + 2.0)
        dx, dy, dz = look[0] - cp[0], look[1] - cp[1], look[2] - cp[2]
        yaw = math.degrees(math.atan2(dy, dx))
        pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        cam.set_actor_location_and_rotation(_v(*cp), unreal.Rotator(roll=0.0, pitch=pitch, yaw=yaw), False, True)

    # ---- HUD
    spd = math.sqrt(_dot(vel, vel))
    S["vmax_seen"] = max(S["vmax_seen"], spd)
    name = MOTORS[S["motor"]][0]
    last = S["laps"][-1] if S["laps"] else None
    lines = [
        "MAD RACER (MA)  %s   [W]加速 [S]ブレーキ/後退 [R]リセット [1/2]レーン [C]カメラ [M]モーター" % name,
        "速度 %.2f m/s (%.1f km/h)   最高 %.2f m/s   %s" % (spd / 100.0, spd * 0.036, S["vmax_seen"] / 100.0, "" if grounded else "（浮いている）"),
        "ラップ %d   現在 %.2f s   前回 %s   ベスト %s" % (len(S["laps"]), (now - S["t_lap"]) if S["t_lap"] is not None else 0.0,
                                                     "%.2f s" % last if last else "-", "%.2f s" % S["best"] if S["best"] else "-"),
    ]
    _hud(world, "\n".join(lines), (1.0, 0.95, 0.3))


def _tick(dt):
    try:
        _step(dt)
    except Exception as e:  # never let an exception spam every frame
        if not S.get("err"):
            unreal.log_error("[M4WD] runtime error: %r" % e)
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
