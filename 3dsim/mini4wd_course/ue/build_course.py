# -*- coding: utf-8 -*-
"""Build a whole course from a list of parts, connected by their sockets.

  Output Log (Cmd: Python):   py "<path>/ue/build_course.py"
                              py "<path>/ue/build_course.py" "S,S,L,L,L,L,S,S,L,L,L,L"

Tokens
  S        straight (46.5 cm)
  L / R    45 degree curve turning left / right     (append :green or :blue to choose the colour, e.g. L:blue)
  LC_UP    lane-change base, ramp up (the bridge is added automatically)
  LC_DOWN  lane-change base placed backwards (ramp down from the bridge). Always directly after LC_UP.
"""
import os, sys
import unreal

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import m4wd_common as C

# Tamiya Oval Home Circuit (lane-change type): 216 x 123 cm, about 10 m for two laps
LAYOUT = "LC_UP,LC_DOWN,L:green,L:green,L:green,L:green,S,S,L:blue,L:blue,L:blue,L:blue"
ORIGIN = (0.0, 0.0, 0.0, 0.0)          # x, y, z (cm), yaw (deg) of the first connection
FOLDER = "Mini4WD_Course"
USE_SCAN_VISUALS = False               # True: show the textured 3D-scan meshes, keep clean collision hidden
CLEAR_EXISTING = True                  # delete previously generated pieces in FOLDER


def clear(folder):
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    for a in eas.get_all_level_actors():
        if C.TAG in [str(t) for t in a.tags] and str(a.get_folder_path()).startswith(folder):
            eas.destroy_actor(a)


def build(layout=LAYOUT, origin=ORIGIN, folder=FOLDER, use_scan=USE_SCAN_VISUALS):
    tokens = [t for t in layout.replace(" ", "").split(",") if t]
    with unreal.ScopedEditorTransaction("Build Mini4WD course"):
        if CLEAR_EXISTING:
            clear(folder)
        T = origin
        for i, tok in enumerate(tokens):
            T, _ = C.add_piece(T, tok, i, folder, use_scan)
    gap = ((T[0] - origin[0]) ** 2 + (T[1] - origin[1]) ** 2 + (T[2] - origin[2]) ** 2) ** 0.5
    dyaw = (T[3] - origin[3] + 180.0) % 360.0 - 180.0
    if gap < 0.05 and abs(dyaw) < 0.05:
        unreal.log("[M4WD] %d pieces placed - closed loop (gap %.3f cm)" % (len(tokens), gap))
    else:
        unreal.log_warning("[M4WD] %d pieces placed - open course: end is %.2f cm / %.2f deg away from the start"
                           % (len(tokens), gap, dyaw))


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else LAYOUT)
