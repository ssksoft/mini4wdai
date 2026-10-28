# -*- coding: utf-8 -*-
"""Build the course-out re-enactment level  /Game/Mini4WD/Maps/Mini4WD_CourseOut

  Output Log (Cmd):  py "<path>/ue/scenario_courseout.py"

Reproduces the real test video (duty100_madraser, standard motor vs Mach-Dash Motor PRO):
  * clockwise running (right-hand curves), curve colours alternate green / blue like the real set
  * lane-change bridge WITHOUT the clear cover (as in the video)
  * car released from standstill in the inner lane at the middle of the far straight
  * pressing Play runs: standard motor (no course-out) -> Mach-Dash Motor PRO (course-out after the lane change)
"""
import json, os, sys, importlib
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import m4wd_common as C
import import_parts as IP
import build_course as BC
import car_setup as CS
for m in (C, IP, BC, CS):
    importlib.reload(m)

MAP = C.ROOT + "/Maps/Mini4WD_CourseOut"
# order of travel: lane change, 4 right-hand curves, 2 straights, 4 right-hand curves
LAYOUT = "LC_UP,LC_DOWN,R:green,R:blue,R:green,R:blue,S,S,R:green,R:blue,R:green,R:blue"
START_PIECE = "07_S"     # junction of the two straights = middle of the far straight (release point in the video)
FOLDER = "Mini4WD_Course"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def open_empty_level():
    if unreal.EditorAssetLibrary.does_asset_exist(MAP):
        les.load_level(MAP)
        for a in eas.get_all_level_actors():
            tags = [str(t) for t in a.tags]
            if any(t.startswith("M4WD") for t in tags):
                eas.destroy_actor(a)
        les.save_current_level()
    else:
        les.new_level_from_template(MAP, "/Engine/Maps/Templates/Template_Default")


def reimport_bridge():
    """bridge without clear cover (regenerated FBX). Safe here: nothing in this level uses it yet."""
    manifest = json.load(open(os.path.join(IP.FBX_DIR, "parts_manifest.json"), encoding="utf-8"))
    mis, cover, pm = IP.make_materials()
    part = "LaneChange_Bridge"
    IP.import_fbx(os.path.join(IP.FBX_DIR, manifest["parts"][part]["file"]), C.PARTS_DIR, with_materials=False)
    mesh = C.load_mesh(part)
    mesh.set_material(0, unreal.EditorAssetLibrary.load_asset(C.MAT_DIR + "/" + C.COLORS[C.DEFAULT_COLOR[part]]))
    IP.check_and_fix(mesh, part, manifest, pm)


def main():
    open_empty_level()
    reimport_bridge()
    BC.CLEAR_EXISTING = True
    BC.build(LAYOUT, (0.0, 0.0, 0.0, 0.0), FOLDER, False)
    # start: right lane of the far straight (= inner lane when running clockwise), just past the timing line
    CS.place_car(START_PIECE, lanes=((+1, "Inner"),), start_offset=2.0, finish_offset=0.0)
    marker = eas.spawn_actor_from_class(unreal.TargetPoint, unreal.Vector(0, 0, 0), unreal.Rotator())
    CS.tagged(marker, "M4WD_SCENARIO", "M4WD_Scenario_CourseOut")
    les.save_current_level()
    CS.ensure_runtime()
    unreal.log("[M4WD] course-out scenario ready: %s - press Play" % MAP)


if __name__ == "__main__":
    main()
