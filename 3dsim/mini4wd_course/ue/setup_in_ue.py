# -*- coding: utf-8 -*-
"""One-shot setup: new level -> import parts -> build the official oval -> camera -> save.

  Output Log (Cmd: Python):  py "<path>/ue/setup_in_ue.py"
"""
import os, sys, importlib
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import m4wd_common as C
import import_parts, build_course
importlib.reload(C); importlib.reload(import_parts); importlib.reload(build_course)

MAP = C.ROOT + "/Maps/Mini4WD_Oval"

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
# 1) open an empty level first, so no placed actor uses a mesh while it is being (re)imported
if unreal.EditorAssetLibrary.does_asset_exist(MAP):
    les.load_level(MAP)
else:
    les.new_level_from_template(MAP, "/Engine/Maps/Templates/Template_Default")
# 2) import parts, materials, collision, sockets
import_parts.main()
# 3) build the official layout (216 x 123 cm oval with lane change)
build_course.build(build_course.LAYOUT)
# 4) look at it
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
ues.set_level_viewport_camera_info(unreal.Vector(-110.0, 60.0, 140.0), unreal.Rotator(roll=0.0, pitch=-36.0, yaw=-35.0))
les.save_current_level()
unreal.log("[M4WD] setup done: %s" % MAP)
