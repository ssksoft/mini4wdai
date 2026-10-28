# -*- coding: utf-8 -*-
"""Add ONE part after the selected course piece - build a course piece by piece in the editor.

  1. Select the last piece in the viewport (nothing selected = start at the world origin)
  2. Output Log (Cmd: Python):   py "<path>/ue/append_part.py" S
                                 py "<path>/ue/append_part.py" L:blue
                                 py "<path>/ue/append_part.py" LC_UP      (then LC_DOWN)
  The new piece is selected afterwards, so you can repeat the command.
  Tip: add your own Editor Utility buttons or keyboard shortcuts that run these commands.
"""
import os, sys
import unreal

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import m4wd_common as C

FOLDER = "Mini4WD_Course"


def append(tok):
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    sel = [a for a in eas.get_selected_level_actors() if C.TAG in [str(t) for t in a.tags]
           and isinstance(a, unreal.StaticMeshActor) and not a.get_actor_label().endswith("_ScanVisual")]
    # ignore a selected bridge: the course continues from the lane-change base
    sel = [a for a in sel if "LaneChange_Bridge" not in a.static_mesh_component.static_mesh.get_name()] or sel
    entry = C.actor_exit(sel[-1]) if sel else (0.0, 0.0, 0.0, 0.0)
    idx = len([a for a in eas.get_all_level_actors() if C.TAG in [str(t) for t in a.tags]])
    with unreal.ScopedEditorTransaction("Append Mini4WD part"):
        exit_, actors = C.add_piece(entry, tok, idx, FOLDER, use_scan=False)
    eas.set_selected_level_actors([actors[0]])
    unreal.log("[M4WD] added %s; next connection at (%.1f, %.1f, %.1f) yaw %.1f" % ((tok,) + tuple(exit_)))


append(sys.argv[1] if len(sys.argv) > 1 else "S")
