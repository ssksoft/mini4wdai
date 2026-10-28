# Runs automatically when the Unreal Editor opens this project.
# Registers the Mini 4WD keyboard-drive runtime from ../mini4wd_course/ue (path relative to the project).
import os
import sys
import unreal

_proj = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
_ue_dir = os.path.normpath(os.path.join(_proj, "..", "mini4wd_course", "ue"))
if os.path.isdir(_ue_dir):
    if _ue_dir not in sys.path:
        sys.path.append(_ue_dir)
    import m4wd_drive
    m4wd_drive.register()
else:
    unreal.log_warning("[M4WD] mini4wd_course/ue not found next to the project: %s" % _ue_dir)
