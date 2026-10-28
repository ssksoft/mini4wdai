# -*- coding: utf-8 -*-
"""Record the course-out scenario (Mini4WD_CourseOut) frame by frame.

  Output Log (Cmd):  py "<path>/ue/record_video.py"

Starts Play, captures one full cycle (standard motor 4 laps -> Mach-Dash Motor PRO course-out) at 30 fps
into MiniYonkuSim/Saved/Screenshots/WindowsEditor/ScreenShot*.png and stops Play by itself.
The game runs slowed down while recording, so it takes a few minutes; the frames are exact real-time frames.
Make the PNGs into an MP4 with, e.g.:
  ffmpeg -framerate 30 -i ScreenShot%05d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p" -c:v libx264 -crf 20 out.mp4
"""
import os, sys, importlib
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import m4wd_drive

MAP = "/Game/Mini4WD/Maps/Mini4WD_CourseOut"


def main(fps=30, slow=0.2):
    importlib.reload(m4wd_drive)
    m4wd_drive.register()
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    if world is None or not world.get_path_name().startswith(MAP):
        les.load_level(MAP)
    m4wd_drive.start_recording(fps=fps, slow=slow)
    les.editor_request_begin_play()
    unreal.log("[M4WD] recording started - wait until 'recording done' appears in the log")


if __name__ == "__main__":
    main()
