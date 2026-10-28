# -*- coding: utf-8 -*-
"""Log the car body motion for the IMU simulation (standard motor, 4 laps, duty 100 %).

  Output Log (Cmd):  py "<path>/ue/record_imu.py"

Opens Mini4WD_CourseOut, starts Play, logs the standard-motor phase and stops Play by itself.
Output: MiniYonkuSim/Saved/IMU/imu_raw_log.csv  -> accelerometer / gyro values: generator/imu_from_log.py
"""
import os, sys, importlib
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import m4wd_drive

MAP = "/Game/Mini4WD/Maps/Mini4WD_CourseOut"


def main(slow=0.1):        # ~1.7 ms game-time ticks at 60 fps -> enough for a 200 Hz IMU
    importlib.reload(m4wd_drive)
    m4wd_drive.register()
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    if world is None or not world.get_path_name().startswith(MAP):
        les.load_level(MAP)
    m4wd_drive.start_imu_log(slow=slow, phase=0)
    les.editor_request_begin_play()
    unreal.log("[M4WD] IMU logging started - wait until 'IMU log done' appears in the log")


if __name__ == "__main__":
    main()
