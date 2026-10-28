# -*- coding: utf-8 -*-
"""Run a batch of lap-time trials in the course-out level (3 laps, standing start, inner lane, clockwise).

  Output Log (Cmd):  py "<path>/ue/run_opt.py" ["<path>/opt/<batch>.json"]

batch json: {"name": "b01", "slow": 1.0, "timeout": 12, "record": false,
             "trials": [{"tag": "a1", "mass": 0.09, "vmax": 7.0, "v_over": 3.0, ...}, ...]}
Results are appended to MiniYonkuSim/Saved/Opt/<name>.csv; Play stops by itself when the batch is done.
With "record": true the run is captured frame by frame (see record_video.py) for the final video;
{"replay": "traj_<tag>.json", "record": true} replays a logged trial (save_traj) instead of simulating again.
"""
import os, sys, json, importlib
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import m4wd_ctrl, m4wd_drive

MAP = "/Game/Mini4WD/Maps/Mini4WD_CourseOut"


def main(path=None):
    path = path or os.path.join(os.path.dirname(HERE), "opt", "queue.json")
    batch = json.load(open(path, encoding="utf-8"))
    importlib.reload(m4wd_ctrl)
    importlib.reload(m4wd_drive)
    m4wd_drive.register()
    les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    if world is None or not world.get_path_name().startswith(MAP):
        les.load_level(MAP)
    base = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()), "Opt")
    if batch.get("replay"):                      # video of a logged trial: {"replay": "traj_<tag>.json", "record": true}
        rp = batch["replay"] if os.path.isabs(batch["replay"]) else os.path.join(base, batch["replay"])
        m4wd_drive.start_replay(rp, batch.get("rate", 1.0))
    else:
        m4wd_drive.start_trials(batch["trials"], os.path.join(base, batch.get("name", "results") + ".csv"),
                                slow=batch.get("slow", 1.0), timeout=batch.get("timeout", 12.0), record=batch.get("record", False))
    if batch.get("record"):
        m4wd_drive.start_recording(fps=batch.get("fps", 30), slow=batch.get("rec_slow", 0.2), max_seconds=batch.get("max_seconds", 30))
    les.editor_request_begin_play()
    unreal.log("[M4WD] batch %s started (%d trials)" % (batch.get("name"), len(batch.get("trials", []))))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
