# -*- coding: utf-8 -*-
"""Put the Mad Racer on the course and make it drivable with the keyboard.

  Output Log (Cmd):  py "<path>/ue/car_setup.py"
Needs the course level from setup_in_ue.py (/Game/Mini4WD/Maps/Mini4WD_Oval).
"""
import json, os, sys
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)
import import_parts as IP
import m4wd_common as C

PKG = os.path.dirname(HERE)
CAR_DIR = C.ROOT + "/Car"
MAP = C.ROOT + "/Maps/Mini4WD_Oval"
SPEC = json.load(open(os.path.join(PKG, "car", "car_spec.json"), encoding="utf-8"))
START_PIECE = "06_S"          # first straight after the lane-change side
LANE_OFFSET = 5.85            # cm, lane centre from the middle wall
FLOOR_TOP = 0.7               # cm

at = unreal.AssetToolsHelpers.get_asset_tools()
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)


def _try(obj, names, value):
    for n in names:
        try:
            obj.set_editor_property(n, value)
            return True
        except Exception:
            pass
    unreal.log_warning("[M4WD] could not set %s" % names[0])
    return False


def physical_material():
    pm = IP.get_or_create("PM_M4WD_Car", CAR_DIR, unreal.PhysicalMaterial, unreal.PhysicalMaterialFactoryNew())
    _try(pm, ["friction"], 0.0)
    _try(pm, ["static_friction"], 0.0)
    _try(pm, ["restitution"], 0.0)
    _try(pm, ["override_friction_combine_mode"], True)
    for enum_name in ("FrictionCombineMode", "EFrictionCombineMode"):
        e = getattr(unreal, enum_name, None)
        if e is not None and hasattr(e, "MIN"):
            _try(pm, ["friction_combine_mode"], e.MIN)
            break
    _try(pm, ["override_restitution_combine_mode"], True)
    unreal.EditorAssetLibrary.save_loaded_asset(pm)
    return pm


def import_car(pm):
    IP.import_fbx(os.path.join(PKG, "car", SPEC["fbx"]), CAR_DIR, with_materials=True)
    mesh = unreal.EditorAssetLibrary.load_asset(CAR_DIR + "/SM_M4WD_MadRacer")
    sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    n = sub.get_simple_collision_count(mesh) + sub.get_convex_collision_count(mesh)
    body = mesh.get_editor_property("body_setup")
    IP._set(body, "phys_material", pm)
    unreal.EditorAssetLibrary.save_loaded_asset(mesh)
    unreal.log("[M4WD] car mesh imported, simple collision shapes: %d (expected 11)" % n)
    return mesh


def game_mode():
    path = CAR_DIR + "/BP_M4WD_GameMode"
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        bp = unreal.EditorAssetLibrary.load_asset(path)
    else:
        f = unreal.BlueprintFactory()
        f.set_editor_property("parent_class", unreal.GameModeBase)
        bp = at.create_asset("BP_M4WD_GameMode", CAR_DIR, None, f)
    cls = unreal.EditorAssetLibrary.load_blueprint_class(path)
    cdo = unreal.get_default_object(cls)
    cdo.set_editor_property("default_pawn_class", None)   # no flying spectator: keys go to the car
    unreal.BlueprintEditorLibrary.compile_blueprint(bp)
    unreal.EditorAssetLibrary.save_loaded_asset(bp)
    return cls


def find_label(label):
    for a in eas.get_all_level_actors():
        if a.get_actor_label() == label:
            return a
    return None


def clear_old():
    for a in eas.get_all_level_actors():
        tags = [str(t) for t in a.tags]
        if any(t.startswith("M4WD_") and t not in (C.TAG, C.TAG_REV) for t in tags):
            eas.destroy_actor(a)


def tagged(actor, tag, label, folder="Mini4WD_Car"):
    actor.set_editor_property("tags", [unreal.Name(tag)])
    actor.set_actor_label(label)
    actor.set_folder_path(folder)
    return actor


def main():
    if unreal.EditorAssetLibrary.does_asset_exist(MAP):
        les.load_level(MAP)
    clear_old()                      # remove the old car first: never re-import a mesh that is placed in the level
    pm = physical_material()
    mesh = import_car(pm)
    gm = game_mode()

    piece = find_label(START_PIECE)
    if piece is None:
        raise RuntimeError("course piece %s not found - run setup_in_ue.py first" % START_PIECE)
    p0 = piece.get_actor_location(); rot = piece.get_actor_rotation()
    fwd = piece.get_actor_forward_vector(); right = piece.get_actor_right_vector()
    starts = []
    for i, side in enumerate((+1, -1)):   # 0 = right lane (outer on this oval), 1 = left lane (inner)
        p = p0 + fwd * 8.0 + right * (side * LANE_OFFSET)
        p.z = FLOOR_TOP + 0.3
        tp = eas.spawn_actor_from_class(unreal.TargetPoint, p, rot)
        tagged(tp, "M4WD_START_%d" % i, "M4WD_Start_%s" % ("Outer" if i == 0 else "Inner"))
        starts.append(p)
    fin = eas.spawn_actor_from_class(unreal.TargetPoint, unreal.Vector(p0.x, p0.y, FLOOR_TOP), rot)
    tagged(fin, "M4WD_FINISH", "M4WD_FinishLine")

    car = eas.spawn_actor_from_object(mesh, starts[0], rot)
    tagged(car, "M4WD_CAR", "MadRacer")
    comp = car.static_mesh_component
    comp.set_mobility(unreal.ComponentMobility.MOVABLE)
    comp.set_collision_profile_name("PhysicsActor")
    comp.set_simulate_physics(True)
    comp.set_mass_override_in_kg("None", SPEC["mass_kg"], True)
    bi = comp.get_editor_property("body_instance")
    _try(bi, ["use_ccd", "b_use_ccd"], True)
    _try(bi, ["linear_damping"], 0.0)
    _try(bi, ["angular_damping"], 0.3)
    _try(bi, ["com_nudge"], unreal.Vector(0.0, 0.0, -0.8))   # batteries sit low in the MA chassis
    _try(bi, ["phys_material_override"], pm)
    comp.set_editor_property("body_instance", bi)

    cam = eas.spawn_actor_from_class(unreal.CameraActor, starts[0] + unreal.Vector(-30, 0, 15), rot)
    tagged(cam, "M4WD_CAM", "M4WD_ChaseCamera")
    try:
        cc = cam.camera_component
        cc.set_editor_property("field_of_view", 70.0)
        cc.set_editor_property("constrain_aspect_ratio", False)
    except Exception:
        pass

    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    ws = world.get_world_settings()
    _try(ws, ["default_game_mode"], gm)
    les.save_current_level()

    # auto-start the keyboard runtime with every editor session of this project
    # (init_unreal.py finds ../mini4wd_course/ue relative to the project, so the repo can live anywhere)
    pydir = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir()), "Python")
    init_py = os.path.join(pydir, "init_unreal.py")
    if not os.path.exists(init_py):
        os.makedirs(pydir, exist_ok=True)
        with open(init_py, "w", encoding="utf-8") as fh:
            fh.write("import sys\nsys.path.append(r'%s')\nimport m4wd_drive\nm4wd_drive.register()\n" % HERE)
    import importlib, m4wd_drive
    importlib.reload(m4wd_drive)
    m4wd_drive.register()
    unreal.log("[M4WD] car ready - press Play (Alt+P), then W to drive")


if __name__ == "__main__":
    main()
