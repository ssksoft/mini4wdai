# -*- coding: utf-8 -*-
"""Import the Mini 4WD course parts into Unreal (run once, or again after regenerating the FBX).

  Tools > Execute Python Script... > ue/import_parts.py
  (or Output Log, Cmd: py "<path>/ue/import_parts.py")

What it does
  * imports fbx/SM_*.fbx  ->  /Game/Mini4WD/Parts      (UCX_ collision + SOCKET_ sockets included)
  * imports scan_fbx/SM_Scan_*.fbx -> /Game/Mini4WD/ScanVisuals (optional textured visuals, no collision)
  * creates materials (black / green / blue / clear cover) and a physical material
  * checks that collision and sockets arrived; repairs sockets from parts_manifest.json if needed
"""
import json, os, sys
import unreal

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE)
import m4wd_common as C

PKG = os.path.dirname(HERE)
FBX_DIR = os.path.join(PKG, "fbx")
SCAN_FBX_DIR = os.path.join(PKG, "scan_fbx")
IMPORT_SCANS = True
TRACK_FRICTION = 0.7       # tyre <-> track
TRACK_RESTITUTION = 0.05

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
mel = unreal.MaterialEditingLibrary


def _set(obj, prop, value):
    if obj is None:
        unreal.log_warning("could not set %s (object missing)" % prop)
        return
    try:
        obj.set_editor_property(prop, value)
    except Exception as e:  # property names differ slightly between engine versions
        unreal.log_warning("could not set %s.%s (%s)" % (obj.get_class().get_name(), prop, e))


def import_fbx(path, dest, with_materials):
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", path)
    task.set_editor_property("destination_path", dest)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    ui = unreal.FbxImportUI()
    _set(ui, "import_mesh", True)
    _set(ui, "import_as_skeletal", False)
    _set(ui, "mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    _set(ui, "import_animations", False)
    _set(ui, "import_materials", with_materials)
    _set(ui, "import_textures", with_materials)
    sm = ui.get_editor_property("static_mesh_import_data")
    _set(sm, "combine_meshes", True)
    _set(sm, "auto_generate_collision", False)      # use the UCX_ hulls from the file
    _set(sm, "generate_lightmap_u_vs", True)
    _set(sm, "convert_scene", True)
    _set(sm, "force_front_x_axis", False)
    _set(sm, "convert_scene_unit", False)
    _set(sm, "import_uniform_scale", 1.0)
    _set(sm, "normal_import_method", unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS)
    task.set_editor_property("options", ui)
    # Always use the classic FBX importer (keeps UCX_/USP_ collision and SOCKET_ sockets)
    cvar = "Interchange.FeatureFlags.Import.FBX"
    prev = None
    try:
        prev = unreal.SystemLibrary.get_console_variable_bool_value(cvar)
        unreal.SystemLibrary.execute_console_command(None, "%s 0" % cvar)
    except Exception:
        pass
    try:
        asset_tools.import_asset_tasks([task])
    finally:
        if prev:
            unreal.SystemLibrary.execute_console_command(None, "%s 1" % cvar)
    return list(task.get_editor_property("imported_object_paths"))


def get_or_create(name, folder, cls, factory):
    path = "%s/%s" % (folder, name)
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.EditorAssetLibrary.load_asset(path)
    return asset_tools.create_asset(name, folder, cls, factory)


def make_materials():
    base = get_or_create("M_Mini4WD_Track", C.MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    if mel.get_num_material_expressions(base) == 0:
        col = mel.create_material_expression(base, unreal.MaterialExpressionVectorParameter, -400, 0)
        col.set_editor_property("parameter_name", "Color")
        col.set_editor_property("default_value", unreal.LinearColor(0.02, 0.02, 0.02, 1.0))
        mel.connect_material_property(col, "", unreal.MaterialProperty.MP_BASE_COLOR)
        rough = mel.create_material_expression(base, unreal.MaterialExpressionScalarParameter, -400, 250)
        rough.set_editor_property("parameter_name", "Roughness")
        rough.set_editor_property("default_value", 0.35)
        mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
        mel.recompile_material(base)
    cover = get_or_create("M_Mini4WD_Cover", C.MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    if mel.get_num_material_expressions(cover) == 0:
        cover.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
        cover.set_editor_property("two_sided", True)
        c = mel.create_material_expression(cover, unreal.MaterialExpressionConstant3Vector, -400, 0)
        c.set_editor_property("constant", unreal.LinearColor(0.85, 0.92, 1.0, 1.0))
        mel.connect_material_property(c, "", unreal.MaterialProperty.MP_BASE_COLOR)
        o = mel.create_material_expression(cover, unreal.MaterialExpressionConstant, -400, 200)
        o.set_editor_property("r", 0.18)
        mel.connect_material_property(o, "", unreal.MaterialProperty.MP_OPACITY)
        mel.recompile_material(cover)
    colors = {"MI_Track_Black": (0.015, 0.015, 0.017), "MI_Track_Green": (0.25, 0.75, 0.08),
              "MI_Track_Blue": (0.03, 0.12, 0.75)}
    out = {}
    for name, rgb in colors.items():
        mi = get_or_create(name, C.MAT_DIR, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        mi.set_editor_property("parent", base)
        mel.set_material_instance_vector_parameter_value(mi, "Color", unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))
        mel.update_material_instance(mi)
        out[name] = mi
    pm = get_or_create("PM_Mini4WD_Track", C.MAT_DIR, unreal.PhysicalMaterial, unreal.PhysicalMaterialFactoryNew())
    _set(pm, "friction", TRACK_FRICTION)
    _set(pm, "restitution", TRACK_RESTITUTION)
    return out, cover, pm


def check_and_fix(mesh, part, manifest, pm):
    sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    n_convex = sub.get_convex_collision_count(mesh)
    expected = manifest["parts"][part]["collision_hulls"]
    body = mesh.get_editor_property("body_setup")
    if n_convex == 0:
        # UCX did not come through: fall back to per-polygon collision of the clean mesh (still exact)
        _set(body, "collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        unreal.log_warning("[M4WD] %s: no UCX collision imported -> using 'Complex as Simple'" % part)
    else:
        _set(body, "collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_DEFAULT)
    _set(body, "phys_material", pm)
    # sockets
    mirrored = False
    for sname, s in manifest["parts"][part]["sockets_ue"].items():
        sock = mesh.find_socket(sname)
        loc, yaw = s["location_cm"], s["yaw_deg"]
        if sock is None:
            sock = unreal.StaticMeshSocket(mesh)
            sock.set_editor_property("socket_name", sname)
            sock.set_editor_property("relative_location", unreal.Vector(loc[0], loc[1], loc[2]))
            sock.set_editor_property("relative_rotation", unreal.Rotator(roll=0.0, pitch=0.0, yaw=yaw))
            mesh.add_socket(sock)
            unreal.log_warning("[M4WD] %s: socket %s created from manifest" % (part, sname))
        else:
            l = sock.get_editor_property("relative_location")
            if abs(l.y + loc[1]) < 0.05 and abs(loc[1]) > 0.05:
                mirrored = True
    if mirrored:
        unreal.log_warning("[M4WD] %s imported mirrored in Y compared to the manifest. Sockets are taken from the mesh, "
                           "so the course still fits together - only the lane-change handedness is flipped." % part)
    unreal.log("[M4WD] %-18s convex hulls %3d / %3d  sockets OK" % (part, n_convex, expected))
    unreal.EditorAssetLibrary.save_loaded_asset(mesh)


def main():
    manifest = json.load(open(os.path.join(FBX_DIR, "parts_manifest.json"), encoding="utf-8"))
    # Use the classic FBX importer (UCX_/SOCKET_ conventions + options below) even when Interchange is enabled.
    cvar = "Interchange.FeatureFlags.Import.FBX"
    prev = None
    try:
        prev = unreal.SystemLibrary.get_console_variable_bool_value(cvar)
        unreal.SystemLibrary.execute_console_command(None, "%s 0" % cvar)
    except Exception:
        pass
    try:
        mis, cover, pm = make_materials()
        for part, info in manifest["parts"].items():
            import_fbx(os.path.join(FBX_DIR, info["file"]), C.PARTS_DIR, with_materials=False)
            mesh = C.load_mesh(part)
            mesh.set_material(0, unreal.EditorAssetLibrary.load_asset(C.MAT_DIR + "/" + C.COLORS[C.DEFAULT_COLOR[part]]))
            if part == "LaneChange_Bridge" and len(mesh.get_editor_property("static_materials")) > 1:
                mesh.set_material(1, cover)
            check_and_fix(mesh, part, manifest, pm)
        if IMPORT_SCANS and os.path.isdir(SCAN_FBX_DIR):
            for f in sorted(os.listdir(SCAN_FBX_DIR)):
                if f.lower().endswith(".fbx"):
                    import_fbx(os.path.join(SCAN_FBX_DIR, f), C.SCAN_DIR, with_materials=True)
        unreal.EditorAssetLibrary.save_directory(C.ROOT)
    finally:
        if prev is not None:
            unreal.SystemLibrary.execute_console_command(None, "%s %d" % (cvar, 1 if prev else 0))
    unreal.log("[M4WD] import finished. Next: run ue/build_course.py")


if __name__ == "__main__":
    main()
