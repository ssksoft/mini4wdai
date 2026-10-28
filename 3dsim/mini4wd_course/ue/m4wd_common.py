# -*- coding: utf-8 -*-
"""Shared helpers for the Mini 4WD course tools (Unreal Editor Python).

Coordinate rules: every part has sockets "In" and "Out" (X = direction of travel).
LaneChange_Base also has "BridgeMount", where LaneChange_Bridge is attached.
"""
import math
import unreal

ROOT = "/Game/Mini4WD"
PARTS_DIR = ROOT + "/Parts"
SCAN_DIR = ROOT + "/ScanVisuals"
MAT_DIR = ROOT + "/Materials"
TAG = "M4WD"
TAG_REV = "M4WD_REV"

# token -> (part name, reversed?)
TOKENS = {
    "S": ("Straight", False),
    "L": ("Curve45_L", False),
    "R": ("Curve45_R", False),
    "LC_UP": ("LaneChange_Base", False),     # ramp up + bridge (bridge is spawned automatically)
    "LC_DOWN": ("LaneChange_Base", True),    # same part, placed backwards: comes down from the bridge
}

COLORS = {  # material instances created by import_parts.py
    "black": "MI_Track_Black", "green": "MI_Track_Green", "blue": "MI_Track_Blue",
}
DEFAULT_COLOR = {"Straight": "black", "Curve45_L": "green", "Curve45_R": "blue",
                 "LaneChange_Base": "black", "LaneChange_Bridge": "black"}


def parse_token(tok):
    """'L', 'L:blue', 'S', 'LC_UP' ... -> (token, color or None)"""
    if ":" in tok:
        t, c = tok.split(":", 1)
        return t.strip().upper(), c.strip().lower()
    return tok.strip().upper(), None


def load_mesh(part):
    m = unreal.EditorAssetLibrary.load_asset("%s/SM_%s" % (PARTS_DIR, part))
    if m is None:
        raise RuntimeError("SM_%s not found in %s. Run import_parts.py first." % (part, PARTS_DIR))
    return m


def socket_xf(mesh, name):
    s = mesh.find_socket(name)
    if s is None:
        raise RuntimeError("socket '%s' missing on %s" % (name, mesh.get_name()))
    loc = s.get_editor_property("relative_location")
    rot = s.get_editor_property("relative_rotation")
    return (loc.x, loc.y, loc.z, rot.yaw)


# ---- planar transforms (x, y, z, yaw_deg); course pieces always lie flat
def compose(T, S):
    x, y, z, yaw = T
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return (x + c * S[0] - s * S[1], y + s * S[0] + c * S[1], z + S[2], yaw + S[3])


def inverse(S):
    c, s = math.cos(math.radians(-S[3])), math.sin(math.radians(-S[3]))
    return (-(c * S[0] - s * S[1]), -(s * S[0] + c * S[1]), -S[2], -S[3])


FLIP = (0.0, 0.0, 0.0, 180.0)


def place(entry, mesh, reverse):
    """entry: world transform of the connection we attach to (X pointing along travel).
    Returns (actor world transform, exit transform)."""
    IN, OUT = socket_xf(mesh, "In"), socket_xf(mesh, "Out")
    if not reverse:
        W = compose(entry, inverse(IN))
        return W, compose(W, OUT)
    W = compose(compose(entry, FLIP), inverse(OUT))
    return W, compose(compose(W, IN), FLIP)


def actor_exit(actor):
    """Exit connection of an already placed course actor (respects the reversed tag)."""
    mesh = actor.static_mesh_component.static_mesh
    loc = actor.get_actor_location(); rot = actor.get_actor_rotation()
    W = (loc.x, loc.y, loc.z, rot.yaw)
    if TAG_REV in [str(t) for t in actor.tags]:
        return compose(compose(W, socket_xf(mesh, "In")), FLIP)
    return compose(W, socket_xf(mesh, "Out"))


def material(color):
    name = COLORS.get(color, color)
    return unreal.EditorAssetLibrary.load_asset("%s/%s" % (MAT_DIR, name))


def spawn(part, W, label, folder, reverse=False, color=None, scan_visual=None):
    eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    mesh = load_mesh(part)
    actor = eas.spawn_actor_from_object(mesh, unreal.Vector(W[0], W[1], W[2]), unreal.Rotator(roll=0.0, pitch=0.0, yaw=W[3]))
    actor.set_actor_label(label)
    actor.set_folder_path(folder)
    tags = [unreal.Name(TAG)] + ([unreal.Name(TAG_REV)] if reverse else [])
    actor.set_editor_property("tags", tags)
    comp = actor.static_mesh_component
    comp.set_mobility(unreal.ComponentMobility.STATIC)
    mi = material(color or DEFAULT_COLOR.get(part, "black"))
    if mi:
        comp.set_material(0, mi)
    if scan_visual:
        vis = unreal.EditorAssetLibrary.load_asset("%s/SM_Scan_%s" % (SCAN_DIR, scan_visual))
        if vis:
            # keep the clean part for collision only, show the textured scan instead
            comp.set_visibility(False)
            actor.set_actor_hidden_in_game(True)
            va = eas.spawn_actor_from_object(vis, unreal.Vector(W[0], W[1], W[2]), unreal.Rotator(roll=0.0, pitch=0.0, yaw=W[3]))
            va.set_actor_label(label + "_ScanVisual")
            va.set_folder_path(folder)
            va.set_editor_property("tags", [unreal.Name(TAG)])
            va.static_mesh_component.set_collision_profile_name("NoCollision")
            va.static_mesh_component.set_mobility(unreal.ComponentMobility.STATIC)
            va.attach_to_actor(actor, "", unreal.AttachmentRule.KEEP_WORLD, unreal.AttachmentRule.KEEP_WORLD,
                               unreal.AttachmentRule.KEEP_WORLD, False)
        else:
            unreal.log_warning("scan visual SM_Scan_%s not found - showing the clean part" % scan_visual)
    return actor


def scan_name(part, reverse, color):
    if part == "Straight": return "Straight"
    if part == "LaneChange_Bridge": return "LaneChange_Bridge"
    if part == "LaneChange_Base": return "LaneChange_Base_B" if reverse else "LaneChange_Base_A"
    c = "Blue" if (color or DEFAULT_COLOR[part]) == "blue" else "Green"
    return ("Curve45R_" if part == "Curve45_R" else "Curve45_") + c


def add_piece(entry, tok, index, folder, use_scan=False):
    """Spawn one piece (plus the bridge for LC_UP). Returns (exit transform, [actors])."""
    t, color = parse_token(tok)
    if t not in TOKENS:
        raise ValueError("unknown token '%s' (use %s)" % (tok, ", ".join(TOKENS)))
    part, rev = TOKENS[t]
    mesh = load_mesh(part)
    W, exit_ = place(entry, mesh, rev)
    actors = [spawn(part, W, "%02d_%s" % (index, t), folder, rev, color,
                    scan_name(part, rev, color) if use_scan else None)]
    if t == "LC_UP":
        bmesh = load_mesh("LaneChange_Bridge")
        Wb = compose(compose(W, socket_xf(mesh, "BridgeMount")), inverse(socket_xf(bmesh, "In")))
        actors.append(spawn("LaneChange_Bridge", Wb, "%02d_Bridge" % index, folder, False, color,
                            "LaneChange_Bridge" if use_scan else None))
    return exit_, actors
