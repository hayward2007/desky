"""Interactive 3D FK/IK preview of the desky arm — no hardware required.

Renders the arm from the URDF <visual> geometry and lets you drive it two
ways, reading the same geometry source as the real robot
(kinematics/configure/desky.urdf):

  * FK: drag one slider per joint (servo degree, 30..330) — the arm redraws
    live and the end-effector position is shown in the title.
  * IK: type a target x/y/z and press "Solve IK" — kinematics.Arm.ik solves the
    joint angles, the sliders jump to the solution, and the pose redraws.

Rendering notes:
  * <box>/<cylinder> visuals render as-is. <mesh> visuals (the real STL links)
    render as a bounding-box APPROXIMATION, not the actual mesh — this module
    is the cheap, always-on preview used by src/app.py's live web dashboard
    and hand-tracking overlay, redrawn every frame, so it trades geometric
    fidelity for speed (the alternative — a few hundred thousand real STL
    triangles reprojected every redraw in pure Python — is far too slow for a
    live loop). For the real mesh geometry, use kinematics.pybullet_sim
    (`python -m kinematics.pybullet_sim`) instead.
  * Every box face has its own colour AND a number (1..6): 1 = +Z top,
    2 = -Z bottom, 3 = +X, 4 = -X, 5 = +Y, 6 = -Y (each link's own frame).
  * Each revolute joint's rotation axis is drawn as a yellow arrow.
  * All link faces go into ONE Poly3DCollection so matplotlib depth-sorts them
    per-face; a separate collection per link would sort whole links against each
    other (painter's algorithm) and make overlapping links pop incorrectly.

Requires matplotlib (not a dependency of the kinematics package itself):
    pip install matplotlib
"""

import math
import os
import struct
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.widgets import Slider, Button, TextBox
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .kinematics import _matmul, _translate, _rpy, _rot_axis
from .urdf_loader import load_arm, _DEFAULT_URDF_PATH
from fundamental.const import SimulateConst
from fundamental.logger import Logger

# Box faces are always emitted in this order; index i -> label (i+1).
FACE_NAMES = SimulateConst.FACE_NAMES
FACE_COLORS = SimulateConst.FACE_COLORS
CYLINDER_COLOR = SimulateConst.CYLINDER_COLOR


# ---------------------------------------------------------------------------
# URDF visual geometry
# ---------------------------------------------------------------------------
def _triplet(text, default=(0.0, 0.0, 0.0)):
    return tuple(float(v) for v in text.split()) if text else default


_stl_bbox_cache = {}


def _stl_local_bbox(path):
    """Return (size_xyz, center_xyz) of a binary STL file's own local
    vertex data (whatever units it was exported in — mm for this project).
    Cached by path since several links reuse the same mesh file."""
    if path in _stl_bbox_cache:
        return _stl_bbox_cache[path]
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    with open(path, "rb") as f:
        f.read(80)  # header
        (n,) = struct.unpack("<I", f.read(4))
        for _ in range(n):
            f.read(12)  # facet normal
            for _ in range(3):
                x, y, z = struct.unpack("<fff", f.read(12))
                lo[0], lo[1], lo[2] = min(lo[0], x), min(lo[1], y), min(lo[2], z)
                hi[0], hi[1], hi[2] = max(hi[0], x), max(hi[1], y), max(hi[2], z)
            f.read(2)  # attribute byte count
    size = tuple(hi[i] - lo[i] for i in range(3))
    center = tuple((hi[i] + lo[i]) / 2 for i in range(3))
    _stl_bbox_cache[path] = (size, center)
    return size, center


def _apply(T, p):
    x, y, z = p
    return (T[0][0]*x + T[0][1]*y + T[0][2]*z + T[0][3],
            T[1][0]*x + T[1][1]*y + T[1][2]*z + T[1][3],
            T[2][0]*x + T[2][1]*y + T[2][2]*z + T[2][3])


def _centroid(face):
    n = len(face)
    return (sum(p[0] for p in face) / n,
            sum(p[1] for p in face) / n,
            sum(p[2] for p in face) / n)


def box_faces(T, sx, sy, sz):
    """Return the 6 box faces in a FIXED order matching FACE_NAMES/FACE_COLORS:
    [+Z top, -Z bottom, +X, -X, +Y, -Y] (in the box's own frame)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    c = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),   # 0..3 (-z)
         (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]       # 4..7 (+z)
    c = [_apply(T, p) for p in c]
    order = [(4, 5, 6, 7),  # +Z top
             (0, 1, 2, 3),  # -Z bottom
             (1, 2, 6, 5),  # +X
             (0, 3, 7, 4),  # -X
             (2, 3, 7, 6),  # +Y
             (0, 1, 5, 4)]  # -Y
    return [[c[i] for i in f] for f in order]


def cylinder_faces(T, r, length, n=20):
    hz = length / 2
    bot, top = [], []
    for k in range(n):
        a = 2 * math.pi * k / n
        x, y = r * math.cos(a), r * math.sin(a)
        bot.append(_apply(T, (x, y, -hz)))
        top.append(_apply(T, (x, y, hz)))
    faces = [bot, top]
    for k in range(n):
        j = (k + 1) % n
        faces.append([bot[k], bot[j], top[j], top[k]])
    return faces


def parse_urdf(path):
    """Return (root_link, chain, visuals) from a URDF.

    chain   : ordered list of segments {child, xyz, rpy, type, axis} from root to tip
    visuals : {link_name: (kind, params, origin_xyz, origin_rpy)}
    """
    root = ET.parse(path).getroot()

    parent_to_joint = {}
    children = set()
    parents = set()
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        parent_to_joint[p] = j
        parents.add(p)
        children.add(j.find("child").get("link"))
    root_link = next(iter(parents - children))

    chain = []
    link = root_link
    while link in parent_to_joint:
        j = parent_to_joint[link]
        origin = j.find("origin")
        axis = j.find("axis")
        chain.append({
            "child": j.find("child").get("link"),
            "xyz": _triplet(origin.get("xyz") if origin is not None else None),
            "rpy": _triplet(origin.get("rpy") if origin is not None else None),
            "type": j.get("type"),
            "name": j.get("name"),
            "axis": _triplet(axis.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
        })
        link = chain[-1]["child"]

    visuals = {}
    for lk in root.findall("link"):
        vis = lk.find("visual")
        if vis is None:
            continue
        origin = vis.find("origin")
        oxyz = _triplet(origin.get("xyz") if origin is not None else None)
        orpy = _triplet(origin.get("rpy") if origin is not None else None)
        geom = vis.find("geometry")
        box, cyl, mesh = geom.find("box"), geom.find("cylinder"), geom.find("mesh")
        if box is not None:
            visuals[lk.get("name")] = ("box", _triplet(box.get("size")), oxyz, orpy)
        elif cyl is not None:
            visuals[lk.get("name")] = ("cylinder",
                                       (float(cyl.get("radius")), float(cyl.get("length"))),
                                       oxyz, orpy)
        elif mesh is not None:
            # Bounding-box approximation, not the real mesh — see module docstring.
            mesh_path = os.path.join(os.path.dirname(path), mesh.get("filename"))
            scale = _triplet(mesh.get("scale"), (1.0, 1.0, 1.0))
            size, center = _stl_local_bbox(mesh_path)
            box_size = tuple(size[i] * scale[i] for i in range(3))
            box_center = tuple(center[i] * scale[i] for i in range(3))
            visuals[lk.get("name")] = ("box", box_size,
                                       tuple(oxyz[i] + box_center[i] for i in range(3)), orpy)
    return root_link, chain, visuals


def link_world_transforms(root_link, chain, q):
    """World 4x4 transform of every link for joint angles q (revolute order).

    Handles fixed joints anywhere in the chain, so it always matches the URDF.
    """
    frames = {root_link: _translate(0, 0, 0)}
    T = frames[root_link]
    qi = 0
    for seg in chain:
        T = _matmul(T, _translate(*seg["xyz"]))
        if seg["rpy"] != (0.0, 0.0, 0.0):
            T = _matmul(T, _rpy(*seg["rpy"]))
        if seg["type"] in ("revolute", "continuous"):
            T = _matmul(T, _rot_axis(seg["axis"], q[qi]))
            qi += 1
        frames[seg["child"]] = T
    return frames


def collect_faces(visuals, frames):
    """Return (faces, colors, labels) for all links, ready for one collection.

    faces  : list of 4-point polygons (world coords)
    colors : matching per-face colour
    labels : list of (centroid, text) for box faces (None-free); numbers 1..6
    """
    faces, colors, labels = [], [], []
    for name, (kind, params, oxyz, orpy) in visuals.items():
        if name not in frames:
            continue
        Tv = _matmul(_matmul(frames[name], _translate(*oxyz)), _rpy(*orpy))
        if kind == "box":
            fs = box_faces(Tv, *params)
            for i, f in enumerate(fs):
                faces.append(f)
                colors.append(FACE_COLORS[i])
                labels.append((_centroid(f), str(i + 1)))
        else:
            for f in cylinder_faces(Tv, *params):
                faces.append(f)
                colors.append(CYLINDER_COLOR)
    return faces, colors, labels


def joint_axes(chain, frames, length=0.07):
    """For each revolute joint return (name, origin, direction) of its axis in
    world coordinates (evaluated at the given frames / pose)."""
    out = []
    for seg in chain:
        if seg["type"] not in ("revolute", "continuous"):
            continue
        T = frames[seg["child"]]
        ax, ay, az = seg["axis"]
        # World direction = rotation part of the child frame applied to the axis.
        dx = T[0][0]*ax + T[0][1]*ay + T[0][2]*az
        dy = T[1][0]*ax + T[1][1]*ay + T[1][2]*az
        dz = T[2][0]*ax + T[2][1]*ay + T[2][2]*az
        norm = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        d = (dx/norm*length, dy/norm*length, dz/norm*length)
        origin = (T[0][3], T[1][3], T[2][3])
        out.append((seg["name"], origin, d))
    return out


def workspace_bounds(arm, root_link, chain, visuals, margin=0.03):
    """A fixed 1:1:1 cube covering the arm's motion, so the view doesn't jump
    while sliders move. Sampled from each joint at its min/0/max limits."""
    n = len(arm.joints)
    samples = [[0.0] * n]
    for i, joint in enumerate(arm.joints):
        for val in (joint.q_min, joint.q_max):
            q = [0.0] * n
            q[i] = val
            samples.append(q)

    pts = []
    for q in samples:
        faces, _, _ = collect_faces(visuals, link_world_transforms(root_link, chain, q))
        for face in faces:
            pts.extend(face)
    xs, ys, zs = zip(*pts)
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 2 * margin
    cx, cy, cz = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2
    return cx, cy, cz, span


def draw_pose(ax, arm, root_link, chain, visuals, q, bounds):
    """Clear `ax` and draw the arm at joint angles q. Returns the FK position."""
    frames = link_world_transforms(root_link, chain, q)
    faces, colors, labels = collect_faces(visuals, frames)

    ax.cla()
    coll = Poly3DCollection(faces, facecolors=colors, edgecolor="#20232a",
                            linewidths=0.4, alpha=1.0, zorder=0)
    coll.set_zsort("average")
    ax.add_collection3d(coll)

    for (cx, cy, cz), text in labels:
        ax.text(cx, cy, cz, text, fontsize=5, ha="center", va="center",
                color="white", zorder=5)

    for name, (ox, oy, oz), (dx, dy, dz) in joint_axes(chain, frames):
        ax.quiver(ox - dx, oy - dy, oz - dz, 2*dx, 2*dy, 2*dz,
                  color="#ffd600", linewidth=2.2, arrow_length_ratio=0.14, zorder=8)

    ee = arm.fk(q)
    ax.scatter([ee[0]], [ee[1]], [ee[2]], color="red", s=45, zorder=9)

    bx, by, bz, span = bounds
    ax.set_xlim(bx - span/2, bx + span/2)
    ax.set_ylim(by - span/2, by + span/2)
    ax.set_zlim(bz - span/2, bz + span/2)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"FK: end-effector = ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f}) m")
    handles = [Patch(facecolor=FACE_COLORS[i], edgecolor="#20232a",
                     label=f"{i+1}: {FACE_NAMES[i]}") for i in range(6)]
    ax.legend(handles=handles, loc="upper left", fontsize=7, title="Box faces")
    return ee


def draw_points(ax, points, connections=None, point_color="lime", line_color="lime",
                s=25, linewidth=1.5, zorder=10):
    """Scatter `points` (world xyz) and draw `connections` (index pairs into
    `points`) as 3D line segments, on top of whatever draw_pose already drew.

    Generic overlay, not robot-specific — used to render e.g. a mediapipe hand
    skeleton (21 landmarks + HAND_CONNECTIONS) alongside the arm.
    """
    if not points:
        return
    xs, ys, zs = zip(*points)
    ax.scatter(xs, ys, zs, color=point_color, s=s, zorder=zorder)
    if connections:
        for i, j in connections:
            if i < len(points) and j < len(points):
                p, q = points[i], points[j]
                ax.plot3D([p[0], q[0]], [p[1], q[1]], [p[2], q[2]],
                          color=line_color, linewidth=linewidth, zorder=zorder)


def main():
    arm = load_arm()
    root_link, chain, visuals = parse_urdf(_DEFAULT_URDF_PATH)
    bounds = workspace_bounds(arm, root_link, chain, visuals)

    fig = plt.figure(figsize=(10, 9))
    # computed_zorder=False so joint-axis arrows / face numbers always draw on
    # top of the opaque boxes instead of being occluded by them.
    ax = fig.add_axes([0.02, 0.34, 0.74, 0.62], projection="3d", computed_zorder=False)
    ax.view_init(elev=22, azim=-55)

    status = fig.text(0.5, 0.015, "", ha="center", fontsize=9, color="#333")

    def current_q():
        # Sliders carry servo degrees; convert to joint angle q via each joint's
        # own calibration (home_deg / direction).
        return [joint.q_from_servo(s.val) for joint, s in zip(arm.joints, sliders)]

    def redraw(_=None):
        draw_pose(ax, arm, root_link, chain, visuals, current_q(), bounds)
        fig.canvas.draw_idle()

    # ---- FK: one slider per joint (servo degree, 30..330), starts at home ----
    sliders = []
    for i, joint in enumerate(arm.joints):
        sax = fig.add_axes([0.10, 0.25 - i * 0.038, 0.52, 0.025])
        lo, hi = sorted((joint.servo_deg(joint.q_min), joint.servo_deg(joint.q_max)))
        s = Slider(sax, f"{joint.name} (servo°)", lo, hi, valinit=joint.home_deg)
        s.on_changed(redraw)
        sliders.append(s)

    # ---- IK: type a target x/y/z, solve, and snap the sliders to it ----
    tb_x = TextBox(fig.add_axes([0.83, 0.24, 0.12, 0.045]), "x ", initial="0.00")
    tb_y = TextBox(fig.add_axes([0.83, 0.18, 0.12, 0.045]), "y ", initial="0.00")
    tb_z = TextBox(fig.add_axes([0.83, 0.12, 0.12, 0.045]), "z ", initial="0.30")
    btn_ik = Button(fig.add_axes([0.83, 0.05, 0.12, 0.05]), "Solve IK")

    def solve_ik(_):
        try:
            target = (float(tb_x.text), float(tb_y.text), float(tb_z.text))
        except ValueError:
            status.set_text("IK: x, y, z must be numbers")
            fig.canvas.draw_idle()
            return
        q, ok = arm.ik(target, seed=current_q())
        if ok:
            # set_val triggers each slider's on_changed -> redraw.
            for joint, s, qi in zip(arm.joints, sliders, q):
                s.set_val(joint.servo_deg(qi))
            status.set_text(f"IK converged  ->  servo(deg) = "
                            f"{[round(d) for d in arm.q_to_servo_deg(q)]}")
        else:
            status.set_text(f"IK did NOT converge for {target} (out of reach?)")
        fig.canvas.draw_idle()

    btn_ik.on_clicked(solve_ik)

    redraw()
    Logger.log("SIM", "Interactive FK/IK viewer: drag joint sliders (FK), "
                      "or enter x/y/z and press Solve IK")
    plt.show()


if __name__ == "__main__":
    main()
