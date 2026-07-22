"""3D preview of the desky arm — no hardware required.

For now this only renders the arm at its HOME pose (all joint angles = 0) using
the URDF <visual> geometry (solid boxes/cylinders per link), so the preview
looks like the actual robot rather than a stick figure. It reads the same
geometry source as the real robot (kinematics/desky.urdf).

Rendering notes:
  * Every box face is given its own colour AND a number (1..6), so you can tell
    which face is which (1 = +Z top, 2 = -Z bottom, 3 = +X, 4 = -X, 5 = +Y,
    6 = -Y, in each link's own frame). A legend maps number -> face.
  * Each revolute joint's rotation axis is drawn as an arrow at the joint.
  * All link faces go into ONE Poly3DCollection so matplotlib depth-sorts them
    per-face. With a separate collection per link, matplotlib sorts whole links
    against each other (painter's algorithm), which makes overlapping links pop
    in front of one another incorrectly.

The animated DEMO_SEQUENCE playback is intentionally left out until the rest of
the pipeline is in place; add it back on top of this static renderer later.

Requires matplotlib (not a dependency of the kinematics package itself):
    pip install matplotlib
"""

import math
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from kinematics.kinematics import _matmul, _translate, _rpy, _rot_axis
from kinematics.urdf_loader import load_arm, _DEFAULT_URDF_PATH
from logger import Logger

# Box faces are always emitted in this order; index i -> label (i+1).
FACE_NAMES = ["+Z (top)", "-Z (bottom)", "+X", "-X", "+Y", "-Y"]
FACE_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4"]
CYLINDER_COLOR = "#9e9e9e"


# ---------------------------------------------------------------------------
# URDF visual geometry
# ---------------------------------------------------------------------------
def _triplet(text, default=(0.0, 0.0, 0.0)):
    return tuple(float(v) for v in text.split()) if text else default


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
        box, cyl = geom.find("box"), geom.find("cylinder")
        if box is not None:
            visuals[lk.get("name")] = ("box", _triplet(box.get("size")), oxyz, orpy)
        elif cyl is not None:
            visuals[lk.get("name")] = ("cylinder",
                                       (float(cyl.get("radius")), float(cyl.get("length"))),
                                       oxyz, orpy)
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


def main():
    arm = load_arm()
    root_link, chain, visuals = parse_urdf(_DEFAULT_URDF_PATH)

    # Home pose: every joint angle at 0.
    q_home = [0.0] * len(arm.joints)
    frames = link_world_transforms(root_link, chain, q_home)

    faces, colors, labels = collect_faces(visuals, frames)

    fig = plt.figure(figsize=(9, 9))
    # computed_zorder=False so our manual zorders win: the single face collection
    # stays behind, joint-axis arrows and face numbers always draw on top instead
    # of being occluded by opaque boxes.
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("desky 5-DOF arm — home pose (URDF visual)")

    # Single collection => matplotlib depth-sorts every face together, which
    # fixes the "one whole link jumps in front of another" overlap artefact.
    coll = Poly3DCollection(faces, facecolors=colors, edgecolor="#20232a",
                            linewidths=0.4, alpha=1.0, zorder=0)
    coll.set_zsort("average")
    ax.add_collection3d(coll)

    # Face numbers (on top of the faces).
    for (cx, cy, cz), text in labels:
        ax.text(cx, cy, cz, text, fontsize=6, ha="center", va="center",
                color="white", zorder=5)

    # Joint rotation axes (arrows) + labels, always drawn on top.
    for name, (ox, oy, oz), (dx, dy, dz) in joint_axes(chain, frames):
        ax.quiver(ox - dx, oy - dy, oz - dz, 2*dx, 2*dy, 2*dz,
                  color="#ffd600", linewidth=2.6, arrow_length_ratio=0.15, zorder=8)
        ax.text(ox + dx, oy + dy, oz + dz, f"  {name} axis", fontsize=7,
                color="#c17900", zorder=9)

    # Equal 1:1:1 aspect from every vertex.
    pts = [p for f in faces for p in f]
    xs, ys, zs = zip(*pts)
    m = 0.03
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) + 2 * m
    cx, cy, cz = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2, (min(zs)+max(zs))/2
    ax.set_xlim(cx - span/2, cx + span/2)
    ax.set_ylim(cy - span/2, cy + span/2)
    ax.set_zlim(cz - span/2, cz + span/2)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=22, azim=-55)

    # Legend: number -> face.
    handles = [Patch(facecolor=FACE_COLORS[i], edgecolor="#20232a",
                     label=f"{i+1}: {FACE_NAMES[i]}") for i in range(6)]
    ax.legend(handles=handles, loc="upper left", fontsize=8, title="Box faces (link frame)")

    Logger.log("SIM", f"Rendering home pose, end-effector at "
                      f"{tuple(round(v, 3) for v in arm.fk(q_home))}")
    plt.show()


if __name__ == "__main__":
    main()
