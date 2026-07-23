"""Load a URDF file into a kinematics.Arm.

Parses the standard URDF <link>/<joint> tree (revolute + fixed joints) plus the
project-specific <dynamixel> extension that carries the DYNAMIXEL id/model and
servo calibration. Serial chains only (one child per link), which matches the
desky arm.

Only Python's standard library is used (xml.etree.ElementTree).
"""

import math
import os
import xml.etree.ElementTree as ET

from .kinematics import Arm, Joint, _matmul, _translate, _rpy
from logger import Logger

_DEFAULT_URDF_PATH = os.path.join(os.path.dirname(__file__), "configure", "desky.urdf")


def _parse_triplet(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return default
    parts = [float(v) for v in text.split()]
    return tuple(parts)


def _rpy_from_matrix(R):
    """Extract URDF (roll, pitch, yaw) from a 3x3 rotation R = Rz*Ry*Rx."""
    pitch = math.atan2(-R[2][0], math.sqrt(R[0][0] ** 2 + R[1][0] ** 2))
    if abs(math.cos(pitch)) < 1e-9:  # gimbal lock
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    else:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    return (roll, pitch, yaw)


def load_arm(path=None):
    """Parse `path` (default: desky.urdf next to this file) and return a
    kinematics.Arm following the serial chain."""
    root = ET.parse(path or _DEFAULT_URDF_PATH).getroot()

    joints = {}          # name -> joint element
    child_to_joint = {}  # child link name -> joint element
    children = set()
    parents = set()
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        joints[name] = j
        child_to_joint[child] = j
        parents.add(parent)
        children.add(child)

    # Root link = a link that is some joint's parent but never a child.
    roots = parents - children
    if len(roots) != 1:
        raise ValueError(f"Expected exactly one root link, found {sorted(roots)}")
    link = next(iter(roots))

    # Walk the chain from root to tip, following parent -> child joints.
    ordered = []
    parent_to_joint = {j.find("parent").get("link"): j for j in root.findall("joint")}
    while link in parent_to_joint:
        j = parent_to_joint[link]
        ordered.append(j)
        link = j.find("child").get("link")

    arm_joints = []
    pending = _translate(0, 0, 0)  # accumulated fixed transform since last revolute
    for j in ordered:
        jtype = j.get("type")
        origin = j.find("origin")
        xyz = _parse_triplet(origin.get("xyz") if origin is not None else None)
        rpy = _parse_triplet(origin.get("rpy") if origin is not None else None)

        # Compose this joint's fixed origin into the pending transform.
        step = _matmul(_translate(*xyz), _rpy(*rpy))
        pending = _matmul(pending, step)

        if jtype in ("revolute", "continuous"):
            offset = (pending[0][3], pending[1][3], pending[2][3])
            R = [[pending[i][k] for k in range(3)] for i in range(3)]
            eff_rpy = _rpy_from_matrix(R)

            axis = _parse_triplet(j.find("axis").get("xyz")) if j.find("axis") is not None \
                else (0.0, 0.0, 1.0)

            limit = j.find("limit")
            q_min = float(limit.get("lower", -math.pi / 2)) if limit is not None else -math.pi / 2
            q_max = float(limit.get("upper", math.pi / 2)) if limit is not None else math.pi / 2

            dxl = j.find("dynamixel")
            if dxl is not None:
                jid = int(dxl.get("id"))
                home_deg = float(dxl.get("home_deg", 150.0))
                direction = int(dxl.get("direction", 1))
            else:
                jid = len(arm_joints) + 1
                home_deg, direction = 150.0, 1

            arm_joints.append(Joint(
                id=jid, axis=axis, offset=offset, rpy=eff_rpy,
                home_deg=home_deg, direction=direction,
                q_min=q_min, q_max=q_max, name=j.get("name")))
            pending = _translate(0, 0, 0)  # reset accumulator after consuming
        # fixed joints just keep accumulating into `pending`

    # Whatever fixed transform trails the last revolute is the tool offset.
    tool_offset = (pending[0][3], pending[1][3], pending[2][3])
    return Arm(joints=arm_joints, tool_offset=tool_offset)


if __name__ == "__main__":
    arm = load_arm()
    Logger.log("URDF", "Loaded joints from URDF:")
    for j in arm.joints:
        Logger.log("URDF", f"  {j.name}: id={j.id} axis={j.axis} offset={j.offset} "
                   f"limits=({round(j.q_min, 3)}, {round(j.q_max, 3)})")
    Logger.log("URDF", f"Tool offset: {arm.tool_offset}")

    q0 = [0.0] * len(arm.joints)
    Logger.log("URDF", f"FK at home pose: {tuple(round(v, 4) for v in arm.fk(q0))}")

    q_true = [0.3, 0.2, -0.4, 0.5, 0.1]
    target = arm.fk(q_true)
    q_sol, ok = arm.ik(target, seed=[0.0] * len(arm.joints))
    reached = arm.fk(q_sol)
    err = math.sqrt(sum((a - b) ** 2 for a, b in zip(target, reached)))
    Logger.log("URDF", f"IK round-trip converged={ok}, position error={err:.6f} m")
    Logger.log("URDF", f"Servo commands (deg): {[round(d, 1) for d in arm.q_to_servo_deg(q_sol)]}")
