"""Sweep each joint's MuJoCo self-collision-free range and update the
<limit> values in kinematics/configure/desky.urdf accordingly.

For each joint, every other joint is held at its home position (q=0) and the
joint under test is stepped outward from q=0 in both directions, one degree-
scale step at a time, until MuJoCo reports a NEW contact pair (see below) or
the servo's physical range (0-300 deg) is reached. Only checks that
single-joint sweep against a fixed home pose, not the full 5-joint
combination — a real 5D sweep is combinatorially expensive, and the cases
this project's arm actually needs guarding against (a link swinging back
into an earlier link) already show up when sweeping one joint at a time.

Requires kinematics/configure/desky.urdf's <collision> geoms (added
alongside <visual>) — MuJoCo only generates contacts between geoms that are
marked as collidable, and visual-only geoms are contype=0/conaffinity=0.
MuJoCo collides mesh geoms as their convex hulls, not the exact (often
concave/interlocking) shape, so mated neighboring parts — e.g. a dynamixel
seated inside its bracket — already show a "contact" at the home pose (q=0
for every joint) even though the real parts fit together fine. To not treat
that as a false self-collision, the geom-pair set present at the all-zero
home pose is taken as the baseline and excluded; the sweep only stops when a
geom pair *not* in that baseline starts touching.

Run from the repository root:
    python -m kinematics.find_joint_limits          # report + write to URDF
    python -m kinematics.find_joint_limits --dry-run # report only
"""

import math
import re
import sys

import mujoco

from .mujoco_sim import _qpos_addrs
from .urdf_loader import _DEFAULT_URDF_PATH, load_arm
from logger import Logger

STEP_DEG = 0.5          # sweep granularity
SAFETY_MARGIN_DEG = 2.0  # back off the found collision boundary by this much


def _physical_bounds(joint):
    """The joint's q range from the servo's physical 0-300 deg travel."""
    a, b = joint.q_from_servo(0.0), joint.q_from_servo(300.0)
    return min(a, b), max(a, b)


def _contact_pairs(data):
    return {(min(c.geom1, c.geom2), max(c.geom1, c.geom2)) for c in data.contact[: data.ncon]}


def _sweep(model, data, qpos_addr, direction, q_bound, baseline_pairs):
    """Step qpos_addr away from 0 in `direction` (+1/-1) until a geom pair not
    in `baseline_pairs` starts touching, or q_bound is reached. Returns the
    last collision-free q."""
    step = math.radians(STEP_DEG) * direction
    q = 0.0
    safe_q = 0.0
    while abs(q) < abs(q_bound):
        q += step
        q = max(q_bound, q) if direction < 0 else min(q_bound, q)
        data.qpos[qpos_addr] = q
        mujoco.mj_forward(model, data)
        if _contact_pairs(data) - baseline_pairs:
            break
        safe_q = q
    data.qpos[qpos_addr] = 0.0
    return safe_q


def find_limits():
    arm = load_arm()
    model = mujoco.MjModel.from_xml_path(_DEFAULT_URDF_PATH)
    data = mujoco.MjData(model)
    qpos_addrs = _qpos_addrs(model, arm)
    margin = math.radians(SAFETY_MARGIN_DEG)

    for addr in qpos_addrs:
        data.qpos[addr] = 0.0
    mujoco.mj_forward(model, data)
    baseline_pairs = _contact_pairs(data)
    if baseline_pairs:
        Logger.log(
            "LIMITS",
            f"{len(baseline_pairs)} geom pair(s) already touch at the home pose "
            "(mated parts / convex-hull artifacts) — excluded as baseline",
        )

    results = []
    for idx, joint in enumerate(arm.joints):
        for addr in qpos_addrs:
            data.qpos[addr] = 0.0

        q_phys_min, q_phys_max = _physical_bounds(joint)
        raw_max = _sweep(model, data, qpos_addrs[idx], +1, q_phys_max, baseline_pairs)
        raw_min = _sweep(model, data, qpos_addrs[idx], -1, q_phys_min, baseline_pairs)

        q_max = max(0.0, raw_max - margin)
        q_min = min(0.0, raw_min + margin)

        limited_by = []
        if raw_max < q_phys_max - 1e-6:
            limited_by.append("+collision")
        if raw_min > q_phys_min + 1e-6:
            limited_by.append("-collision")
        reason = ", ".join(limited_by) if limited_by else "hardware range only"

        Logger.log(
            "LIMITS",
            f"joint{joint.id}: [{math.degrees(q_min):.1f}, {math.degrees(q_max):.1f}] deg "
            f"({reason})",
        )
        results.append((joint.id, q_min, q_max))
    return results


def write_limits(results):
    with open(_DEFAULT_URDF_PATH) as f:
        text = f.read()

    for joint_id, q_min, q_max in results:
        pattern = re.compile(
            rf'(<joint name="joint{joint_id}"[^>]*>.*?<limit lower=")[^"]*(" upper=")[^"]*(")',
            re.DOTALL,
        )
        text, n = pattern.subn(rf"\g<1>{q_min:.5f}\g<2>{q_max:.5f}\g<3>", text, count=1)
        if n != 1:
            raise ValueError(f'could not find <limit> for joint{joint_id} in {_DEFAULT_URDF_PATH}')

    with open(_DEFAULT_URDF_PATH, "w") as f:
        f.write(text)


if __name__ == "__main__":
    results = find_limits()
    if "--dry-run" in sys.argv:
        Logger.log("LIMITS", "--dry-run: URDF not modified")
    else:
        write_limits(results)
        Logger.log("LIMITS", f"Updated {_DEFAULT_URDF_PATH}")
