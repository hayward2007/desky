"""Sweep each joint's MuJoCo self-collision-free range and update
kinematics/configure/desky.urdf's <limit> (and joint2's <coupled_limit>)
accordingly.

For joints 1, 3, 4, 5: every other joint is held at its home position (q=0)
and the joint under test is stepped outward from q=0 in both directions, one
degree-scale step at a time, until MuJoCo reports a NEW contact pair (see
below) or the servo's physical range (0-300 deg) is reached.

joint2 is different: its self-collision-free range actually depends on
joint3's current angle (the two links pass close enough to each other that
rotating the shoulder narrows or widens how far the roll joint can safely
turn). A single static <limit> for joint2 would have to be the worst case
across every joint3 angle, which is needlessly restrictive everywhere else.
So instead: joint3 is swept across its own resting range at COUPLE_SAMPLES
points, and at each one joint2 is independently swept (same algorithm as
above) to find its collision-free range *at that joint3 angle*. That table
of (joint3 angle -> joint2 range) points is written into the URDF as a
project-specific <coupled_limit joint="joint3"> extension inside joint2's
<joint> element (standard URDF tools ignore it; kinematics.urdf_loader and
kinematics.kinematics.Joint.bounds()/clamp() interpolate it). joint2's plain
<limit> is still written too, as the intersection across all sampled joint3
angles — the conservative bound that holds no matter what joint3 is doing,
used as a fallback by anything that doesn't know about the coupling.

Only single-joint (or, for joint2, single-pair) sweeps are checked, not the
full 5-joint combination — a real 5D sweep is combinatorially expensive, and
the cases this project's arm actually needs guarding against (a link
swinging back into an earlier or neighboring link) already show up this way.

joint5 additionally gets a hard cap (JOINT5_CAMERA_SAFE_SERVO_DEG) intersected
with its self-collision range — it carries the phone/camera mount, and MuJoCo
has no model of the camera or its cable to discover that constraint on its own.

Requires kinematics/configure/desky.urdf's <collision> geoms (added
alongside <visual>) — MuJoCo only generates contacts between geoms that are
marked as collidable, and visual-only geoms are contype=0/conaffinity=0.
MuJoCo collides mesh geoms as their convex hulls, not the exact (often
concave/interlocking) shape, so mated neighboring parts — e.g. a dynamixel
seated inside its bracket — already show a "contact" at rest even though the
real parts fit together fine. To not treat that as a false self-collision,
the geom-pair set present at the pose being swept *from* is taken as that
sweep's baseline and excluded; a sweep only stops when a geom pair *not* in
its baseline starts touching.

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
from fundamental.const import FindJointLimitsConst
from fundamental.logger import Logger

STEP_DEG = FindJointLimitsConst.STEP_DEG                      # sweep granularity
SAFETY_MARGIN_DEG = FindJointLimitsConst.SAFETY_MARGIN_DEG    # back off the found collision boundary by this much
COUPLE_SAMPLES = FindJointLimitsConst.COUPLE_SAMPLES          # joint3 sample points for joint2's coupled_limit table

# joint5 carries the phone/camera mount. MuJoCo only knows about mesh
# self-collision, not the camera or its cable, so it can't discover on its
# own that an extreme wrist rotation would wind up the cable or leave the
# camera pointed somewhere useless — this is a hard, independent cap,
# intersected with whatever the self-collision sweep finds for joint5.
JOINT5_CAMERA_SAFE_SERVO_DEG = FindJointLimitsConst.JOINT5_CAMERA_SAFE_SERVO_DEG


def _physical_bounds(joint):
    """The joint's q range from the servo's physical 0-300 deg travel."""
    a, b = joint.q_from_servo(0.0), joint.q_from_servo(300.0)
    return min(a, b), max(a, b)


def _contact_pairs(data):
    return {(min(c.geom1, c.geom2), max(c.geom1, c.geom2)) for c in data.contact[: data.ncon]}


def _sweep(model, data, qpos_addr, direction, q_bound, baseline_pairs):
    """Step qpos_addr away from its current value in `direction` (+1/-1)
    until a geom pair not in `baseline_pairs` starts touching, or q_bound is
    reached. Returns the last collision-free q. Leaves every other qpos
    untouched; resets qpos_addr itself back to its starting value."""
    start = data.qpos[qpos_addr]
    step = math.radians(STEP_DEG) * direction
    q = start
    safe_q = start
    while abs(q - start) < abs(q_bound - start):
        q += step
        q = max(q_bound, q) if direction < 0 else min(q_bound, q)
        data.qpos[qpos_addr] = q
        mujoco.mj_forward(model, data)
        if _contact_pairs(data) - baseline_pairs:
            break
        safe_q = q
    data.qpos[qpos_addr] = start
    return safe_q


def _margin_bounds(raw_min, raw_max, margin, center=0.0):
    q_max = max(center, raw_max - margin)
    q_min = min(center, raw_min + margin)
    return q_min, q_max


def find_independent_limits(model, data, qpos_addrs, arm, margin):
    """Single-joint sweep (others at home) for every joint. Returns
    {joint_id: (q_min, q_max, reason)}."""
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

    results = {}
    for idx, joint in enumerate(arm.joints):
        for addr in qpos_addrs:
            data.qpos[addr] = 0.0

        q_phys_min, q_phys_max = _physical_bounds(joint)
        raw_max = _sweep(model, data, qpos_addrs[idx], +1, q_phys_max, baseline_pairs)
        raw_min = _sweep(model, data, qpos_addrs[idx], -1, q_phys_min, baseline_pairs)
        q_min, q_max = _margin_bounds(raw_min, raw_max, margin)

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
        results[joint.id] = (q_min, q_max, reason)
    return results


def find_coupled_limit(model, data, qpos_addrs, arm, moving_id, other_id, other_range, margin):
    """Sweep `moving_id` (e.g. joint2) at COUPLE_SAMPLES angles of `other_id`
    (e.g. joint3) spanning other_range, every other joint at home. Returns
    (table, static_min, static_max) — table is [(other_q, q_min, q_max), ...]
    sorted by other_q, and static_{min,max} is the intersection across every
    sample (the conservative single-range fallback)."""
    idx_moving = arm.id_to_index[moving_id]
    idx_other = arm.id_to_index[other_id]
    joint = arm.joints[idx_moving]
    q_phys_min, q_phys_max = _physical_bounds(joint)
    other_min, other_max = other_range

    table = []
    for i in range(COUPLE_SAMPLES):
        other_q = other_min + (other_max - other_min) * i / (COUPLE_SAMPLES - 1)

        for addr in qpos_addrs:
            data.qpos[addr] = 0.0
        data.qpos[qpos_addrs[idx_other]] = other_q
        mujoco.mj_forward(model, data)
        baseline_pairs = _contact_pairs(data)

        raw_max = _sweep(model, data, qpos_addrs[idx_moving], +1, q_phys_max, baseline_pairs)
        raw_min = _sweep(model, data, qpos_addrs[idx_moving], -1, q_phys_min, baseline_pairs)
        q_min, q_max = _margin_bounds(raw_min, raw_max, margin)
        table.append((other_q, q_min, q_max))
        Logger.log(
            "LIMITS",
            f"  joint{other_id}={math.degrees(other_q):+.1f} deg -> "
            f"joint{moving_id} in [{math.degrees(q_min):.1f}, {math.degrees(q_max):.1f}] deg",
        )

    static_min = max(t[1] for t in table)
    static_max = min(t[2] for t in table)
    if static_min > static_max:
        Logger.log(
            "LIMITS",
            f"WARNING: joint{moving_id}'s per-{('joint' + str(other_id))} ranges don't all "
            "overlap; static fallback would be empty — widening to the envelope instead",
        )
        static_min = min(t[1] for t in table)
        static_max = max(t[2] for t in table)

    Logger.log(
        "LIMITS",
        f"joint{moving_id} static fallback (valid for any joint{other_id}): "
        f"[{math.degrees(static_min):.1f}, {math.degrees(static_max):.1f}] deg",
    )
    return table, static_min, static_max


def _apply_camera_cap(arm, results):
    """Intersect joint5's computed range with JOINT5_CAMERA_SAFE_SERVO_DEG."""
    joint5 = arm.joints[arm.id_to_index[5]]
    cap_lo, cap_hi = sorted((
        joint5.q_from_servo(JOINT5_CAMERA_SAFE_SERVO_DEG[0]),
        joint5.q_from_servo(JOINT5_CAMERA_SAFE_SERVO_DEG[1]),
    ))
    capped = []
    for jid, q_min, q_max in results:
        if jid == 5:
            new_min, new_max = max(q_min, cap_lo), min(q_max, cap_hi)
            Logger.log(
                "LIMITS",
                f"joint5: capped to camera-safe [{math.degrees(new_min):.1f}, "
                f"{math.degrees(new_max):.1f}] deg (servo {JOINT5_CAMERA_SAFE_SERVO_DEG})",
            )
            capped.append((jid, new_min, new_max))
        else:
            capped.append((jid, q_min, q_max))
    return capped


def find_limits():
    arm = load_arm()
    model = mujoco.MjModel.from_xml_path(_DEFAULT_URDF_PATH)
    data = mujoco.MjData(model)
    qpos_addrs = _qpos_addrs(model, arm)
    margin = math.radians(SAFETY_MARGIN_DEG)

    independent = find_independent_limits(model, data, qpos_addrs, arm, margin)
    joint3_range = independent[3][:2]

    Logger.log("LIMITS", "Sweeping joint2 across joint3's range for the coupled limit...")
    table, static_min, static_max = find_coupled_limit(
        model, data, qpos_addrs, arm, moving_id=2, other_id=3,
        other_range=joint3_range, margin=margin,
    )

    static_results = [(jid, q_min, q_max) for jid, (q_min, q_max, _reason) in independent.items()]
    static_results = [
        (jid, static_min, static_max) if jid == 2 else (jid, q_min, q_max)
        for jid, q_min, q_max in static_results
    ]
    static_results = _apply_camera_cap(arm, static_results)
    coupled = (2, 3, table)
    return static_results, coupled


def write_urdf(static_results, coupled=None):
    with open(_DEFAULT_URDF_PATH) as f:
        text = f.read()

    limits = {jid: (q_min, q_max) for jid, q_min, q_max in static_results}

    for joint_id, (q_min, q_max) in limits.items():
        block_pattern = re.compile(rf'<joint name="joint{joint_id}"[^>]*>.*?</joint>', re.DOTALL)
        m = block_pattern.search(text)
        if not m:
            raise ValueError(f'joint{joint_id} not found in {_DEFAULT_URDF_PATH}')
        block = m.group(0)

        block, n = re.subn(
            r'(<limit lower=")[^"]*(" upper=")[^"]*(")',
            rf"\g<1>{q_min:.5f}\g<2>{q_max:.5f}\g<3>",
            block, count=1,
        )
        if n != 1:
            raise ValueError(f'no <limit> found for joint{joint_id} in {_DEFAULT_URDF_PATH}')

        if coupled is not None and coupled[0] == joint_id:
            _, other_id, table = coupled
            points = "\n".join(
                f'      <point q="{q:.5f}" lower="{lo:.5f}" upper="{hi:.5f}"/>'
                for q, lo, hi in table
            )
            new_cl = f'<coupled_limit joint="joint{other_id}">\n{points}\n    </coupled_limit>'
            existing_cl = re.search(r"<coupled_limit\b.*?</coupled_limit>", block, re.DOTALL)
            if existing_cl:
                block = block[: existing_cl.start()] + new_cl + block[existing_cl.end() :]
            else:
                block, n = re.subn(
                    r'(<limit lower="[^"]*" upper="[^"]*"[^/]*/>)',
                    lambda mo: mo.group(1) + "\n    " + new_cl,
                    block, count=1,
                )

        text = text[: m.start()] + block + text[m.end() :]

    with open(_DEFAULT_URDF_PATH, "w") as f:
        f.write(text)


if __name__ == "__main__":
    static_results, coupled = find_limits()
    if "--dry-run" in sys.argv:
        Logger.log("LIMITS", "--dry-run: URDF not modified")
    else:
        write_urdf(static_results, coupled)
        Logger.log("LIMITS", f"Updated {_DEFAULT_URDF_PATH}")
