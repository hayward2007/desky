"""Interactive mesh-based FK/IK preview of the desky arm using MuJoCo.

Loads kinematics/configure/desky.urdf — the real STL meshes from the Fusion
360 export (kinematics/meshes/), joined exactly like the physical build:

    base_link -> base_shell -> dynamixel1 -> guard -> bracket_f5
              -> dynamixel2 -> bracket_f6 -> dynamixel3 -> link1
              -> dynamixel4 -> link2 -> dynamixel5 -> end_effector -> tool_tip

via MuJoCo's built-in URDF compiler (mujoco.MjModel.from_xml_path handles a
URDF root the same as an MJCF one), opens MuJoCo's passive viewer, and drives
it with plain terminal commands using the exact same math as the real robot
(kinematics.kinematics.Arm — the same FK/IK hardware.util.ArmController
uses). MuJoCo's own viewer has no scriptable slider/text-box widgets to hook
custom FK/IK controls into (unlike pybullet's addUserDebugParameter), so
input goes through stdin instead, in a background thread so it doesn't block
the render loop:

    fk <id 1-5> <servo_deg>   e.g. `fk 1 200`       -- move one joint
    ik <x> <y> <z>            e.g. `ik 0.1 0 0.3`    -- solve IK, same Arm.ik()
    home                      -- reset every joint to q=0 (servo home)
    quit / q

Each joint's rotation axis is drawn as a colored arrow (one color per joint
id), rooted at the joint's world-space anchor and pointing along its
world-space axis — both read straight from MuJoCo's data.xanchor/data.xaxis
after mj_forward, so the arrows track FK/IK moves automatically.

Requires mujoco (not a dependency of the kinematics package itself):
    pip install mujoco

Run from the repository root:
    python -m kinematics.mujoco_sim
"""

import queue
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from .urdf_loader import load_arm, _DEFAULT_URDF_PATH
from fundamental.const import MujocoSimConst
from fundamental.logger import Logger

# Joint-axis overlay: one colored arrow per revolute joint, drawn from its
# world-space anchor (data.xanchor) along its world-space rotation axis
# (data.xaxis) — MuJoCo computes both every mj_forward, so no extra FK math
# is needed here. Colors just cycle per joint id so the 5 arrows are visually
# distinguishable; they don't encode yaw/roll/pitch.
_AXIS_LEN = MujocoSimConst.AXIS_LEN
_AXIS_WIDTH = MujocoSimConst.AXIS_WIDTH
_AXIS_COLORS = MujocoSimConst.AXIS_COLORS


def _reclamp_all(arm, q):
    """Re-clamp every joint against its current coupled partner's angle (if
    any). Needed because moving one joint (e.g. joint3) can push another
    joint (e.g. joint2) outside the range that's now safe for it, even if
    that other joint didn't move itself — see Joint.coupled_table."""
    q = q[:]
    for i, joint in enumerate(arm.joints):
        q_other = q[arm.id_to_index[joint.coupled_with]] if joint.coupled_with is not None else None
        q[i] = joint.clamp(q[i], q_other=q_other)
    return q


def _draw_joint_axes(viewer, model, data):
    scn = viewer.user_scn
    scn.ngeom = 0
    for jid in range(model.njnt):
        if scn.ngeom >= scn.maxgeom:
            break
        anchor = data.xanchor[jid]
        axis = data.xaxis[jid]
        p0 = anchor
        p1 = anchor + _AXIS_LEN * axis
        g = scn.geoms[scn.ngeom]
        rgba = np.array(_AXIS_COLORS[jid % len(_AXIS_COLORS)], dtype=np.float32)
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                             np.eye(3).flatten(), rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, _AXIS_WIDTH, p0, p1)
        scn.ngeom += 1


def _qpos_addrs(model, arm):
    """qpos index for each arm.joints[i], looked up by the URDF joint name
    (f"joint{joint.id}") MuJoCo's URDF compiler carries over unchanged."""
    addrs = []
    for joint in arm.joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{joint.id}")
        if jid < 0:
            raise ValueError(f"joint{joint.id} not found in compiled MuJoCo model")
        addrs.append(model.jnt_qposadr[jid])
    return addrs


def _read_stdin(cmd_queue):
    while True:
        try:
            line = input()
        except EOFError:
            break
        cmd_queue.put(line.strip())
        if line.strip().lower() in ("quit", "q"):
            break


def main():
    arm = load_arm()
    model = mujoco.MjModel.from_xml_path(_DEFAULT_URDF_PATH)
    data = mujoco.MjData(model)
    qpos_addrs = _qpos_addrs(model, arm)

    q = [0.0] * len(arm.joints)

    def apply(q_new):
        nonlocal q
        q = q_new
        for addr, qi in zip(qpos_addrs, q):
            data.qpos[addr] = qi
        mujoco.mj_forward(model, data)  # kinematics only, no dynamics stepping

    apply(q)

    cmd_queue = queue.Queue()
    threading.Thread(target=_read_stdin, args=(cmd_queue,), daemon=True).start()

    Logger.log("SIM", "Interactive mesh FK/IK viewer (MuJoCo). Commands (in this terminal):")
    Logger.log("SIM", "  fk <id 1-5> <servo_deg>   e.g. 'fk 1 200'")
    Logger.log("SIM", "  ik <x> <y> <z>            e.g. 'ik 0.1 0 0.3'")
    Logger.log("SIM", "  home                      reset every joint to q=0")
    Logger.log("SIM", "  quit / q")
    Logger.log("SIM", "In the viewer window: scroll to zoom, right-drag to pan, "
                      "left-drag to orbit, double-click a body to centre on it.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # The arm is ~0.4m tall; MuJoCo's default camera framing isn't tuned
        # for a model this small and can leave it out of frame entirely.
        viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
        viewer.cam.distance = 0.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25

        while viewer.is_running():
            try:
                line = cmd_queue.get_nowait()
            except queue.Empty:
                line = None

            if line:
                parts = line.split()
                cmd = parts[0].lower() if parts else ""
                try:
                    if cmd == "fk" and len(parts) == 3:
                        jid, deg = int(parts[1]), float(parts[2])
                        idx = arm.id_to_index[jid]
                        joint = arm.joints[idx]
                        q_other = q[arm.id_to_index[joint.coupled_with]] \
                            if joint.coupled_with is not None else None
                        q_new = q[:]
                        q_new[idx] = joint.clamp(joint.q_from_servo(deg), q_other=q_other)
                        q_new = _reclamp_all(arm, q_new)
                        apply(q_new)
                        Logger.log("SIM", f"end-effector: {tuple(round(v, 4) for v in arm.fk(q))}")
                    elif cmd == "ik" and len(parts) == 4:
                        target = tuple(float(v) for v in parts[1:])
                        q_sol, ok = arm.ik(target, seed=q)
                        if ok:
                            apply(q_sol)
                            Logger.log("SIM", f"IK converged -> servo(deg) = "
                                              f"{[round(d, 1) for d in arm.q_to_servo_deg(q_sol)]}")
                        else:
                            Logger.log("SIM", f"IK did NOT converge for {target} (out of reach?)")
                    elif cmd == "home":
                        apply([0.0] * len(arm.joints))
                    elif cmd in ("quit", "q"):
                        break
                    else:
                        Logger.log("SIM", f"unrecognized command: {line!r}")
                except (ValueError, StopIteration, KeyError):
                    Logger.log("SIM", f"bad command: {line!r}")

            _draw_joint_axes(viewer, model, data)
            viewer.sync()
            time.sleep(1 / 60)

    Logger.log("SIM", "Viewer closed")


if __name__ == "__main__":
    main()
