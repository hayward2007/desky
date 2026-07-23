from .control_table import *
from .controller import *
from logger import Logger
from kinematics.urdf_loader import load_arm

import time

class Actuator:
    __TIME_INTERVAL = 0.025

    __MIN_SPEED = 1

    def __init__(self, id: int, model: str, controller: Controller):
        self.id = id
        self.controller = controller

        if model == "AX-18A":
            self.control_table = AX_18A

        Logger.log("ACTUATOR", f"ID {self.id} initialized (model={model})")
        self.controller.set_speed(self.id, self.__MIN_SPEED, self.control_table)
        self._last_speed = self.__MIN_SPEED

    def goto(self, degree: float, speed: float = 5):
        Logger.log("ACTUATOR", f"ID {self.id} goto degree={degree} speed={speed}")
        # Moving_Speed is only re-written when it actually changes. Every
        # caller in this codebase drives with the same default speed, so
        # without this check goto() was writing Moving_Speed on every single
        # call (one extra TxRx round trip per joint, plus a mandatory 25ms
        # settle sleep) purely to rewrite the value already sitting in the
        # register — doubling serial traffic per move and, when a follower
        # commands a new position on every processed camera frame, adding up
        # to a blocking delay per frame large enough to stall the display
        # loop. Skipping the redundant write also means one less chance for
        # a corrupted packet to land a bad value in Moving_Speed (this
        # hardware does intermittently report bad status packets).
        if speed != self._last_speed:
            self.controller.set_speed(self.id, speed, self.control_table)
            self._last_speed = speed
            time.sleep(self.__TIME_INTERVAL)
        self.controller.set_goal_position(self.id, degree, self.control_table)

    def get_position(self):
        position = self.controller.get_present_position(self.id, self.control_table)
        Logger.log("ACTUATOR", f"ID {self.id} position={position}")
        return position


class ArmController:
    """Links kinematics.Arm (FK/IK) to a set of hardware Actuators.

    Actuators are matched to joints by `Actuator.id` == the joint's DYNAMIXEL
    id (from the URDF), so they can be passed in any order.
    """

    def __init__(self, actuators, arm=None):
        self.arm = arm if arm is not None else load_arm()

        by_id = {actuator.id: actuator for actuator in actuators}
        missing = [joint.id for joint in self.arm.joints if joint.id not in by_id]
        if missing:
            raise ValueError(f"Missing actuators for joint id(s): {missing}")
        self.actuators = [by_id[joint.id] for joint in self.arm.joints]

    def goto_position(self, target_pos, target_rot=None, speed: float = 5, seed=None):
        """Solve IK for `target_pos` and drive every actuator to the result.

        Returns (q, converged) from kinematics.Arm.ik. Leaves the actuators
        untouched if IK fails to converge.
        """
        q, converged = self.arm.ik(target_pos, target_rot=target_rot, seed=seed)
        if not converged:
            Logger.log("ARM", f"IK did not converge for target={target_pos}")
            return q, converged

        servo_degs = self.arm.q_to_servo_deg(q)
        Logger.log("ARM", f"IK converged for target={target_pos} -> "
                   f"servo_deg={[round(d, 1) for d in servo_degs]}")
        for actuator, deg in zip(self.actuators, servo_degs):
            actuator.goto(deg, speed=speed)
        return q, converged

    def goto_joints(self, servo_degs, speed: float = 5):
        """FK-style control: command EVERY actuator to a given servo angle and
        return the resulting end-effector position via forward kinematics.

        servo_degs: list of servo angles (deg, 0..300) in self.arm.joints order,
        or a dict {dynamixel_id: deg}. Unlike goto_position (which runs IK), this
        sets the joints directly and reports where the tool ends up (FK).
        """
        if isinstance(servo_degs, dict):
            servo_degs = [servo_degs[joint.id] for joint in self.arm.joints]
        if len(servo_degs) != len(self.actuators):
            raise ValueError(f"expected {len(self.actuators)} servo angles, got {len(servo_degs)}")

        for actuator, deg in zip(self.actuators, servo_degs):
            actuator.goto(deg, speed=speed)

        q = self.arm.servo_deg_to_q(servo_degs)
        pos = self.arm.fk(q)
        Logger.log("ARM", f"goto_joints servo_deg={[round(d, 1) for d in servo_degs]} -> "
                   f"FK {tuple(round(v, 4) for v in pos)}")
        return pos

    def get_position(self):
        """Read back every actuator's servo angle and return the FK position.

        Returns None if any actuator fails to report its position.
        """
        servo_degs = [actuator.get_position() for actuator in self.actuators]
        if any(deg is None for deg in servo_degs):
            Logger.log("ARM", "get_position failed: one or more actuators returned None")
            return None

        q = self.arm.servo_deg_to_q(servo_degs)
        pos = self.arm.fk(q)
        Logger.log("ARM", f"FK position={tuple(round(v, 4) for v in pos)}")
        return pos