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

    def goto(self, degree: float, speed: float = 10):
        Logger.log("ACTUATOR", f"ID {self.id} goto degree={degree} speed={speed}")
        self.controller.set_speed(self.id, speed, self.control_table)
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

    def goto_position(self, target_pos, target_rot=None, speed: float = 10, seed=None):
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