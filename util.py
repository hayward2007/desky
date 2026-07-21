from control_table import AX_18A
from controller import Controller

import time

class Actuator:
    __TIME_INTERVAL = 0.025

    __MIN_SPEED = 1

    def __init__(self, id: int, model: str, controller: Controller):
        self.id = id
        self.controller = controller

        if model == "AX-18A":
            self.control_table = AX_18A

        self.controller.set_speed(self.id, self.__MIN_SPEED, self.control_table)

    def goto(self, degree: float, speed: float = 10):
        self.controller.set_speed(self.id, speed, self.control_table)
        time.sleep(self.__TIME_INTERVAL)
        self.controller.set_goal_position(self.id, degree, self.control_table)

    def get_position(self):
        return self.controller.get_present_position(self.id, self.control_table)