
# import tensorflow as tf
# import asyncio
import math
import time
import os

from hardware.controller import Controller
from hardware.util import Actuator
from logger import Logger
from scenario import DEMO_SEQUENCE

Logger.enabled = True  # attach logging for this run; set False to silence

controller = Controller()

actuators = {i: Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)}

for step in DEMO_SEQUENCE:
    Logger.log("MAIN", f"Commanding positions {step['positions']}")
    for servo_id, degree in step["positions"].items():
        actuators[servo_id].goto(degree)
    time.sleep(step["hold"])