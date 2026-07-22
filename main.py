
# import tensorflow as tf
# import asyncio
import math
import time
import os

from controller import Controller
from util import Actuator
from logger import Logger

Logger.enabled = True  # attach logging for this run; set False to silence

controller = Controller()


actuator1 = Actuator(id=1, model="AX-18A", controller=controller)
actuator2 = Actuator(id=2, model="AX-18A", controller=controller)
actuator3 = Actuator(id=3, model="AX-18A", controller=controller)
actuator4 = Actuator(id=4, model="AX-18A", controller=controller)
actuator5 = Actuator(id=5, model="AX-18A", controller=controller)

Logger.log("MAIN", "Moving all actuators to 180 deg")
actuator1.goto(180)
actuator2.goto(180)
actuator3.goto(180)
actuator4.goto(180)
actuator5.goto(180)

time.sleep(2)

Logger.log("MAIN", "Moving actuators 3-5 to pose 2")
actuator3.goto(170)
actuator4.goto(220)
actuator5.goto(280)

time.sleep(5)