
# import tensorflow as tf
# import asyncio
import math
import time
import os

from controller import Controller
from util import Actuator

controller = Controller()


actuator1 = Actuator(id=1, model="AX-18A", controller=controller)
actuator2 = Actuator(id=2, model="AX-18A", controller=controller)
actuator3 = Actuator(id=3, model="AX-18A", controller=controller)
actuator4 = Actuator(id=4, model="AX-18A", controller=controller)
actuator5 = Actuator(id=5, model="AX-18A", controller=controller)

actuator1.goto(180)
actuator2.goto(180) 
actuator3.goto(180)
actuator4.goto(180)
actuator5.goto(180)

time.sleep(2)

actuator3.goto(170)
actuator4.goto(220)
actuator5.goto(280)

time.sleep(5)