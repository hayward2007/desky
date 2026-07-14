from control_table import AX_18A
from controller import Controller

import time

class Actuator:
    __K_P = 0.2
    __K_D = 0.05
    __TIME_INTERVAL = 0.025
    
    __MIN_SPEED = 1
    __MAX_SPEED = 75
    
    
    def __init__(self, id: int, model: str, controller: Controller):
        self.id = id
        self.controller = controller
        
        if model == "AX-18A":
            self.control_table = AX_18A
            
        self.controller.set_speed(self.id, self.__MIN_SPEED, self.control_table) 
            
            
    
    # def initialize_variables(self):
    #     self.present_position = self.controller.get_present_position(self.id, self.control_table)
    #     self.error = 0
    #     self.previous_error = 0
    #     self.moving_time = 0
    #     self.derivative = 0
        
        
        
    def goto_pd(self, degree: float):
        self.controller.set_goal_position(self.id, degree, self.control_table)
    
        present_position = self.controller.get_present_position(self.id, self.control_table)
        error = abs(degree - present_position)
        previous_error = error
        
        moving_time = 0
        derivative = 0
        
        while error >= 1:
            present_position = self.controller.get_present_position(self.id, self.control_table)
            time.sleep(self.__TIME_INTERVAL)
            moving_time += self.__TIME_INTERVAL
            
            error = abs(degree - present_position)
            derivative = (error - previous_error) / (self.__TIME_INTERVAL * 2)
            previous_error = error
            
            P_out = self.__K_P * error * min(1, (4 ** (moving_time - 1)))
            D_out = self.__K_D * derivative
            
            output = P_out + D_out
            
            speed_percentage = min(max(output, self.__MIN_SPEED), self.__MAX_SPEED)  # Clamp speed between __MIN_SPEED and __MAX_SPEED
            
            print(f"Present Position: {present_position} degrees, Speed Output: {output}%, Error: {error}, P: {P_out}, D: {D_out}, Moving Time: {moving_time}")
            
            self.controller.set_speed(self.id, speed_percentage, self.control_table)
            time.sleep(self.__TIME_INTERVAL)
            moving_time += self.__TIME_INTERVAL
        

    def goto(self, degree: float):
        self.controller.set_speed(self.id, 10, self.control_table)
        time.sleep(self.__TIME_INTERVAL)
        self.controller.set_goal_position(self.id, degree, self.control_table)