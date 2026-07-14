from dynamixel_sdk import *
from control_table import *
from dotenv import load_dotenv
import os
import time

# configure .env before running, refer README.md

class Controller :
    def __init__(self, device_name: str = None, baudrate: int = None, protocol_version: float = None) :
        print("[CONTROLLER] Initializing...")
        load_dotenv()
        
        self._device_name = device_name if device_name else '/dev/' + os.getenv("DEVICE_NAME")
        self._baudrate = baudrate if baudrate else int(os.getenv("BAUDRATE"))
        self._protocol_version = protocol_version if protocol_version else float(os.getenv("PROTOCOL_VERSION"))
        
        self.port_handler = PortHandler(self._device_name)
        self.packet_handler = PacketHandler(self._protocol_version)

        if self.port_handler.openPort() :
            print("[CONTROLLER] Succeeded to open the port")
        else :
            raise Exception("[CONTROLLER] Failed to open the port")

        if self.port_handler.setBaudRate(self._baudrate) :
            print("[CONTROLLER] Succeeded to set the baudrate")
        else :
            raise Exception("[CONTROLLER] Failed to set the baudrate")
        
        time.sleep(0.1)  # Wait for the port to stabilize after opening and setting baudrate

    def __del__(self) :
        self.port_handler.closePort()
        print("[CONTROLLER] Succeeded to close the port")
        
        
    # input is percentage, 0 to 100
    def set_speed(self, id: int, speed: float, control_table: ActuatorControlTable) :
        if speed < 0 or speed > 100 :
            raise ValueError("[CONTROLLER] Speed must be between 0 and 100")
        speed_value = int(speed / 100 * control_table.Unit_Number)  # Convert percentage to
        return self.packet_handler.write2ByteTxRx(self.port_handler, id, control_table.Address.Moving_Speed, speed_value)
        
        
    # input is degrees, 0 to 300, maximum range for AX-18A is 0 to 300 degrees
    def set_goal_position(self, id: int, position: float, control_table: ActuatorControlTable) :
        if position < 0 or position > 300 :
            raise ValueError("[CONTROLLER] Position must be between 0 and 300")
        position_value = int(position / 360 * control_table.Unit_Number)  # Convert degrees to unit value
        return self.packet_handler.write2ByteTxRx(self.port_handler, id, control_table.Address.Goal_Position, position_value)
        
        
    # output is degrees, 0 to 300, maximum range for AX-18A is 0 to 300 degrees
    def get_present_position(self, id: int, control_table: ActuatorControlTable) :
        result, error, _ = self.packet_handler.read2ByteTxRx(self.port_handler, id, control_table.Address.Present_Position)
        if error != 0:
            raise Exception(f"[CONTROLLER] Error reading present position: {error}")
        position_degrees = result / control_table.Unit_Number * 360  # Convert unit value to degrees
        return position_degrees

    # def set_mode(self, id: int, mode: int) :
    #     if id in Actuator.lower_index :
    #         self.set_torque(id, Actuator.torque.off);
    #         result = self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.address.operating_mode, mode);
    #         print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Mode set to {mode}".ljust(35, " "));
    #         self.set_torque(id, Actuator.torque.on);
    
    # def set_all_mode(self, mode: int) :
    #     for i in Actuator.lower_index :
    #         self.set_mode(i, mode);
    
    # def get_mode(self, id: int) :
    #     print(self.packet_handler_2.read1ByteTxRx(self.port_handler, id, AX_18A.Torque_Enable));

    # def set_speed(self, id: int, speed: int, address: int = Actuator.model.XM.address.profile_velocity) :
    #     if self.__is_MX(id) :
    #         self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.moving_speed, speed);
    #     else :
    #         self.packet_handler_2.write4ByteTxRx(self.port_handler, id, address, speed);
    #     print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Speed set to {speed}".ljust(35, " "));
    
    # def set_all_speed(self, speed: int) :
    #     for i in Actuator.index :
    #         self.set_speed(i, speed);

    # def set_acceleration(self, id: int, acceleration: int) :
    #     if not self.__is_MX(id) :
    #         self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.profile_acceleration, acceleration);
    #         print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Acceleration set to {acceleration}".ljust(35, " "));

    # def set_torque(self, id: int, torque: int) :
    #     if self.__is_MX(id) :
    #         self.packet_handler_1.write1ByteTxRx(self.port_handler, id, Actuator.model.MX.address.enable_torque, torque);
    #     else :
    #         self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.address.torque_enable, torque);
    #     print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Torque turned {'on' if torque == 1 else 'off'}".ljust(35, " "));
    
    # def set_all_torque(self, torque: int) :
    #     for i in Actuator.index :
    #         self.set_torque(i, torque);
    
    # def enable_torque(self) :
    #     self.set_all_torque(Actuator.torque.on);
    
    # def disable_torque(self) :
    #     self.set_all_torque(Actuator.torque.off);

    # def set_position(self, id: int, position) :
    #     position = int(position / 360 * 4096 if id % 2 == 1 else 4096 - ( position / 360 * 4096 ));
    #     if self.__is_MX(id) :
    #         self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.goal_position, position);
    #     else :
    #         self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.goal_position, position);
    #     print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Position set to {position} degrees".ljust(35, " "));

    # def set_raw_position(self, id: int, position: int) :
    #     if self.__is_MX(id) :
    #         self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.goal_position, position);
    #     else :
    #         self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.goal_position, position);
    #     print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[SET] Position set to {position}".ljust(35, " "));

    # def get_position(self, id: int) :
    #     if self.__is_MX(id) :
    #         result, error, _ = self.packet_handler_1.read2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.present_position);
    #     else :
    #         result, error, _ = self.packet_handler_2.read4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.present_position);
    #     print(f"[CONTROLLER] Actuator ID : {id}".ljust(35, " ") + f"[GET] Current Position: {result}".ljust(35, " "));
    #     return result;