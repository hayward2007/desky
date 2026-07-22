from dynamixel_sdk import *
from control_table import *
from dotenv import load_dotenv
from logger import Logger
import os
import time

# configure .env before running, refer README.md

class Controller :
    def __init__(self, device_name: str = None, baudrate: int = None, protocol_version: float = None) :
        Logger.log("CONTROLLER", "Initializing...")
        load_dotenv()
        
        self._device_name = device_name if device_name else '/dev/' + os.getenv("DEVICE_NAME")
        self._baudrate = baudrate if baudrate else int(os.getenv("BAUDRATE"))
        self._protocol_version = protocol_version if protocol_version else float(os.getenv("PROTOCOL_VERSION"))
        
        self.port_handler = None  # set before any open attempt so __del__/close is always safe
        self.packet_handler = PacketHandler(self._protocol_version)

        self.port_handler = PortHandler(self._device_name)

        if self.port_handler.openPort() :
            Logger.log("CONTROLLER", "Succeeded to open the port")
        else :
            raise Exception("[CONTROLLER] Failed to open the port")

        if self.port_handler.setBaudRate(self._baudrate) :
            Logger.log("CONTROLLER", "Succeeded to set the baudrate")
        else :
            raise Exception("[CONTROLLER] Failed to set the baudrate")

        time.sleep(0.1)  # Wait for the port to stabilize after opening and setting baudrate

    def close(self) :
        # Idempotent, exception-safe port teardown. Safe to call even if init failed partway.
        if getattr(self, "port_handler", None) is not None :
            try :
                self.port_handler.closePort()
                Logger.log("CONTROLLER", "Succeeded to close the port")
            except Exception as e :
                Logger.log("CONTROLLER", f"Error while closing the port: {e}")
            finally :
                self.port_handler = None

    def __del__(self) :
        self.close()

    # Support `with Controller() as c:` for deterministic cleanup instead of relying on GC.
    def __enter__(self) :
        return self

    def __exit__(self, exc_type, exc_value, traceback) :
        self.close()
        return False
        
        
    def _check_comm(self, id: int, action: str, comm_result: int, error: int) -> bool :
        # Returns True on success. On failure, logs a warning and returns False
        # instead of raising, so a single dropped packet never kills the run.
        ok = True
        if comm_result != COMM_SUCCESS :
            Logger.log("CONTROLLER", f"Actuator ID {id} [{action}] comm error: "
                       f"{self.packet_handler.getTxRxResult(comm_result)}")
            ok = False
        if error != 0 :
            Logger.log("CONTROLLER", f"Actuator ID {id} [{action}] hardware error: "
                       f"{self.packet_handler.getRxPacketError(error)}")
            ok = False
        return ok

    # input is percentage, 0 to 100
    def set_speed(self, id: int, speed: float, control_table: ActuatorControlTable) -> bool :
        if speed < 0 or speed > 100 :
            raise ValueError("[CONTROLLER] Speed must be between 0 and 100")
        speed_value = int(speed / 100 * control_table.Unit_Number)  # Convert percentage to unit value
        try :
            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, id, control_table.Address.Moving_Speed, speed_value)
        except Exception as e :
            Logger.log("CONTROLLER", f"Actuator ID {id} [SET SPEED] exception: {e}")
            return False
        return self._check_comm(id, "SET SPEED", comm_result, error)

    # input is degrees, 0 to 300, maximum range for AX-18A is 0 to 300 degrees
    def set_goal_position(self, id: int, position: float, control_table: ActuatorControlTable) -> bool :
        if position < 0 or position > 300 :
            raise ValueError("[CONTROLLER] Position must be between 0 and 300")
        position_value = int(position / 360 * control_table.Unit_Number)  # Convert degrees to unit value
        try :
            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, id, control_table.Address.Goal_Position, position_value)
        except Exception as e :
            Logger.log("CONTROLLER", f"Actuator ID {id} [SET POSITION] exception: {e}")
            return False
        return self._check_comm(id, "SET POSITION", comm_result, error)

    # output is degrees, 0 to 300, maximum range for AX-18A is 0 to 300 degrees
    # Returns position in degrees, or None if the read failed (caller must handle None).
    def get_present_position(self, id: int, control_table: ActuatorControlTable) :
        try :
            data, comm_result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, id, control_table.Address.Present_Position)
        except Exception as e :
            Logger.log("CONTROLLER", f"Actuator ID {id} [GET POSITION] exception: {e}")
            return None
        if not self._check_comm(id, "GET POSITION", comm_result, error) :
            return None
        position_degrees = data / control_table.Unit_Number * 360  # Convert unit value to degrees
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