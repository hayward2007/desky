# 0	2	Model Number	Model Number	R	18
# 2	1	Firmware Version	Firmware Version	R	-
# 3	1	ID	DYNAMIXEL ID	RW	1
# 4	1	Baud Rate	Communication Speed	RW	1
# 5	1	Return Delay Time	Response Delay Time	RW	250
# 6	2	CW Angle Limit	Clockwise Angle Limit	RW	0
# 8	2	CCW Angle Limit	Counter-Clockwise Angle Limit	RW	1023
# 11	1	Temperature Limit	Maximum Internal Temperature Limit	RW	75
# 12	1	Min Voltage Limit	Minimum Input Voltage Limit	RW	60
# 13	1	Max Voltage Limit	Maximum Input Voltage Limit	RW	140
# 14	2	Max Torque	Maximum Torque	RW	983
# 16	1	Status Return Level	Select Types of Status Return	RW	2
# 17	1	Alarm LED	LED for Alarm	RW	36
# 18	1	Shutdown	Shutdown Error Information	RW	36
# 24	1	Torque Enable	Motor Torque On/Off	RW	0
# 25	1	LED	Status LED On/Off	RW	0
# 26	1	CW Compliance Margin	CW Compliance Margin	RW	1
# 27	1	CCW Compliance Margin	CCW Compliance Margin	RW	1
# 28	1	CW Compliance Slope	CW Compliance Slope	RW	32
# 29	1	CCW Compliance Slope	CCW Compliance Slope	RW	32
# 30	2	Goal Position	Target Position	RW	-
# 32	2	Moving Speed	Moving Speed	RW	-
# 34	2	Torque Limit	Torque Limit	RW	Max Torque
# 36	2	Present Position	Present Position	R	-
# 38	2	Present Speed	Present Speed	R	-
# 40	2	Present Load	Present Load	R	-
# 42	1	Present Voltage	Present Voltage	R	-
# 43	1	Present Temperature	Present Temperature	R	-
# 44	1	Registered	If Instruction is registered	R	0
# 46	1	Moving	Movement Status	R	0
# 47	1	Lock	Locking EEPROM	RW	0
# 48	2	Punch	Minimum Current Threshold	RW	32

class ActuatorControlTable :
    Unit_Number: int
    Protocol_Version: float
    
    class Address :
        Model_Number: int
        Firmware_Version: int
        ID: int
        Baud_Rate: int
        Return_Delay_Time: int
        CW_Angle_Limit: int
        CCW_Angle_Limit: int
        Temperature_Limit: int
        Min_Voltage_Limit: int
        Max_Voltage_Limit: int
        Max_Torque: int
        Status_Return_Level: int
        Alarm_LED: int
        Shutdown: int
        Torque_Enable: int
        LED: int
        CW_Compliance_Margin: int
        CCW_Compliance_Margin: int
        CW_Compliance_Slope: int
        CCW_Compliance_Slope: int
        Goal_Position: int
        Moving_Speed: int
        Torque_Limit: int
        Present_Position: int
        Present_Speed: int
        Present_Load: int
        Present_Voltage: int
        Present_Temperature: int
        Registered: int
        
    class Size :
        Model_Number: int
        Firmware_Version: int
        ID: int
        Baud_Rate: int
        Return_Delay_Time: int
        CW_Angle_Limit: int
        CCW_Angle_Limit: int
        Temperature_Limit: int
        Min_Voltage_Limit: int
        Max_Voltage_Limit: int
        Max_Torque: int
        Status_Return_Level: int
        Alarm_LED: int
        Shutdown: int
        Torque_Enable: int
        LED: int
        CW_Compliance_Margin: int
        CCW_Compliance_Margin: int
        CW_Compliance_Slope: int
        CCW_Compliance_Slope: int
        Goal_Position: int
        Moving_Speed: int
        Torque_Limit: int
        Present_Position: int
        Present_Speed: int
        Present_Load: int
        Present_Voltage: int
        Present_Temperature: int
        Registered: int


class AX_18A(ActuatorControlTable) :
    Unit_Number = 1023
    Protocol_Version = 1.0
    
    class Address :
        Model_Number = 0
        Firmware_Version = 2
        ID = 3
        Baud_Rate = 4
        Return_Delay_Time = 5
        CW_Angle_Limit = 6
        CCW_Angle_Limit = 8
        Temperature_Limit = 11
        Min_Voltage_Limit = 12
        Max_Voltage_Limit = 13
        Max_Torque = 14
        Status_Return_Level = 16
        Alarm_LED = 17
        Shutdown = 18
        Torque_Enable = 24
        LED = 25
        CW_Compliance_Margin = 26
        CCW_Compliance_Margin = 27
        CW_Compliance_Slope = 28
        CCW_Compliance_Slope = 29
        Goal_Position = 30
        Moving_Speed = 32
        Torque_Limit = 34
        Present_Position = 36
        Present_Speed = 38
        Present_Load = 40
        Present_Voltage = 42
        Present_Temperature = 43
        Registered = 44
        Moving = 46
        Lock = 47
        Punch = 48
        
    class Size :
        Model_Number = 2
        Firmware_Version = 1
        ID = 1
        Baud_Rate = 1
        Return_Delay_Time = 1
        CW_Angle_Limit = 2
        CCW_Angle_Limit = 2
        Temperature_Limit = 1
        Min_Voltage_Limit = 1
        Max_Voltage_Limit = 1
        Max_Torque = 2
        Status_Return_Level = 1
        Alarm_LED = 1
        Shutdown = 1
        Torque_Enable = 1
        LED = 1
        CW_Compliance_Margin = 1
        CCW_Compliance_Margin = 1
        CW_Compliance_Slope = 1
        CCW_Compliance_Slope = 1
        Goal_Position = 2
        Moving_Speed = 2
        Torque_Limit = 2
        Present_Position = 2
        Present_Speed = 2
        Present_Load = 2
        Present_Voltage = 1
        Present_Temperature = 1
        Registered = 1
        Moving = 1
        Lock = 1
        Punch = 2