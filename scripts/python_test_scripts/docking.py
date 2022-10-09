#!/usr/bin/env python3
import os
import time

# Dynamixel APIs
from dynamixel_sdk import * # Uses Dynamixel SDK library

# Power Seonsor APIs
from power_sensor import PowerSensor

# QbDockingStation class
class QbDockingStation(object):  

    portHandler = None
    packetHandler = None

    # Control table address
    ADDR_MODE                   = 11
    ADDR_SHUTDOWN_REASON        = 63
    ADDR_TORQUE                 = 64
    ADDR_LED                    = 65
    ADDR_GOAL_POSITION          = 116
    ADDR_GOAL_PWM               = 100
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_LOAD           = 126
    DXL_MINIMUM_POSITION_VALUE  = 3000         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 21000      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    # Verbose switch
    VERBOSE                     = False

    EXT_POS_CONTROL             = 4     # Value for Extended Position Control Mode of the Operation Mode Register
    PWM_CONTROL                 = 16    # Value for PWM Control Mode of the Operation Mode Register
    LED_ENABLE                  = 1     # Value for enabling the LED
    LED_DISABLE                 = 0     # Value for disabling the LED
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 30    # Dynamixel moving status threshold
    PWM_GOAL_MIN                = 800   # 50% PWM goal
    PWM_GOAL_MAX                = -800  # -50% PWM goal
    LOAD_MIN                    = -520  # minimum load boundry
    LOAD_MAX                    = 520   # maximum load boundry

    # Direction params
    DIR_NORMAL                  = 0
    DIR_REVERSE                 = 1
    DIR_SETTING                 = DIR_NORMAL

    # Minimum encoder position
    CAL_MIN_BOUNDREY            = 0
    # Maximum encoder position
    CAL_MAX_BOUNDREY            = 0

    # Calibration State Definition
    ST_CAL_INIT                 = 0
    ST_CAL_MOVE_UP              = 1
    ST_CAL_CHECK_LOAD_MIN       = 2
    ST_CAL_STORE_MIN_BOUNDREY   = 3
    ST_CAL_MOVE_DOWN            = 4
    ST_CAL_CHECK_LOAD_MAX       = 5
    ST_CAL_STORE_MAX_BOUNDREY   = 6
    ST_CAL_END_RETRACT          = 7
    ST_CAL_DONE                 = 8
    ST_CAL_ERROR                = 9
    calibration_state = ST_CAL_DONE

    # Docking State Definition
    ST_DOC_INIT                 = 0
    ST_DOC_MOVE                 = 1
    ST_DOC_CHECK_CONNECTION     = 2
    ST_DOC_CONNECTED            = 3
    ST_DOC_END_RETRACT          = 4
    ST_DOC_FAILURE              = 5
    ST_DOC_ERROR                = 6
    docking_state = ST_DOC_INIT

    # Undocking State Definition
    ST_UNDOC_INIT                 = 0
    ST_UNDOC_MOVE                 = 1
    ST_UNDOC_CHECK_CONNECTION     = 2
    ST_UNDOC_DISCONNECTED         = 3
    ST_UNDOC_END_RETRACT          = 4
    ST_UNDOC_FAILURE              = 5
    ST_UNDOC_ERROR                = 6
    undocking_state = ST_UNDOC_INIT

    power_sensor = None

    # Init
    def __init__(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize packetHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1self.packetHandler or Protocol2self.packetHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            if(self.VERBOSE): print("[INFO][GEN] - Succeeded to open the port")
        else:
            print("[ERROR][GEN] - Failed to open the port")

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            if(self.VERBOSE): print("[INFO][GEN] - Succeeded to change the baudrate")
        else:
            print("[ERROR][GEN] - Failed to change the baudrate")

        # Reset the Dynamixel to clear any HW errors
        self.reboot()
        # Set Dynamixel direction
        self.set_direction(self.DIR_SETTING)

        if(self.VERBOSE): print("[INFO][GEN] Dynamixel has been successfully connected")
        
        # Create/init power sensor class
        self.power_sensor = PowerSensor()
        

    def disable(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
        # Disable Dynamixel LED
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_LED, self.LED_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))

    # Enable Dynamixel torq
    def enable(self):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
        # Enable Dynamixel LED
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_LED, self.LED_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))


    # Denit relay controller class
    def deinit(self):
        # Disable the motor
        self.disable()
        # Close port
        self.portHandler.closePort()
        if(self.VERBOSE): print("[INFO][GEN] Dynamixel has been successfully disconnected")


    # Reboot Dynamixel HW and reset all HW errors
    def reboot(self):
        # Disable Dynamixel 
        self.disable()
        if(self.VERBOSE): print("[INFO][GEN] - Rebooting Dynamixel...")
        # Reboot Dynamixel to clear all errors
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, self.DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            if(self.VERBOSE): print("[ERROR][INFO] - Dynamixel has been successfully rebooted")
        # Renable Dynamixel 
        time.sleep(0.5) # give some time for the servo to reboot
        if(self.VERBOSE): print("[INFO][GEN] - Rebooting Done.")
        self.enable()

    # Change motor control mode
    def change_control_mode(self, _control_mode):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("[ERROR][CAL] - %s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        # Change Control Mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MODE, _control_mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % (self.calibration_state, self.packetHandler.getTxRxResult(dxl_comm_result)))
            return False
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % (self.calibration_state, self.packetHandler.getRxPacketError(dxl_error)))
            return False

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True


    # Set Motor Direction
    def set_direction(self, _direction):
        # Read Operation Mode register
        dxl_operation_mode, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        # Set Normal/Reverse Mode bit
        if (_direction == self.DIR_NORMAL):
            dxl_operation_mode = (dxl_operation_mode & ~1)
            return True
        elif (_direction == self.DIR_REVERSE):
            dxl_operation_mode = (dxl_operation_mode | 1)
            return True
        else:
            return False


    def read_connection_status(self):
        connection_voltage = self.power_sensor.read_bus_voltage()
        if(self.VERBOSE): print("[INFO][GEN] Connection status - bus voltage: %d (mV)" % (connection_voltage))
        if(connection_voltage >= 5000):
            return True 
        else:
            return False


    # Calibration sequence
    def calibration_sequence(self):
        # Calibration State Machine
        if self.calibration_state == self.ST_CAL_INIT:
            # Set Control to PWM mode
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.calibration_state = self.ST_CAL_MOVE_UP
            else:
                self.calibration_state = self.ST_CAL_ERROR
                print("[ERROR][CAL][ST_CAL_INIT] - control mode change failed")

        elif self.calibration_state == self.ST_CAL_MOVE_UP:
            # Rotate motor CW
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_MOVE_UP")
            # Convert shigned short to signed int
            pwm_goal = pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_MOVE_UP] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_MOVE_UP] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            else:
                self.calibration_state = self.ST_CAL_CHECK_LOAD_MIN

        elif self.calibration_state == self.ST_CAL_CHECK_LOAD_MIN:
            # Check if the load is more than (+-)50%
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_CHECK_LOAD_MIN")
            dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
            # Convert shigned short to signed int
            dxl_present_load_signed = int.from_bytes((dxl_present_load).to_bytes(2, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_CHECK_LOAD_MIN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_CHECK_LOAD_MIN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            elif(dxl_present_load_signed >= self.LOAD_MIN and dxl_present_load_signed <= self.LOAD_MAX):
                self.calibration_state = self.ST_CAL_CHECK_LOAD_MIN
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_CHECK_LOAD_MIN] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.calibration_state = self.ST_CAL_STORE_MIN_BOUNDREY
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_CHECK_LOAD_MIN] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.calibration_state == self.ST_CAL_STORE_MIN_BOUNDREY:
            # If load detected, store current position as min boundrey
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_STORE_MIN_BOUNDREY")
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            dxl_present_position_signed = int.from_bytes((dxl_present_position).to_bytes(4, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_STORE_MIN_BOUNDREY] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_STORE_MIN_BOUNDREY] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            else:
                self.CAL_MIN_BOUNDREY = dxl_present_position_signed - 200
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_STORE_MIN_BOUNDREY] - CAL_MIN_BONDREY = %d" % (self.CAL_MIN_BOUNDREY))
                self.calibration_state = self.ST_CAL_MOVE_DOWN

        elif self.calibration_state == self.ST_CAL_MOVE_DOWN:
            # Reverse motor rotation (CCW)
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_MOVE_DOWN")
            # Convert shigned short to signed int
            pwm_goal = int.from_bytes((self.PWM_GOAL_MAX).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_MOVE_DOWN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_MOVE_DOWN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            else:
                self.calibration_state = self.ST_CAL_CHECK_LOAD_MAX
                time.sleep(0.5)

        elif self.calibration_state == self.ST_CAL_CHECK_LOAD_MAX:
            # Check if the load is more than (+-)50%
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_CHECK_LOAD_MAX")
            dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
            # Convert shigned short to signed int
            dxl_present_load_signed = int.from_bytes((dxl_present_load).to_bytes(2, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_CHECK_LOAD_MAX] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_CHECK_LOAD_MAX] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            elif(dxl_present_load_signed >= self.LOAD_MIN and dxl_present_load_signed <= self.LOAD_MAX):
                self.calibration_state = self.ST_CAL_CHECK_LOAD_MAX
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_CHECK_LOAD_MAX] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.calibration_state = self.ST_CAL_STORE_MAX_BOUNDREY
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_CHECK_LOAD_MAX] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.calibration_state == self.ST_CAL_STORE_MAX_BOUNDREY:
            # If load detected, store current position as max boundrey
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_STORE_MAX_BOUNDREY")
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            dxl_present_position_signed = int.from_bytes((dxl_present_position).to_bytes(4, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_STORE_MAX_BOUNDREY] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_STORE_MAX_BOUNDREY] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            else:
                self.CAL_MAX_BOUNDREY = dxl_present_position_signed - 200
                if(self.VERBOSE): print("[INFO][CAL][ST_CAL_STORE_MAX_BOUNDREY] - CAL_MAX_BOUNDREY = %d" % (self.CAL_MAX_BOUNDREY))
                self.calibration_state = self.ST_CAL_END_RETRACT

        elif self.calibration_state == self.ST_CAL_END_RETRACT:
            # Rotate motor CW
            if(self.VERBOSE): print("[INFO][CAL] ST_CAL_END_RETRACT")
            # Convert shigned short to signed int
            pwm_goal = pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][CAL][ST_CAL_END_RETRACT] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.calibration_state = self.ST_CAL_ERROR
            elif dxl_error != 0:
                print("[ERROR][CAL][ST_CAL_END_RETRACT] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.calibration_state = self.ST_CAL_ERROR
            else:
                # Delay with 1 second to give some time for the motor to move
                time.sleep(1)
                if(self.VERBOSE): print("[INFO][CAL] ST_CAL_DONE")
                self.calibration_state = self.ST_CAL_DONE

        elif self.calibration_state == self.ST_CAL_DONE:
            #if(self.VERBOSE): print("[INFO][CAL] ST_CAL_DONE")
            self.disable()

        elif self.calibration_state == self.ST_CAL_ERROR:
            #if(self.VERBOSE): print("[INFO][CAL] ST_CAL_ERROR");
            self.disable()

        else:
            # If unknown state, go to error state
            if(self.VERBOSE): print("[INFO][CAL] DEFAULT")
            self.calibration_state = self.ST_CAL_ERROR


    # Dock with robot
    def dock_connector(self):
        # Docking State Machine
        if self.docking_state == self.ST_DOC_INIT:
            # Set Control to PWM mode
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.docking_state = self.ST_DOC_MOVE
            else:
                self.docking_state = self.ST_DOC_ERROR
                print("[ERROR][DOC][ST_DOC_INIT] - control mode change failed")

        elif self.docking_state == self.ST_DOC_MOVE:
            # Reverse motor rotation (CCW)
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_MOVE")
            # Convert shigned short to signed int
            pwm_goal = int.from_bytes((self.PWM_GOAL_MAX).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][DOC][ST_DOC_MOVE] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.docking_state = self.ST_DOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][DOC][ST_DOC_MOVE] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.docking_state = self.ST_DOC_ERROR
            else:
                self.docking_state = self.ST_DOC_CHECK_CONNECTION

        elif self.docking_state == self.ST_DOC_CHECK_CONNECTION:
            # Read connection status
            connection_status = self.read_connection_status()
            # Check if the load is more than (+-)50%
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_CHECK_CONNECTION")
            dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
            # Convert shigned short to signed int
            dxl_present_load_signed = int.from_bytes((dxl_present_load).to_bytes(2, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][DOC][ST_DOC_CHECK_CONNECTION] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.docking_state = self.ST_DOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][DOC][ST_DOC_CHECK_CONNECTION] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.docking_state = self.ST_DOC_ERROR
            elif connection_status == True:
                self.docking_state = self.ST_DOC_CONNECTED
                if(self.VERBOSE): print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))
                if(self.VERBOSE): print("[INFO][DOC] ST_DOC_CONNECTED")
            elif(dxl_present_load_signed >= self.LOAD_MIN and dxl_present_load_signed <= self.LOAD_MAX):
                self.docking_state = self.ST_DOC_CHECK_CONNECTION
                if(self.VERBOSE): print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.docking_state = self.ST_DOC_END_RETRACT
                if(self.VERBOSE): print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.docking_state == self.ST_DOC_CONNECTED:
            #print("[INFO][DOC] ST_DOC_CONNECTED")
            self.disable()

        elif self.docking_state == self.ST_DOC_END_RETRACT:
            # Rotate motor CW
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_END_RETRACT")
            # Convert shigned short to signed int
            pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][DOC][ST_DOC_END_RETRACT] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.docking_state = self.ST_DOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][DOC][ST_DOC_END_RETRACT] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.docking_state = self.ST_DOC_ERROR
            else:
                # Delay with 1 second to give some time for the motor to move
                time.sleep(1)
                # Convert shigned short to signed int
                pwm_goal = int.from_bytes((0).to_bytes(2, byteorder='big', signed=True), byteorder='big')
                # Stop the servo. Set PWM to 0
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
                if(self.VERBOSE): print("[INFO][DOC] ST_DOC_FAILURE")
                self.docking_state = self.ST_DOC_FAILURE

        elif self.docking_state == self.ST_DOC_FAILURE:
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_FAILURE")
            self.disable()
            self.docking_state = self.ST_DOC_INIT

        elif self.docking_state == self.ST_DOC_ERROR:
            if(self.VERBOSE): print("[INFO][DOC] ST_DOC_ERROR")
            self.disable()
            self.docking_state = self.ST_DOC_INIT
        
        # return status
        return self.docking_state


    # Undock from the robot
    def undock_connector(self):
        # Undocking State Machine
        if self.undocking_state == self.ST_UNDOC_INIT:
            # Set Control to PWM mode
            if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.undocking_state = self.ST_UNDOC_MOVE
            else:
                self.undocking_state = self.ST_UNDOC_ERROR
                print("[ERROR][UNDOC][ST_UNDOC_INIT] - control mode change failed")

        elif self.undocking_state == self.ST_UNDOC_MOVE:
            # Reverse motor rotation (CW)
            if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_MOVE")
            # Convert shigned short to signed int
            pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][UNDOC][ST_UNDOC_MOVE] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.undocking_state = self.ST_UNDOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][UNDOC][ST_UNDOC_MOVE] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.undocking_state = self.ST_UNDOC_ERROR
            else:
                self.undocking_state = self.ST_UNDOC_CHECK_CONNECTION

        elif self.undocking_state == self.ST_UNDOC_CHECK_CONNECTION:
            # Read connection status
            connection_status = self.read_connection_status()
            # Check if the load is more than (+-)50%
            if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_CHECK_CONNECTION")
            dxl_present_load, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_LOAD)
            # Convert shigned short to signed int
            dxl_present_load_signed = int.from_bytes((dxl_present_load).to_bytes(2, byteorder='big'), signed=True, byteorder='big')
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][UNDOC][ST_UNDOC_CHECK_CONNECTION] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.undocking_state = self.ST_UNDOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][UNDOC][ST_UNDOC_CHECK_CONNECTION] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.undocking_state = self.ST_UNDOC_ERROR
            elif(dxl_present_load_signed >= self.LOAD_MIN and dxl_present_load_signed <= self.LOAD_MAX):
                self.undocking_state = self.ST_UNDOC_CHECK_CONNECTION
                if(self.VERBOSE): print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))
            elif connection_status == False:
                if(self.VERBOSE): print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))
                # Convert shigned short to signed int
                pwm_goal = int.from_bytes((self.PWM_GOAL_MAX).to_bytes(2, byteorder='big', signed=True), byteorder='big')
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
                if dxl_comm_result != COMM_SUCCESS:
                    print("[ERROR][UNDOC][ST_UNDOC_DISCONNECTED] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                    self.undocking_state = self.ST_UNDOC_ERROR
                elif dxl_error != 0:
                    print("[ERROR][UNDOC][ST_UNDOC_DISCONNECTED] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                    self.undocking_state = self.ST_UNDOC_ERROR
                else:
                    # Delay with 0.5 second to give some time for the motor to move
                    time.sleep(1)
                    # Convert shigned short to signed int
                    pwm_goal = int.from_bytes((0).to_bytes(2, byteorder='big', signed=True), byteorder='big')
                    # Stop the servo. Set PWM to 0
                    dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
                self.undocking_state = self.ST_UNDOC_DISCONNECTED
            else:
                self.undocking_state = self.ST_UNDOC_END_RETRACT
                if(self.VERBOSE): print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.undocking_state == self.ST_UNDOC_DISCONNECTED:
            if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_DISCONNECTED")
            self.disable()
            self.undocking_state = self.ST_UNDOC_INIT

        elif self.undocking_state == self.ST_UNDOC_END_RETRACT:
            # Rotate motor CCW
            if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_END_RETRACT")
            # Convert shigned short to signed int
            pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
            if dxl_comm_result != COMM_SUCCESS:
                print("[ERROR][UNDOC][ST_UNDOC_END_RETRACT] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                self.undocking_state = self.ST_UNDOC_ERROR
            elif dxl_error != 0:
                print("[ERROR][UNDOC][ST_UNDOC_END_RETRACT] - %s" % self.packetHandler.getRxPacketError(dxl_error))
                self.undocking_state = self.ST_UNDOC_ERROR
            else:
                # Delay with 0.5 second to give some time for the motor to move
                time.sleep(0.5)
                # Convert shigned short to signed int
                pwm_goal = int.from_bytes((0).to_bytes(2, byteorder='big', signed=True), byteorder='big')
                # Stop the servo. Set PWM to 0
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_PWM, pwm_goal)
                if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_FAILURE")
                self.undocking_state = self.ST_UNDOC_FAILURE

        elif self.undocking_state == self.ST_UNDOC_FAILURE:
            #if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_FAILURE")
            self.disable()

        elif self.undocking_state == self.ST_UNDOC_ERROR:
            #if(self.VERBOSE): print("[INFO][UNDOC] ST_UNDOC_ERROR")
            self.disable()
        
        # return status
        return self.undocking_state
    


    # Main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        while (1):
            self.dock_connector()
            time.sleep(0.01)
    

    #shutdown hook
    def terminate(self):
        pass



# Main thread
def start_manager():
    global qb_docking_station
    # Init and start the qB class
    qb_docking_station = QbDockingStation()
    qb_docking_station.calibration_state = qb_docking_station.ST_CAL_INIT
    qb_docking_station.main_loop()
    
# Main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except KeyboardInterrupt:
        # do nothing here
        qb_docking_station.deinit()
