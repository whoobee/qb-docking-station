#!/usr/bin/env python3


# Generic includes
import os
import time
import traceback

# ROS includes

import queue
import actionlib
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
# import custome msgs and srvs
from qb_docking_station.msg import QbDockingStationAction, QbDockingStationFeedback, QbDockingStationResult

# Dynamixel APIs
from dynamixel_sdk import * # Uses Dynamixel SDK library
# Power Seonsor APIs
from power_sensor import PowerSensor


#ROS logger class
class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

#ROS parameter parse
def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val


# QbDockingStation class
class QbDockingStation(object):  

    #execution rate
    main_rate = 10
    #ros pub freq
    qb_docking_station_calc_hz = 2

    portHandler = None
    packetHandler = None

    # Control table address
    ADDR_MODE                   = 11
    ADDR_SHUTDOWN_REASON        = 63
    ADDR_TORQUE_ENABLE          = 64
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

    # Docking State Definition
    ST_UNDOC_INIT                 = 0
    ST_UNDOC_MOVE                 = 1
    ST_UNDOC_CHECK_CONNECTION     = 2
    ST_UNDOC_DISCONNECTED         = 3
    ST_UNDOC_END_RETRACT          = 4
    ST_UNDOC_FAILURE              = 5
    ST_UNDOC_ERROR                = 6
    undocking_state = ST_UNDOC_INIT

    # Power Seonsor handler
    power_sensor = None


    # Init
    def __init__(self):

        #read params
        self.qb_doking_dxl_id = get_param('~qb_doking_dxl_id', 1)
        self.qb_doking_dxl_serial = get_param('~qb_doking_dxl_serial', "/dev/ttyUSB0")
        self.qb_doking_dxl_baud = get_param('~qb_doking_dxl_baud', 57600)
        self.qb_doking_dxl_pwm_treshold = get_param('~qb_doking_dxl_pwm_treshold', 800)
        self.qb_doking_dxl_load_treshold = get_param('~qb_doking_dxl_load_treshold', 520)
        self.qb_doking_verbose = get_param('~qb_doking_verbose', False)
        # set parameters
        self.DXL_ID = self.qb_doking_dxl_id
        self.DEVICENAME = self.qb_doking_dxl_serial
        self.BAUDRATE = self.qb_doking_dxl_baud
        self.PWM_GOAL_MIN = self.qb_doking_dxl_pwm_treshold
        self.PWM_GOAL_MAX = -self.qb_doking_dxl_pwm_treshold
        self.LOAD_MIN = self.qb_doking_dxl_load_treshold
        self.LOAD_MAX = -self.qb_doking_dxl_load_treshold
        self.VERBOSE = self.qb_doking_verbose

        #setup shutdown hook
        rospy.on_shutdown(self.terminate)

        #setup action server
        self.qb_docking_station_server = actionlib.SimpleActionServer('qb_docking_station_server', QbDockingStationAction, execute_cb=self.qb_docking_station_server_hndl, auto_start=False)
        self.qb_docking_station_server.start()

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
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

        # Reboot Dynamixel to clear all errors
        #dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, self.DXL_ID)
        #if dxl_comm_result != COMM_SUCCESS:
        #    print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
        #    print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))

        # Set Dynamixel Control Mode to Extended Position Control Mode
        #dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_MODE, self.EXT_POS_CONTROL)
        #if dxl_comm_result != COMM_SUCCESS:
        #    print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
        #    print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
        #else:
        #    print("Dynamixel has been successfully connected")
        
        

        # Enable Dynamixel LED
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_LED, self.LED_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[INFO][GEN] Dynamixel has been successfully connected")

        # Create/init power sensor class
        self.power_sensor = PowerSensor()

    # docking station server callback
    def qb_docking_station_server_hndl(self, goal):
        if(goal == self.EXTEND_CONENCTOR):
            pass
        elif(goal == self.RETRACT_CONNECTOR):
            pass
        elif(goal == self.ENGAGE_POWER):
            pass
        elif(goal == self.DISENGAGE_POWER):
            pass
        else:
            pass

    def disable(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
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


    def deinit(self):
        # Disable the motor
        self.disable()
        # Close port
        self.portHandler.closePort()
        print("[INFO][GEN] Dynamixel has been successfully disconnected")


    def change_control_mode(self, _control_mode):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
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
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("[ERROR][GEN] - %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("[ERROR][GEN] - %s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True


    # Set Direction
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
        print("[INFO][GEN] Connection status - bus voltage: %d (mV)" % (connection_voltage))
        if(connection_voltage >= 5000):
            return True 
        else:
            return False

    # Calibration sequence
    def calibration_sequence(self):
        # Calibration State Machine
        if self.calibration_state == self.ST_CAL_INIT:
            # Set Control to PWM mode
            print("[INFO][CAL] ST_CAL_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.calibration_state = self.ST_CAL_MOVE_UP
            else:
                self.calibration_state = self.ST_CAL_ERROR
                print("[ERROR][CAL][ST_CAL_INIT] - control mode change failed")

        elif self.calibration_state == self.ST_CAL_MOVE_UP:
            # Rotate motor CW
            print("[INFO][CAL] ST_CAL_MOVE_UP")
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
            print("[INFO][CAL] ST_CAL_CHECK_LOAD_MIN")
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
                print("[INFO][CAL][ST_CAL_CHECK_LOAD_MIN] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.calibration_state = self.ST_CAL_STORE_MIN_BOUNDREY
                print("[INFO][CAL][ST_CAL_CHECK_LOAD_MIN] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.calibration_state == self.ST_CAL_STORE_MIN_BOUNDREY:
            # If load detected, store current position as min boundrey
            print("[INFO][CAL] ST_CAL_STORE_MIN_BOUNDREY")
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
                print("[INFO][CAL][ST_CAL_STORE_MIN_BOUNDREY] - CAL_MIN_BONDREY = %d" % (self.CAL_MIN_BOUNDREY))
                self.calibration_state = self.ST_CAL_MOVE_DOWN

        elif self.calibration_state == self.ST_CAL_MOVE_DOWN:
            # Reverse motor rotation (CCW)
            print("[INFO][CAL] ST_CAL_MOVE_DOWN")
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
            print("[INFO][CAL] ST_CAL_CHECK_LOAD_MAX")
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
                print("[INFO][CAL][ST_CAL_CHECK_LOAD_MAX] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.calibration_state = self.ST_CAL_STORE_MAX_BOUNDREY
                print("[INFO][CAL][ST_CAL_CHECK_LOAD_MAX] - CAL_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.calibration_state == self.ST_CAL_STORE_MAX_BOUNDREY:
            # If load detected, store current position as max boundrey
            print("[INFO][CAL] ST_CAL_STORE_MAX_BOUNDREY")
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
                print("[INFO][CAL][ST_CAL_STORE_MAX_BOUNDREY] - CAL_MAX_BOUNDREY = %d" % (self.CAL_MAX_BOUNDREY))
                self.calibration_state = self.ST_CAL_END_RETRACT

        elif self.calibration_state == self.ST_CAL_END_RETRACT:
            # Rotate motor CW
            print("[INFO][CAL] ST_CAL_END_RETRACT")
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
                print("[INFO][CAL] ST_CAL_DONE")
                self.calibration_state = self.ST_CAL_DONE

        elif self.calibration_state == self.ST_CAL_DONE:
            #print("[INFO][CAL] ST_CAL_DONE")
            self.disable()

        elif self.calibration_state == self.ST_CAL_ERROR:
            #print("[INFO][CAL] ST_CAL_ERROR");
            self.disable()

        else:
            # If unknown state, go to error state
            print("[INFO][CAL] DEFAULT")
            self.calibration_state = self.ST_CAL_ERROR
        
    # Dock with robot
    def dock_connector(self):
        # Docking State Machine
        if self.docking_state == self.ST_DOC_INIT:
            # Set Control to PWM mode
            print("[INFO][DOC] ST_DOC_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.docking_state = self.ST_DOC_MOVE
            else:
                self.docking_state = self.ST_DOC_ERROR
                print("[ERROR][DOC][ST_DOC_INIT] - control mode change failed")

        elif self.docking_state == self.ST_DOC_MOVE:
            # Reverse motor rotation (CCW)
            print("[INFO][DOC] ST_DOC_MOVE")
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
            print("[INFO][DOC] ST_DOC_CHECK_CONNECTION")
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
                print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))
                print("[INFO][DOC] ST_DOC_CONNECTED")
            elif(dxl_present_load_signed >= self.LOAD_MIN and dxl_present_load_signed <= self.LOAD_MAX):
                self.docking_state = self.ST_DOC_CHECK_CONNECTION
                print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))
            else:
                self.docking_state = self.ST_DOC_END_RETRACT
                print("[INFO][DOC][ST_DOC_CHECK_CONNECTION] - DOC_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.docking_state == self.ST_DOC_CONNECTED:
            #print("[INFO][DOC] ST_DOC_CONNECTED")
            self.disable()

        elif self.docking_state == self.ST_DOC_END_RETRACT:
            # Rotate motor CW
            print("[INFO][DOC] ST_DOC_END_RETRACT")
            # Convert shigned short to signed int
            pwm_goal = pwm_goal = int.from_bytes((self.PWM_GOAL_MIN).to_bytes(2, byteorder='big', signed=True), byteorder='big')
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
                print("[INFO][DOC] ST_DOC_FAILURE")
                self.docking_state = self.ST_DOC_FAILURE

        elif self.docking_state == self.ST_DOC_FAILURE:
            #print("[INFO][DOC] ST_DOC_FAILURE")
            self.disable()

        elif self.docking_state == self.ST_DOC_ERROR:
            #print("[INFO][DOC] ST_DOC_ERROR")
            self.disable()


    # Undock from the robot
    def undock_connector(self):
        # Undocking State Machine
        if self.undocking_state == self.ST_UNDOC_INIT:
            # Set Control to PWM mode
            print("[INFO][UNDOC] ST_UNDOC_INIT")
            if self.change_control_mode(self.PWM_CONTROL) == True:
                self.undocking_state = self.ST_UNDOC_MOVE
            else:
                self.undocking_state = self.ST_UNDOC_ERROR
                print("[ERROR][UNDOC][ST_UNDOC_INIT] - control mode change failed")

        elif self.undocking_state == self.ST_UNDOC_MOVE:
            # Reverse motor rotation (CW)
            print("[INFO][UNDOC] ST_UNDOC_MOVE")
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
            print("[INFO][UNDOC] ST_UNDOC_CHECK_CONNECTION")
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
                print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))
            elif connection_status == False:
                self.undocking_state = self.ST_UNDOC_DISCONNECTED
                print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))
                print("[INFO][UNDOC] ST_UNDOC_DISCONNECTED")
            else:
                self.undocking_state = self.ST_UNDOC_END_RETRACT
                print("[INFO][UNDOC][ST_UNDOC_CHECK_CONNECTION] - UNDOC_PRES_LOAD = %d" % (dxl_present_load_signed))

        elif self.undocking_state == self.ST_UNDOC_DISCONNECTED:
            #print("[INFO][UNDOC] ST_UNDOC_DISCONNECTED")
            self.disable()

        elif self.undocking_state == self.ST_UNDOC_END_RETRACT:
            # Rotate motor CCW
            print("[INFO][UNDOC] ST_UNDOC_END_RETRACT")
            # Convert shigned short to signed int
            pwm_goal = pwm_goal = int.from_bytes((self.PWM_GOAL_MAX).to_bytes(2, byteorder='big', signed=True), byteorder='big')
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
                print("[INFO][UNDOC] ST_UNDOC_FAILURE")
                self.undocking_state = self.ST_UNDOC_FAILURE

        elif self.undocking_state == self.ST_UNDOC_FAILURE:
            #print("[INFO][UNDOC] ST_UNDOC_FAILURE")
            self.disable()

        elif self.undocking_state == self.ST_UNDOC_ERROR:
            #print("[INFO][UNDOC] ST_UNDOC_ERROR")
            self.disable()


    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_docking_station_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        # calculate timestamp and publish
        time_now = rospy.Time.now()
        # call cyclic functionality
        self.dock_connector()
    
    #shutdown hook
    def terminate(self):
        pass



# Main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_docking_station_mngr')
    #global qb_docking
    # Init and start the qB dockign station class
    qb_docking = QbDockingStation()
    qb_docking.calibration_state = qb_docking.ST_CAL_INIT
    qb_docking.main_loop()
    #shutdown listener
    while not rospy.is_shutdown():
        qb_docking.main_rate.sleep()
    
# Main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass
        # deinit doking station handler
        #qb_docking.deinit()
