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

# Motor Controller APIs
from motor_control import MotorController

# Power Seonsor APIs
from power_sensor import PowerSensor

# Relay Controller APIs
from relay_control import RelayController

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

    # Docking Station State Definition
    ST_INIT                 = 0
    ST_EXTEND_CONENCTOR     = 1
    ST_RETRACT_CONNECTOR    = 2
    ST_ENGAGE_POWER         = 3
    ST_DISENGAGE_POWER      = 4
    ST_SUCCESS              = 5
    ST_FAILURE              = 6
    ST_ERROR                = 7
    docking_station_state = ST_INIT

    # Motor Controller handler
    motor_controller = None

    # Power Seonsor handler
    power_sensor = None

    # GPIO used for relay commands
    RELAY_GPIO = 4
    # Relay Controller handler
    relay_controller = None


    # Init
    def __init__(self):
        #read params
        self.qb_doking_dxl_id = get_param('~qb_doking_dxl_id', 1)
        self.qb_doking_dxl_serial = get_param('~qb_doking_dxl_serial', "/dev/ttyUSB0")
        self.qb_doking_dxl_baud = get_param('~qb_doking_dxl_baud', 57600)
        self.qb_doking_dxl_pwm_treshold = get_param('~qb_doking_dxl_pwm_treshold', 800)
        self.qb_doking_dxl_load_treshold = get_param('~qb_doking_dxl_load_treshold', 520)
        self.qb_doking_verbose = get_param('~qb_doking_verbose', False)
        # Server feedback/result definition
        self.qb_docking_station_server_feedback = QbDockingStationFeedback()
        self.qb_docking_station_server_result = QbDockingStationResult()
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup action server
        self.qb_docking_station_server = actionlib.SimpleActionServer('qb_docking_station_server', QbDockingStationAction, execute_cb=self.qb_docking_station_server_hndl, auto_start=False)
        self.qb_docking_station_server.start()
        # Create/init motor controller class
        self.motor_controller = MotorController(self.qb_doking_dxl_id, 
                                                self.qb_doking_dxl_serial,
                                                self.qb_doking_dxl_baud,
                                                self.qb_doking_dxl_pwm_treshold,
                                                self.qb_doking_dxl_load_treshold,
                                                self.qb_doking_verbose)
        # Create/init power sensor class
        self.power_sensor = PowerSensor()
        # Create/init relay controller class
        self.relay_controller = RelayController(self.RELAY_GPIO)


    # Docking station Server Callback
    def qb_docking_station_server_hndl(self, goal):
        self.docking_station_state = goal.command
        print("Goal set to - %d " %  self.docking_station_state)
        #self.qb_docking_station_server.set_succeeded(self.qb_docking_station_server_result)

    # Docking Station Deinitialization 
    def deinit(self):
        # Deinit motor controller
        self.motor_controller.deinit()
        # Deinit relay controller
        self.relay_controller.deinit()


    # Main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_docking_station_calc_hz)), self.fast_timer)


    # Timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        # calculate timestamp and publish
        time_now = rospy.Time.now()
        # no goal received
        if(self.docking_station_state == self.ST_INIT):
            pass
        # goal received to extend connector to the robot
        elif(self.docking_station_state == self.ST_EXTEND_CONENCTOR):
            __result = self.motor_controller.dock_connector()
            if(__result == self.motor_controller.ST_DOC_CONNECTED):
                # connection extablished
                self.docking_station_state = self.ST_SUCCESS
            elif (__result == self.motor_controller.ST_DOC_FAILURE or __result == self.motor_controller.ST_DOC_ERROR):
                # connection error
                self.docking_station_state = self.ST_FAILURE
            else:
                # in progress
                self.qb_docking_station_server_feedback.status = __result
                self.qb_docking_station_server.publish_feedback(self.qb_docking_station_server_feedback)
                self.docking_station_state = self.ST_EXTEND_CONENCTOR

        # goal received to retract connector from the robot
        elif(self.docking_station_state == self.ST_RETRACT_CONNECTOR):
            __result = self.motor_controller.undock_connector()
            if(__result == self.motor_controller.ST_UNDOC_DISCONNECTED):
                # connector disengaged
                self.docking_station_state = self.ST_SUCCESS
            elif (__result == self.motor_controller.ST_UNDOC_FAILURE or __result == self.motor_controller.ST_UNDOC_ERROR):
                # connector still engaged
                self.docking_station_state = self.ST_FAILURE
            else:
                # in progress
                self.qb_docking_station_server_feedback.status = __result
                self.qb_docking_station_server.publish_feedback(self.qb_docking_station_server_feedback)
                self.docking_station_state = self.ST_RETRACT_CONNECTOR

        # goal received to energise the power connector
        elif(self.docking_station_state == self.ST_ENGAGE_POWER):
            self.relay_controller.engage_relay()

        # goal received to disengage the power connector
        elif(self.docking_station_state == self.ST_DISENGAGE_POWER):
            self.relay_controller.disengage_relay()

        # goal successfuly executed
        elif(self.docking_station_state == self.ST_SUCCESS):
            # reset the state machine
            self.qb_docking_station_server.set_succeeded(self.qb_docking_station_server_result)
            self.docking_station_state =  self.ST_INIT

        # goal cannoe be successuly reached
        elif(self.docking_station_state == self.ST_FAILURE):
            # reset the state machine
            self.qb_docking_station_server.set_aborted(self.qb_docking_station_server_result)
            self.docking_station_state =  self.ST_INIT

        # handling error
        elif(self.docking_station_state == self.ST_ERROR):
            self.qb_docking_station_server.set_aborted(self.qb_docking_station_server_result)

        # unknown state
        else:
            self.docking_station_state = self.ST_ERROR


    def terminate(self):
        pass



# Main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_docking_station_mngr')
    #global qb_docking
    # Init and start the qB dockign station class
    qb_docking = QbDockingStation()
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
        qb_docking.deinit()
