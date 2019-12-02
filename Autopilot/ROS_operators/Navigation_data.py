#!/usr/bin/env python
'''
This class with a ROS publisher is used to get current nav data from sensor

Based on examples from FFI
Questions: anhellesnes@fhs.mil.no
'''

import rospy
import math as m

import std_msgs.msg 
import mavros_msgs.msg 
import sensor_msgs.msg 
import geometry_msgs.msg

from Classes.GPS_class import GPS
from Classes.Vector_class import Vector

class navData:
    '''Helper class that fetches data from sensor (PX4) through ROS topics'''

    def __init__(self):
        '''Initialises subscribers to active given topics'''

        rospy.init_node('navigation', anonymous=True)

        statePX     = "/mavros/state" 
        GPSPX       = "/mavros/global_position/raw/fix"
        compassPX   = "/mavros/vfr_hud"
        velocityPX  = "/mavros/global_position/raw/gps_vel"

        rospy.Subscriber(statePX, mavros_msgs.msg.State, self._handle_state)
        rospy.Subscriber(GPSPX, sensor_msgs.msg.NavSatFix, self._handle_GPS)
        rospy.Subscriber(compassPX, mavros_msgs.msg.VFR_HUD, self._handle_compass)
        rospy.Subscriber(velocityPX, geometry_msgs.msg.TwistStamped, self._handle_velocity)

        self.is_connected = False
        self.has_GPS      = False
        self.has_bearing  = False
        self.has_velocity = False

        self.connection = False
        self.mode = ""

        self.lat = 0.0
        self.lon = 0.0
        self.GPS = GPS()

        self.bearing = 0.0

        self.velocity = 0.0

    def _handle_state(self, msg):
        self.connection = msg.connected
        self.mode = msg.mode

        self.is_connected = True

    def _handle_GPS(self, msg):
        self.lat = msg.latitude 
        self.lon = msg.longitude
        self.GPS.set(self.lat, self.lon)

        self.has_GPS = True

    def _handle_compass(self, msg):
        self.bearing = msg.heading

        self.has_bearing = True

    def _handle_velocity(self, msg):
        velEast = msg.twist.linear.x
        velNorth = msg.twist.linear.y

        self.velocity = m.sqrt((velEast)**2+(velNorth)**2)

        self.has_velocity = True

    def get_connection_state(self):
        '''Helper function get connection state of sensor

        Returns:
            Boolean to determine conneciton state
        '''
        return self.connection 

    def get_GPS(self):
        '''Helper function get current GPS position

        Returns:
            GPS point to current position
        '''
        return self.GPS

    def get_bearing(self):
        '''Helper function get current bearing

        Returns:
            Float of current bearing
        '''
        return self.bearing

    def get_velocity(self):
        '''Helper function get current velocity

        Returns:
            Float of current velocity
        '''
        return self.velocity

    def get_Vector(self):
        '''Helper function get current movement vector

        Returns:
            Vector object containing current movement
        '''
        self.vector = Vector(self.velocity, self.bearing)
        return self.vector

    def is_ready(self):
        '''Helper function to determine if all sensors are ready and publishing

        Returns:
            Boolean of wether sensors are ready or not
        '''
        if all( [self.is_connected == True, 
                self.has_GPS == True, 
                self.has_bearing == True, 
                self.has_velocity == True] ):
            return True
        else:
            return False



