#!/usr/bin/env python
import rospy
import math as m

import std_msgs.msg 
import mavros_msgs.msg 
import sensor_msgs.msg 
import geometry_msgs.msg

#for GPS test 
import geographic_msgs.msg 

from GPS_class import GPS
from Vector_class import Vector

class navData:
    def __init__(self):
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
        return self.connection 

    def get_GPS(self):
        return self.GPS

    def get_bearing(self):
        return self.bearing

    def get_velocity(self):
        return self.velocity

    def get_Vector(self):
        self.vector = Vector(self.velocity, self.bearing)
        return self.vector

    def is_ready(self):
        if all( [self.is_connected == True, 
                self.has_GPS == True, 
                self.has_bearing == True, 
                self.has_velocity == True] ):
            return True
        else:
            return False


#for testing of GPS
class newGPS:
    def __init__(self):
        self.GPS = GPS()
        #gps topic without publisher
        GPS_adress = "/mavros/global_position/set_gp_origin"

        self.get_gps = rospy.Subscriber(GPS_adress, geographic_msgs.msg.GeoPoint, self._update_gps)

        self.GPS_received = False

    def __call__(self, current_wanted):
        if self.GPS_received == True:
            self.GPS_received = False
            return self.GPS
        else:
            return current_wanted

    def _update_gps(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        self.GPS.set(lat,lon)
        if lat > 50.0:
            self.GPS_received = True


