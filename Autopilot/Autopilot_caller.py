#!/usr/bin/env python
import time

import Plotter_PID as plot

import To_arduino
from PID import PID
from GPS_definition import GPS
from Vector_definition import Vector
from ROS_operators.Navigation_data import navData

class autopilot:
    def __init__(self, use_guidance = True):
        self.use_guidance = use_guidance
        
    def set_wanted_xy(self, velocity_east, velocity_north):
        pass

    def set_wanted_vec(self, wanted)
        pass
    
    def __call__(self, current):
        if isinstance(current, Vector):
            pass
        if self.use_guidance:
            pass
        else:
            pass
    

    def show_history(self, *args):
        pass

    ## Få den til å fungere uten å vite om ROS eller arduino - tar inn ønsket og nåværende ting        

def connection_setup(nav): #can be adjusted by adding arduino object
    wait_time = 0.0
    while nav.get_connection_state() == False and wait_time < 5.0:
        time.sleep(0.1)
        wait_time += 0.1
        print("Trying to connect...")
    
    return nav.is_ready()


def update(nav, controller):
    GPS_wanted = GPS() 
    GPS_actual = GPS()

    speed_out_table = [] 
    speed_actual_table = []
    angle_out_table = []
    angle_actual_table = []
    GPS_wanted.set(60.394238, 5.266250)

    totTime = 0.0
    while totTime < 10.0:
        print("i loop")
        GPS_timeout = 0.0

        if GPS_timeout < 2.0 or GPS_timeout == 0.0:
            GPS_actual_raw = nav.get_GPS()
            GPS_actual.set(GPS_actual_raw[0], GPS_actual_raw[1])
            GPS_actual.show()

            vector_wanted = GPS_actual.calculate(GPS_wanted)
            vector_wanted.showVector()
            controller.set_wanted(vector_wanted) 
            if GPS_timeout < 2.0:
                GPS_timeout = 0.0

        velocity_actual = nav.get_velocity()
        bearing_actual  = nav.get_bearing()
        vector_actual = Vector(velocity_actual, bearing_actual)
        vector_actual.showVector()

        vector_out = controller.update(vector_actual) 

        speed_out_table.append(vector_out.magnitude) 
        speed_actual_table.append(vector_actual.magnitude)
        angle_out_table.append(vector_out.angle)
        angle_actual_table.append(vector_actual.angle)

        GPS_timeout += 0.1
        time.sleep(0.1)
        totTime += 0.1
        
    ply = plot.plotter()


    ply.present(speed_out_table,
                speed_actual_table,
                angle_out_table,
                angle_actual_table)

