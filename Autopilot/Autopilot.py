#!/usr/bin/env python
import time
import sys
import rospy

from GPS_class import GPS
from PID_plotter import Plotter
from Vector_class import Vector
from Arduino_data import Arduino
from Autopilot_call import Autopilot
from ROS_operators.Navigation_data import navData
from ROS_operators.Navigation_data import newGPS #newGPS for testing
from ROS_operators.Autopilot_talker import Talker


def main():
    nav = navData()
    plt = Plotter()
    new_gps = newGPS() 
    autopilot = Autopilot()
    
    #uncomment for test
    arduino = Arduino('/dev/ttyACM1', speedLimit = 0.6) #speed limiter for testing
    
    #fix for test
    autopilot_talker = Talker()

    sOut, sAct, sWan, aOut, aAct, aWan = [], [], [], [], [], []
    wait_time, clicks = 0.0, 0
    wanted_GPS = GPS(60.394217, 5.266299)

    while nav.is_ready() == False and arduino.is_ready() == False:
    #waits for both systems to connect
        wait_time += 0.1
        time.sleep(0.1)
        if wait_time > 10.0: #exit if timeout is over 10s
            sys.exit(0)

    if nav.is_ready() == True:
        print("Pixhawk is connected and ready at: ", nav.mode)

    if arduino.is_ready() == True:
        print("Arduino is connected and started at: ", arduino.port)

    print("entering autopilot loop...")

    while True:
        try:
            wanted_GPS = new_gps(wanted_GPS)

            current_GPS = nav.get_GPS()
            current_vector = nav.get_Vector()

            wanted_vector = current_GPS.calculate(wanted_GPS)

            autopilot.set_wanted_vector(wanted_vector)

            change_vector = autopilot(current_vector)

            print("change vector")
            change_vector.showVector() 
            
            arduino(change_vector.magnitude, change_vector.angle) #possible addition of another dampening for angle
            
            #publishing to ROS
            autopilot_talker(current_vector, 
                            current_GPS,
                            wanted_vector,
                            wanted_GPS, #change for behaviour
                            change_vector,
                            nav_status=True,
                            ardu_status=True
                            )


            #for visualisation of values:
            sAct.append(current_vector.magnitude)
            aAct.append(current_vector.angle)
            sWan.append(wanted_vector.magnitude)
            aWan.append(wanted_vector.angle)
            sOut.append(change_vector.magnitude)
            aOut.append(change_vector.angle)

            #presentation of current data after 20 clicks
            if clicks >= 20:
                plt.present(sAct, sWan, sOut)
                plt.present(aAct, aWan, aOut)
                clicks = 0 
            else:
                clicks += 1

            #comment out for full test
            time.sleep(0.5)

        except rospy.ROSInterruptException():
            sys.exit()
        finally:
            pass



if __name__ == "__main__":
    main()

