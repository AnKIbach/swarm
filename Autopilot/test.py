#!/usr/bin/env python
import rospy
import time

from ROS_operators.Autopilot_sub import swarmWanted
from ROS_operators.Navigation_data import navData

def main():
    #nav = navData()
    wanted = swarmWanted()

    while not rospy.is_shutdown():
        try: 
            sest = wanted.time_since
            print(sest)
            time.sleep(0.5)
        except:
            pass
 
if __name__ == "__main__":
    main()