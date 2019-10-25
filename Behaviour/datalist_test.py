#!/usr/bin/env python
import time
import rospy

from Global_data import swarmData # maybe cause of error 

def main():
    data = swarmData()

    time.sleep(2)

    while not rospy.is_shutdown():
            try:
                listest = data()
                p = listest[1].movement.bearing
                print(p)
                time.sleep(0.1)

            except rospy.ROSInterruptException():
                pass

if __name__=="__main__":
    main()