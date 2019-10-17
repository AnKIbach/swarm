#!/usr/bin/env python
import time
import rospy

import global_data

def main():
    data = global_data.swarmData()

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