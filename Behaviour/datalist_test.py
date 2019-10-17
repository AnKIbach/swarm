import sys
import time

import rospy

from global_data import swarmData

def main():
    data = swarmData()

    time.sleep(2)

    while not rospy.is_shutdown():
            try:
                listest = data()

                print(listest[1].movment.bearing)

            except rospy.ROSInterruptException():
                sys.exit()

if __name__=="__main__":
    main()