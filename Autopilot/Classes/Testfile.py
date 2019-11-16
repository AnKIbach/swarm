from PID import PID
from Vector_class import Vector

import json
import socket
import struct
import zlib

def delta_angle_test():
    pid = PID()
    wanted = Vector(0.0, 0.0)
    current = Vector(0.0, 0.0)
    piv = Vector()

    pid.set_wanted(wanted)

    for i in range(0,360,1):
        wanted.set(0.0,i)
        pid.set_wanted(wanted)
        
        for j in range(0,360,1):
            current.set(0.0,j)
            piv = pid._get_delta_angle(current)
            if j in(45, 90, 135, 180, 225, 270, 315, 360) and i in(45, 90, 135, 180, 225, 270, 315, 360):
                print("current angle: ", j , " | wanted angle: ", i)
                print("closest angle: ", round(piv, 0))
                print("")

def pid_test():
    print(struct.pack('b', 0))
    

if __name__=="__main__":
    pid_test()