import psutil

from Classes.Objects import Header
from Classes.Objects import Odometry
from Classes.Objects import Status
from Classes.Objects import Command

def header2GCS(msg):
    msgType = msg['msgType']
    return msgType

def odometry2GCS(msg):
    odometry = Odometry(msg)
    return odometry

def status2GCS(msg):
    status = Status(msg)
    return status

def GCS2command(noe): 
    return noe
