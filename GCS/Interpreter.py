import psutil
import json

from Classes.Objects import Header
from Classes.Objects import Odometry
from Classes.Objects import Status
from Classes.Objects import Command

def header2GCS(msg):
    header = Header(msg)
    return header

def odometry2GCS(msg):
    odometry = Odometry(msg)
    return odometry

def status2GCS(msg):
    status = Status(msg)
    return status

def GCS2command(noe): 
    return noe
