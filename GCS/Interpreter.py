import psutil
import json

from objects import Header
from objects import Odometry
from objects import Status
from objects import Command

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
    pass
