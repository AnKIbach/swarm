'''
This file contains functions for converting between ROS msgs and JSON
based on examples from FFI

Questions: anhellesnes@fhs.mil.no
'''

import rospy
import psutil

from swarm.msg import Area
from swarm.msg import Movement
from swarm.msg import Position

from swarm.msg import SwarmStatus
from swarm.msg import SwarmOdometry
from swarm.msg import SwarmCommand
from swarm.msg import SwarmHeader
from swarm.msg import BoatStatus
from swarm.msg import BoatOdometry
from swarm.msg import StateStatus


def header2Json(header):
    msg = {}
    msg['secs']       = header.secs
    msg['nsecs']      = header.nsecs
    msg['seq']        = header.seq
    msg['id']         = header.id
    msg['msgType']    = header.msgType
    msg['reTransmit'] = header.reTransmit
    msg['Ack']        = header.Ack
    return msg

def json2Header(msg):
    header            = SwarmHeader()
    header.secs       = msg['secs']
    header.nsecs      = msg['nsecs']
    header.seq        = msg['seq']
    header.id         = msg['id']
    header.msgType    = msg['msgType']
    header.reTransmit = msg['reTransmit']
    header.Ack        = msg['Ack']
    return header

def position2Json(position):
    msg = {}
    msg['latitude'] = position.latitude
    msg['longitude'] = position.longitude
    return msg

def json2Position(msg):
    position = Position()
    position.latitude  = msg['latitude']
    position.longitude = msg['longitude']
    return position

def movement2Json(movement):
    msg = {}
    msg["speed"] = movement.velocity
    msg["heading"] = movement.bearing
    return msg

def json2Movement(msg):
    movement = Movement()
    movement.velocity = msg["speed"]
    movement.bearing  = msg["heading"]
    return movement

def boatOdometry2Json(BoatOdometry):
    msg = {}
    msg["header"]   = header2Json(BoatOdometry.header)
    msg["position"] = position2Json(BoatOdometry.position)
    msg["movement"] = movement2Json(BoatOdometry.movement)
    return msg

def json2BoatOdometry(msg):
    odom = BoatOdometry()
    odom.header   = json2Header(msg["header"])
    odom.position = json2Position(msg["position"])
    odom.movement = json2Movement(msg["movement"])
    return odom

def boatStatus2Json(status):
    msg = {}
    msg["fcu_mode"]                   = status.fcuMode
    msg["fcu_status"]                 = status.fcuStatus
    msg["time_since_launch"]          = status.timeSinceLaunch
    msg["distance_from_launch"]       = status.distanceFromLaunch
    msg["num_gps_satelites"]          = status.numGpsSatelites
    msg["pixhawk_ready"]              = status.pixhawkReady 
    msg["arduino_ready"]              = status.arduinoReady 
    msg["has_gps_fix"]                = status.hasGPSFix
    msg["has_wifi"]                   = status.hasWiFi
    return msg

def json2BoatStatus(msg):
    status = BoatStatus()
    status.fcuMode              = msg["fcu_mode"]
    status.fcuStatus            = msg["fcu_status"] 
    status.timeSinceLaunch      = msg["time_since_launch"]
    status.distanceFromLaunch   = msg["distance_from_launch"] 
    status.numGpsSatelites      = msg["num_gps_satelites"] 
    status.pixhawkReady         = msg["pixhawk_ready"] 
    status.arduinoReady         = msg["arduino_ready"] 
    status.hasGPSFix            = msg["has_gps_fix"]
    status.hasWiFi              = msg["has_wifi"] 
    return status

def swarmODometry2Json(SwarmOdometry):
    msg = {}
    msg["header"]   = header2Json(SwarmOdometry.header)
    msg["position"] = position2Json(SwarmOdometry.position)
    msg["movement"] = movement2Json(SwarmOdometry.movement)
    return msg

def json2SwarmOdometry(msg):
    odom = SwarmOdometry()
    odom.header   = json2Header(msg["header"])
    odom.position = json2Position(msg["position"])
    odom.movement = json2Movement(msg["movement"])
    return odom

def swarmStatus2Json(SwarmStatus):
    msg = {}
    msg['header'] = header2Json(SwarmStatus.header)
    msg['status'] = boatStatus2Json(SwarmStatus.boatStatus)
    return msg

def json2SwarmStatus(msg):
    swarmstat = SwarmStatus()
    swarmstat.header     = json2Header(msg['header'])
    swarmstat.boatStatus = json2BoatStatus(msg['status'])
    return swarmstat
 
def swarmCommand2Json(swarmCommand):
    msg = {}
    msg['header']          = header2Json(swarmCommand.header)
    msg['taskType']        = swarmCommand.taskType
    msg['headingMode']     = swarmCommand.headingMode
    msg['colavMode']       = swarmCommand.colavMode
    msg['destination']     = position2Json(swarmCommand.destination)
    msg['fence']           = position2Json(swarmCommand.fence)
    msg['speed']           = swarmCommand.speed
    msg['heading']         = swarmCommand.heading
    msg['doImidiate']     = swarmCommand.doImidiate
    return msg

def json2SwarmCommand(msg):
    cmd = SwarmCommand()
    cmd.header             = json2Header(msg['header'])
    cmd.taskType           = msg['taskType']
    cmd.headingMode        = msg['headingMode']
    cmd.colavMode          = msg['colavMode']
    cmd.destination        = json2Position(msg['destination'])
    cmd.fence              = json2Position(msg['fence'])
    cmd.speed              = msg['speed']
    cmd.heading            = msg['heading']
    cmd.doImidiate         = msg['doImidiate']
    return cmd

def cpuStatus2Json():
    msg = {}

    msg['cpu'] = {}
    msg['cpu']['percent'] = psutil.cpu_percent(percpu=True)
    msg['memory'] = {}
    mem = psutil.virtual_memory()
    msg['memory']['total'] = mem.total
    msg['memory']['available'] = mem.available
    msg['memory']['percent'] = mem.percent
    msg['memory']['used'] = mem.used
    msg['memory']['free'] = mem.free
    time_now = rospy.get_rostime()
    msg['time'] = {'secs': time_now.secs,
                   'nsecs': time_now.nsecs}
    return msg
