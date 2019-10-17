import rospy
import psutil

from autopilot.msg import Area
from autopilot.msg import Movement
from autopilot.msg import Position

from autopilot.msg import SwarmStatus
from autopilot.msg import SwarmOdometry
from autopilot.msg import SwarmCommand
from autopilot.msg import SwarmHeader
from autopilot.msg import BoatStatus
from autopilot.msg import BoatOdometry
from autopilot.msg import StateStatus


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
    msg['lat'] = position.latitude
    msg['lon'] = position.longitude
    return msg

def json2Position(msg):
    position = Position()
    position.latitude  = msg['lat']
    position.longitude = msg['lon']
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
    msg['tasktype']        = swarmCommand.taskType
    msg['headingMode']     = swarmCommand.headingMode
    msg['colavMode']       = swarmCommand.colavMode
    msg['destination']     = position2Json(swarmCommand.destination)
    msg['speed']           = swarmCommand.speed
    msg['heading']         = swarmCommand.heading
    # msg['area']            = area2Json(swarmCommand.area)
    msg['do_imediate']     = swarmCommand.doImidiate
    #msg['id']              = swarmCommand.id #uncertain of value
    return msg

def json2SwarmCommand(msg):
    cmd = SwarmCommand()
    cmd.header             = json2Header(msg['header'])
    cmd.taskType           = msg['tasktype']
    cmd.headingMode        = msg['headingMode']
    cmd.colavMode          = msg['colavMode']
    cmd.destination        = json2Position(msg['destination'])
    cmd.speed              = msg['speed']
    cmd.heading            = msg['heading']
    # cmd.area               = json2Area( msg['area'])
    cmd.doImidiate         = msg['do_imediate']
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

# #uncertain of value in code
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Vector3


# def detection2Json(detection):
#     msg = {}
#     msg['id']         = detection.id
#     msg['time_stamp'] = detection.time_stamp
#     msg['latitude']   = detection.latitude
#     msg['longitude']  = detection.longitude
#     msg['type']       = detection.type
#     return msg


# def latLonAltVector2Json(latLonAltVector):
#     msg = {}
#     msg['lat'] = latLonAltVector.latitude
#     msg['lon'] = latLonAltVector.longitude
#     msg['alt'] = latLonAltVector.altitude
#     return msg

# def json2LatLonAltVector(msg):
#     latLonAltVector = LatLonAltVector()
#     latLonAltVector.latitude = msg['lat']
#     latLonAltVector.longitude = msg['lon']
#     latLonAltVector.altitude = msg['alt']
#     return latLonAltVector

# def orientation2Json(orientation):
#     msg = {}
#     msg["w"] = orientation.w
#     msg["x"] = orientation.x
#     msg["y"] = orientation.y
#     msg["z"] = orientation.z
#     return msg

# def json2Orientation(msg):
#     orientation = Quaternion()
#     orientation.w = msg["w"]
#     orientation.x = msg["x"]
#     orientation.y = msg["y"]
#     orientation.z = msg["z"]
#     return orientation

# def pose2Json(pose):
#     msg = {}
#     msg["pos"] = position2Json(pose.pose.position)
#     msg["ati"] = orientation2Json(pose.pose.orientation)
#     return msg

# def json2Pose(msg):
#     pose = PoseWithCovariance()
#     pose.pose.position    = json2Position(   msg["pos"])
#     pose.pose.orientation = json2Orientation(msg["ati"])
#     return pose

# def odometry2Json(odometry): #not 100% on the definitons here
#     msg = {}
#     msg["position"]   = position2Json(odometry)
#     msg["movement"]   = movement2Json(odometry)
#     return msg

# def json2Odometry(msg):
#     odometry = BoatOdometry()
#     odometry.movement = json2Position(msg["position"])
#     odometry.position = json2Movement(msg["movement"])
#     return odometry

# def linVel2Json(linVel):
#     msg = {}
#     msg["E"] = linVel.x
#     msg["N"] = linVel.y
#     msg["U"] = linVel.z
#     return msg

# def json2LinVel(msg):
#     linVel   = Vector3()
#     linVel.x = msg["E"]
#     linVel.y = msg["N"]
#     linVel.z = msg["U"]
#     return linVel

# def angVel2Json(angVel):
#     msg = {}
#     msg["r"] = angVel.x
#     msg["p"] = angVel.y
#     msg["y"] = angVel.z
#     return msg

# def json2AngVel(msg):
#     angVel   = Vector3()
#     angVel.x = msg["r"]
#     angVel.y = msg["p"]
#     angVel.z = msg["y"]
#     return angVel

# def swarmStatus2Json(swarm_status):
#     msg = {}
#     msg["header"]        = header2Json(swarm_status.header)
#     msg["flight_status"] = flightStatus2Json(swarm_status.flightStatus)
#     msg["status"]  = stateStatus2Json( swarm_status.stateStatus)
#     return msg

# def json2SwarmStatus(msg):
#     swarm_status = SwarmStatus()
#     swarm_status.header       = json2Header(msg['header'])
#     swarm_status.flightStatus = json2FlightStatus(msg['flight_status'])
#     swarm_status.stateStatus  = json2StateStatus( msg['status'])
#     return swarm_status

# def flightStatus2Json(flight_status):
#     msg = {}
#     msg["aircraft_state"]      = flight_status.aircraftStatus
#     msg["fcu_mode"]            = flight_status.fcuMode
#     msg["fcu_status"]          = flight_status.fcuStatus
#     return msg

# def json2FlightStatus(msg):
#     flight_status = FlightStatus()
#     flight_status.aircraftStatus = msg["aircraft_state"]
#     flight_status.fcuMode = msg["fcu_mode"]
#     flight_status.fcuStatus = msg["fcu_status"]
#     return flight_status