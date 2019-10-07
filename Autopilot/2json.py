import rospy

from communication.msg import LatLonAltVector
from communication.msg import Area
from communication.msg import Route

from communication.msg import SwarmStatus
from communication.msg import SwarmOdometry
from communication.msg import SwarmCommand
from communication.msg import SwarmHeader
from communication.msg import FlightStatus
from communication.msg import StateStatus

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3




def header2Json(header):
    msg = {}
    msg['secs']       = header.secs
    msg['nsecs']      = header.nsecs
    msg['seq']        = header.seq
    msg['id']         = header.id
    msg['msgType']    = header.msgType
    msg['reTransmit'] = header.reTransmit
    msg['requestAck'] = header.requestAck
    return msg

def json2Header(msg):
    header            = SwarmHeader()
    header.secs       = msg['secs']
    header.nsecs      = msg['nsecs']
    header.seq        = msg['seq']
    header.id         = msg['id']
    header.msgType    = msg['msgType']
    header.reTransmit = msg['reTransmit']
    header.requestAck = msg['requestAck']
    return header

def position2Json(position):
    msg = {}
    msg['x'] = position.x
    msg['y'] = position.y
    msg['z'] = position.z
    return msg

def json2Position(msg):
    position = Point()
    position.x = msg['x']
    position.y = msg['y']
    position.z = msg['z']
    return position

def latLonAltVector2Json(latLonAltVector):
    msg = {}
    msg['lat'] = latLonAltVector.latitude
    msg['lon'] = latLonAltVector.longitude
    msg['alt'] = latLonAltVector.altitude
    return msg

def json2LatLonAltVector(msg):
    latLonAltVector = LatLonAltVector()
    latLonAltVector.latitude = msg['lat']
    latLonAltVector.longitude = msg['lon']
    latLonAltVector.altitude = msg['alt']
    return latLonAltVector

def orientation2Json(orientation):
    msg = {}
    msg["w"] = orientation.w
    msg["x"] = orientation.x
    msg["y"] = orientation.y
    msg["z"] = orientation.z
    return msg

def json2Orientation(msg):
    orientation = Quaternion()
    orientation.w = msg["w"]
    orientation.x = msg["x"]
    orientation.y = msg["y"]
    orientation.z = msg["z"]
    return orientation

def pose2Json(pose):
    msg = {}
    msg["pos"] = position2Json(pose.pose.position)
    msg["ati"] = orientation2Json(pose.pose.orientation)
    return msg

def json2Pose(msg):
    pose = PoseWithCovariance()
    pose.pose.position    = json2Position(   msg["pos"])
    pose.pose.orientation = json2Orientation(msg["ati"])
    return pose



def velocity2Json(vel):
    msg = {}
    msg["lin"] = linVel2Json(vel.twist.linear)
    msg["ang"] = angVel2Json(vel.twist.angular)
    return msg

def json2Velocity(msg):
    vel = TwistWithCovariance()
    vel.twist.linear  = json2LinVel(msg["lin"] )
    vel.twist.angular = json2AngVel(msg["ang"] )
    return vel

def swarmOdometry2Json(swarmOdometry):
    msg = {}
    msg["header"]   = header2Json(swarmOdometry.header)
    msg["odometry"] = odometry2Json(swarmOdometry.odometry)
    return msg

def json2SwarmOdometry(msg):
    swarmOdometry = SwarmOdometry()
    swarmOdometry.header   = json2Header(msg["header"])
    swarmOdometry.odometry = json2Odometry(msg["odometry"])
    return swarmOdometry

def odometry2Json(odometry):
    msg = {}
    msg["pose"]   = pose2Json(    odometry.pose)
    msg["vel"]    = velocity2Json(odometry.twist)
    return msg

def json2Odometry(msg):
    odometry = Odometry() #runtimedata
    odometry.pose   = json2Pose(msg["pose"])
    odometry.twist  = json2Velocity(msg["vel"])
    return odometry

def swarmCommand2Json(swarmCommand):
    msg = {}
    msg['header']          = header2Json(swarmCommand.header)
    msg['tasktype']        = swarmCommand.taskType
    msg['headingMode']     = swarmCommand.headingMode
    msg['colavMode']       = swarmCommand.colavMode
    msg['destination']     = latLonAltVector2Json( swarmCommand.destination        )
    msg['locationOfInterest'] = latLonAltVector2Json( swarmCommand.locationOfInterest )
    msg['speed']           = swarmCommand.speed
    msg['heading']         = swarmCommand.heading
    msg['route']           = route2Json(swarmCommand.route)
    msg['area']            = area2Json(swarmCommand.area)
    msg['do_imediate']     = swarmCommand.doImidiate
    #msg['id']              = swarmCommand.id
    return msg

def json2SwarmCommand(msg):
    cmd = SwarmCommand()
    cmd.header             = json2Header(msg['header'])
    cmd.taskType           = msg['tasktype']
    cmd.headingMode        = msg['headingMode']
    cmd.colavMode          = msg['colavMode']
    cmd.destination        = json2LatLonAltVector( msg['destination']     )
    cmd.locationOfInterest = json2LatLonAltVector( msg['locationOfInterest'] )
    cmd.speed              = msg['speed']
    cmd.heading            = msg['heading']
    cmd.route              = json2Route(msg['route'])
    cmd.area               = json2Area( msg['area'])
    cmd.doImidiate         = msg['do_imediate']
    return cmd



def stateStatus2Json(state_status):
    msg = {}
    msg["battery_ratio"]              = state_status.batteryRatio
    msg["time_since_launch"]          = state_status.timeSinceLaunch
    msg["distance_from_launch"]       = state_status.distanceFromLaunch
    msg["has_gps_fix"]                = state_status.hasGPSFix
    msg["num_gps_satelites"]          = state_status.numGpsSatelites
    msg["has_controller_signal"]      = state_status.hasControllerSignal
    msg["has_attitude_stabilization"] = state_status.hasAttitudeStabilization
    msg["has_yaw_position"]           = state_status.hasYawPosition
    msg["has_altitude_control"]       = state_status.hasAltitudeControl
    msg["has_position_control"]       = state_status.hasPositionControl
    return msg

def json2StateStatus(msg):
    state_status = StateStatus()
    state_status.batteryRatio             = msg["battery_ratio"]
    state_status.timeSinceLaunch          = msg["time_since_launch"]
    state_status.distanceFromLaunch       = msg["distance_from_launch"]
    state_status.hasGPSFix                = msg["has_gps_fix"]
    state_status.numGpsSatelites          = msg["num_gps_satelites"]
    state_status.hasControllerSignal      = msg["has_controller_signal"]
    state_status.hasAttitudeStabilization = msg["has_attitude_stabilization"]
    state_status.hasYawPosition           = msg["has_yaw_position"]
    state_status.hasAltitudeControl       = msg["has_altitude_control"]
    state_status.hasPositionControl       = msg["has_position_control"]
    return state_status

def swarmStatus2Json(swarm_status):
    msg = {}
    msg["header"]        = header2Json(swarm_status.header)
    msg["flight_status"] = flightStatus2Json(swarm_status.flightStatus)
    msg["state_status"]  = stateStatus2Json( swarm_status.stateStatus)
    return msg

def json2SwarmStatus(msg):
    swarm_status = SwarmStatus()
    swarm_status.header       = json2Header(msg['header'])
    swarm_status.flightStatus = json2FlightStatus(msg['flight_status'])
    swarm_status.stateStatus  = json2StateStatus( msg['state_status'])
    return swarm_status

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


# def detection2Json(detection):
#     msg = {}
#     msg['id']         = detection.id
#     msg['time_stamp'] = detection.time_stamp
#     msg['latitude']   = detection.latitude
#     msg['longitude']  = detection.longitude
#     msg['type']       = detection.type
#     return msg

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