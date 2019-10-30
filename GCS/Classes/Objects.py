class Header(object):
    def __init__(self):
        self.secs       = 0
        self.nsecs      = 0
        self.seq        = 0
        self.id         = 0
        self.reTransmit = 0
        self.Ack        = 0

    def set(self, msg):
        self.secs       = msg['secs']
        self.nsecs      = msg['nsecs']
        self.seq        = msg['seq']
        self.id         = msg['id']
        self.msgType    = msg['msgType']
        self.reTransmit = msg['reTransmit']
        self.Ack        = msg['Ack']

class Odometry(Header):
    def __init__(self):
        super(Odometry, self).__init__()
        self.latitude = 0.0
        self.longitude = 0.0
        self.velocity = 0.0
        self.bearing = 0.0

    def set(self, msg):      
        super().set(msg['header'])
        self._handle_position(msg["position"])
        self._handle_movement(msg["movement"])

    def _handle_position(self, position):
        self.latitude  = position["latitude"]
        self.longitude = position["longitude"]

    def _handle_movement(self, movement):
        self.velocity = movement["velocity"]
        self.bearing  = movement["bearing"] 

    def get_latitude(self):
        return self.latitude
    def get_longitude(self):
        return self.longitude
    def get_speed(self):
        return self.velocity
    def get_bearing(self):
        return self.bearing

class Status(Header):
    def __init__(self, msg):
        super(Status, self).__init__(msg['header'])

        self.fcuMode              = msg["fcu_mode"]
        self.fcuStatus            = msg["fcu_status"] 
        self.timeSinceLaunch      = msg["time_since_launch"]
        self.distanceFromLaunch   = msg["distance_from_launch"] 
        self.numGpsSatelites      = msg["num_gps_satelites"] 
        self.pixhawkReady         = msg["pixhawk_ready"] 
        self.arduinoReady         = msg["arduino_ready"] 
        self.hasGPSFix            = msg["has_gps_fix"]
        self.hasWiFi              = msg["has_wifi"] 


class Command(Header):    
    def __init__(self, msg):
        super(Command, self).__init__(msg['header'])

        pass

