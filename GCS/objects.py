

class header(object):
    def __init__(self, msg):
        self.secs       = msg['secs']
        self.nsecs      = msg['nsecs']
        self.seq        = msg['seq']
        self.id         = msg['id']
        self.msgType    = msg['msgType']
        self.reTransmit = msg['reTransmit']
        self.Ack        = msg['Ack']

class odometry(header):
    def __init__(self, msg):
        super(odometry, self).__init__(msg['header'])
        
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

class status(header):
    def __init__(self, msg):
        super(status, self).__init__(msg['header'])

        self.fcuMode              = msg["fcu_mode"]
        self.fcuStatus            = msg["fcu_status"] 
        self.timeSinceLaunch      = msg["time_since_launch"]
        self.distanceFromLaunch   = msg["distance_from_launch"] 
        self.numGpsSatelites      = msg["num_gps_satelites"] 
        self.pixhawkReady         = msg["pixhawk_ready"] 
        self.arduinoReady         = msg["arduino_ready"] 
        self.hasGPSFix            = msg["has_gps_fix"]
        self.hasWiFi              = msg["has_wifi"] 


class command(header):    
    def __init__(self, msg):
        super(command, self).__init__(msg['header'])

        pass

