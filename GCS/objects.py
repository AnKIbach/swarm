

class header(object):
    def __init__(self, id, msgType, Ack):
        self.id 

class odometry(header):
    def __init__(self, id, msgType, Ack, lat, lon, vel, hdg):
        super(odometry, self).__init__(id, msgType, Ack)

        
