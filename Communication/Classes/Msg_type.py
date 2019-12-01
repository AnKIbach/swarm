from enum import IntEnum

class MsgType(IntEnum):
    '''Helper class method to enumerate message types to reciever'''
    HEARTBEAT      = 0
    ODOMETRY       = 1
    SWARM_COMMAND  = 2
    BOAT_STATUS    = 3
