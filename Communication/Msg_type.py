from enum import IntEnum

class MsgType(IntEnum):
    HEARTBEAT      = 0
    ODOMETRY       = 1
    SWARM_COMMAND  = 2
    BOAT_STATUS    = 3