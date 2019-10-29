#!/usr/bin/env python2

"""
ROS node responsible for decoding position messages and distributing them
on multicast UDP
"""

import sys
import os

# sys.path.append(
#     os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rospy

from Multicast.Multicaster import MulticastSender
import Interpreter

from swarm.msg import SwarmCommand

HEADER_FMT = 'b'

class CommandPublisher(object):
    """
    This class exposes two methods, "handle_nav_data" and "handle_pose_data"
    which can be used as callback functions. These functions serializes the
    given ROS messages into JSON messages before sending the data on
    UDP multicast
    """
    def __init__(self, mcast_grp, mcast_port, ttl=2, compress=False,
            state_hz=1.0, cpu_hz=1.0):
      
        self._msender = MulticastSender(mcast_grp, mcast_port, compress=compress)
        self._swarmCommand = None

    def shutdown(self):
        """
        Helper method to shutdown all running code
        """
        try:
            if self._publish is not None:
                self._publish.cancel()
        except:
            pass

    def handle_swarm_command(self, swarmCommand):
        """
        Handles SwarmCommand callback
        """

        msg = Interpreter.GCS2command(swarmCommand)
        self._msender.send_message(msg)

        
def main():
    rospy.init_node('gcs_tx_node', anonymous=True)

    rospy.logdebug("Started pos_udp_gcs node")

    mcast_grp = rospy.get_param('~mcast_addr', "225.0.0.25")
    mcast_port = rospy.get_param('~mcast_port', 4243)
    compress = rospy.get_param('~compression', True)
    state_hz = rospy.get_param('~state_hz', 1.0)
    cpu_hz = rospy.get_param('~cpu_hz', 1.0)
    ttl = rospy.get_param('~ttl', 1)


    rospy.loginfo("Using multicast group: {}:{}".format(mcast_grp, mcast_port))

    rospy.logdebug("Should output be compressed: {!s}".format(compress))
    rospy.logdebug("Time to live for UDP: {!s}".format(ttl))

    publisher = CommandPublisher(mcast_grp, mcast_port,
            ttl=ttl, compress=compress, state_hz=state_hz,
            cpu_hz=cpu_hz)
    
    swarm_order_sub = rospy.get_param('~swarm_order_sub', "/gcs/data/command")
    rospy.Subscriber(swarm_order_sub, SwarmCommand, publisher.handle_swarm_command)
   
    #Give control over to ROS so that Python doesn't exit
    rospy.loginfo("Starting to listen")
    rospy.spin()
    publisher.shutdown()
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
