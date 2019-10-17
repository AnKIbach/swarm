import socket
import struct
import sys

class receive:
    def __init__(self):
        self.others_position = [()]


    def update(self, *args):
        multicast_group = '224.3.29.71'
        server_address = ('', 10000)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sock.bind(server_address)
            
        group = socket.inet_aton(multicast_group)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        while True:
            print >>sys.stderr, '\nwaiting to receive message'
            data, address = sock.recvfrom(1024)
            
            print >>sys.stderr, 'received %s bytes from %s' % (len(data), address)
            print >>sys.stderr, data

            print >>sys.stderr, 'sending acknowledgement to', address
            sock.sendto('ack', address)
