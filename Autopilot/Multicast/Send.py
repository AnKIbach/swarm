import socket
import struct
import sys

class send:
    def __init__(self):
        self.name = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.bearing = 0.0
        self.velocity = 0.0

        self.has_sent_data = False

    def update(self, *args):
        message = 'very important data'
        multicast_group = ('224.3.29.71', 10000)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sock.settimeout(0.2)

        try:

            print >> sys.stderr, 'sending "%s"' % message
            sent = sock.sendto(message, multicast_group)

            while True:
                print >>sys.stderr, 'waiting to receive'
                try:
                    data, server = sock.recvfrom(16)
                except socket.timeout:
                    print >>sys.stderr, 'timed out, no more responses'
                    break
                else:
                    print >>sys.stderr, 'received "%s" from %s' % (data, server)

        finally:
            print >>sys.stderr, 'closing socket'
            sock.close()