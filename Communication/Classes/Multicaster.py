#!/usr/bin/env python
'''
This module contains a helper classes that is designed to support
sending and receiving UDP multicast messages

based on example from FFI

Questions: anhellesnes@fhs.mil.no
'''

import json
import socket
import struct
import zlib

# Global header used on all messages, this is the 'struct' format
HEADER_FMT = 'b'


class AbstractMulticastHandler(object):
    def __init__(self, mcast_grp, mcast_port):
        # Multicast group to send and join
        self._mcast_grp = mcast_grp
        # Multicast port to send and join
        self._mcast_port = mcast_port
        # Multicast enable socket to send on:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM,
                socket.IPPROTO_UDP)
        # Set reuse address in case someone else is using the same address
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

class MulticastListener(AbstractMulticastHandler):
    def __init__(self, mcast_grp, mcast_port, timeout=None, num_recv=8192):
        '''
        Create a new Multicast listener

        Args:
            mcast_grp: String of multicast group to send message to
            mcast_port: Integer with multicast port
            timeout: Float with socket timeout in seconds, default no timeout
            num_recv: Integer with number of bytes to receive at a time,
                default 8192 bytes
        '''
        super(MulticastListener, self).__init__(mcast_grp, mcast_port)
        # How many bytes should we receive at a time
        self._num_recv = num_recv
        # How big is the header in received UDP messages
        self._header_size = struct.calcsize(HEADER_FMT)
        # Bind to any address, this is done to support Windows which
        # doesn't support binding to a specific address
        self._sock.bind(('', mcast_port))
        # Join correct multicast group for reception
        mreq = struct.pack("4sl", socket.inet_aton(mcast_grp),
                socket.INADDR_ANY)
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                mreq)
        # Set optional timeout if user desires
        self.timeout = timeout

    @property
    def timeout(self):
        """
        Get the socket timeout of this listener

        Returns:
            Timeout in seconds or None if no timeout is set
        """
        return self._sock.gettimeout()

    @timeout.setter
    def timeout(self, time):
        """
        Set the timeout for the underlying socket

        Args:
            time: Timeout in seconds of None to remove timeout
        """
        self._sock.settimeout(time)

    @property
    def num_recv(self):
        """
        Get the number of bytes to receive by default

        Returns:
            Number of bytes to receive
        """
        return self._num_recv

    @num_recv.setter
    def num_recv(self, bytez):
        """
        Set the number of bytes to receive by default

        Args:
            Number of bytes to receive
        """
        self._num_recv = bytez

    def listen(self, bytez=None):
        """
        Listen for multicast messages

        Args:
            bytez: Number of bytes to receive, None uses default 'num_recv'
        Returns:
            None if we received message that contained 0 bytes or a dictionary
            of data
        """
        if bytez is None:
            bytez = self.num_recv
        data = self._sock.recv(bytez)
        if len(data) > self._header_size:
            # Unpack header
            header = struct.unpack_from(HEADER_FMT, data)[0]
            data = data[self._header_size:]
            if header & 1 == 1:
                print("trying decomp")
                # Use -15 so that zlib does not check for header or checksum
                data = zlib.decompress(data, -15)
            return self._load_data(data.decode('utf-8'))
        else:
            raise IOError("Not enough bytes to unpack in message: {!r}".format(data))

    def _load_data(self, data):
        """
        Helper method to unload data

        Returns:
            Dictionary of values
        """
        # TODO: Implement several data schemes in addition to JSON
        return json.loads(data)

class MulticastSender(AbstractMulticastHandler):
    def __init__(self, mcast_grp, mcast_port, ttl=1, compress=False,
            compress_level=6):
        """
        Create a new Multicast sender

        Args:
            mcast_grp: String of multicast group to send message to
            mcast_port: Integer with multicast port
            ttl: Integer of time-to-live for packet, default 1 hop
            compress: Should messages be compressed, default False
            compress_level: See zlib.compress, [0, 9], higher value more
                compression (also more processing time)
        """
        super(MulticastSender, self).__init__(mcast_grp, mcast_port)
        # Should messages be compressed
        self._compress = compress
        # zlib compression level
        self._rate = compress_level
        # Set time-to-live for packets
        self._sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    @property
    def compress(self):
        """
        Should outgoing messages be compressed?
        """
        return self._compress

    @compress.setter
    def compress(self, do_compress):
        """
        Set compression on sender
        """
        self._compress = do_compress

    @property
    def compression_level(self):
        """
        Get the compression of this sender

        Returns:
            0-9 where 0 is no compression and 9 is maximum
            compression
        """
        return self._rate

    @compression_level.setter
    def compression_level(self, level):
        """
        Set the compression level

        Args:
            0-9 where 0 is no compression and 9 is maximum
            compression
        """
        assert 0 <= level <= 9, "Compression level must be between [0, 9]"
        self._rate = level

    def send_message(self, message):
        """
        Send a message over UDP multicast

        Args:
            message: dictionary of data to send
        """
        # TODO: implement different data schemes in addition to JSON
        return self._send_data(json.dumps(message).encode('utf-8'))

    def _send_data(self, data):
        """
        Helper method to actually send data

        Args:
            data: bytes of data to transmit
        """
        header = 0
        if self._compress:
            print("compressing data..")
            header |= 1
            # We remove zlib header and checksum as that is superfluous and
            # many libraries doesn't support them
            data = zlib.compress(data, self._rate)[2:-4]
        self._sock.sendto(struct.pack(HEADER_FMT, header) + data,
                (self._mcast_grp, self._mcast_port))
        return True
