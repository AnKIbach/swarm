import sys
import socket
import tkinter
from tkinter import Frame, Label, Button, RAISED
import time
import threading
import queue
import random

import Interpreter

from Classes.Msg_type import MsgType
from Classes.Objects import Odometry
from Classes.Objects import Status
from Multicast.Multicaster import MulticastListener

BOATS_IN_SWARM = 4
MCAST_GROUP = '225.0.0.25'
MCAST_PORT = 4243
TIMEOUT = 1.0
BACKGROUND_COLOR = "#333333" 
SUB_FRAME_COLOR = "#3C3F41"  
STAND_TEXT_WIDTH = 18  
STAND_DATA_WIDTH = 10  


class GUI:
    def __init__(self, master, queue, endCommand):
        self.master = master
        self.queue = queue
        #-------GUI part-----#
        self.master.geometry("1200x600")
        self.hxw = 600  # height and width of master frame
        self.master.title("Ground Control Station - SVERM")

        # ========== FRAMES ========== #
        # Initialize uppermost frame
        frame = Frame(master, width=self.hxw, height=self.hxw, bg="#333333")
        frame.pack(fill='both', expand='yes')

        self._place_div(endCommand)
        self._place_subFrames()
        self._place_velLabels()
        self._place_berLabels()
        self._place_posLabels()
        self._place_frameLabels()
        self._place_dataLabelsVel()
        self._place_dataLabelsBer()
        self._place_dataLabelsLat()
        self._place_dataLabelsLon()

        self.statusLabel = Label(self.subFrameBottom, text="NOT VERIFIED", fg="orange", bg="#808080", width="20", height="2")
        self.statusLabel.place(x=self.hxw / 2 + self.hxw / 8, y=80)
        # Label to mark status
        self.statusTextLabel = Label(self.subFrameBottom, text="Current Status:", fg="white", bg=BACKGROUND_COLOR)
        self.statusTextLabel.place(x=self.hxw / 2 + self.hxw / 8 + 30, y=50)

    def processIncoming(self):
        '''Handles all messages in the queue'''
        while self.queue.qsize():
            try:
                msg = self.queue.get(0)
                print(msg)
                if msg['id'] == 0:
                    self.velData0.config(text=msg['velocity'])
                    self.berData0.config(text=msg['bearing'])
                    self.latData0.config(text=msg['latitude'])
                    self.lonData0.config(text=msg['longitude'])

                if msg['id'] == 1:
                    self.velData1.config(text=msg['velocity'])
                    self.berData1.config(text=msg['bearing'])
                    self.latData1.config(text=msg['latitude'])
                    self.lonData1.config(text=msg['longitude'])

                if msg['id'] == 2:
                    self.velData2.config(text=msg['velocity'])
                    self.berData2.config(text=msg['bearing'])
                    self.latData2.config(text=msg['latitude'])
                    self.lonData2.config(text=msg['longitude'])

                if msg['id'] == 3:
                    self.velData3.config(text=msg['velocity'])
                    self.berData3.config(text=msg['bearing'])
                    self.latData3.config(text=msg['latitude'])
                    self.lonData3.config(text=msg['longitude'])

            except queue.Empty:
                pass

    def _place_div(self, endCommand):
        button = Button(self.master, text="close GCS", command=endCommand)
        button.place(x=1100, y=500)

    def _place_subFrames(self):
        self.subFrame0 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame0.place(x=0*300, y=5)
        self.subFrame1 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame1.place(x=1*300, y=5)
        self.subFrame2 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame2.place(x=2*300, y=5)
        self.subFrame3 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame3.place(x=3*300, y=5)
        #add more for more baots
        self.subFrameBottom = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw)
        self.subFrameBottom.place(x=300, y=self.hxw - 285)

    def _place_frameLabels(self):
        frameLabel0 = Label(self.subFrame0, text="BOAT_ID 01", fg="white", bg=SUB_FRAME_COLOR)
        frameLabel0.place(x=(self.hxw / 2) / 4 + 15, y=5)
        frameLabel1 = Label(self.subFrame1, text="BOAT_ID 02", fg="white", bg=SUB_FRAME_COLOR)
        frameLabel1.place(x=(self.hxw / 2) / 4 + 15, y=5)
        frameLabel2 = Label(self.subFrame2, text="BOAT_ID 03", fg="white", bg=SUB_FRAME_COLOR)
        frameLabel2.place(x=(self.hxw / 2) / 4 + 15, y=5)
        frameLabel3 = Label(self.subFrame3, text="BOAT_ID 04", fg="white", bg=SUB_FRAME_COLOR)
        frameLabel3.place(x=(self.hxw / 2) / 4 + 15, y=5)

    def _place_velLabels(self):
        Velocity0 = Label(self.subFrame0, text="Velocity (m/s) ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Velocity0.place(x=10, y=40)
        Velocity1 = Label(self.subFrame1, text="Velocity (m/s) ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Velocity1.place(x=10, y=40)
        Velocity2 = Label(self.subFrame2, text="Velocity (m/s) ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Velocity2.place(x=10, y=40)
        Velocity3 = Label(self.subFrame3, text="Velocity (m/s) ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Velocity3.place(x=10, y=40)

    def _place_berLabels(self):
        Bearing0 = Label(self.subFrame0, text="Bearing ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Bearing0.place(x=10, y=80)
        Bearing1 = Label(self.subFrame1, text="Bearing ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Bearing1.place(x=10, y=80)
        Bearing2 = Label(self.subFrame2, text="Bearing ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Bearing2.place(x=10, y=80)
        Bearing3 = Label(self.subFrame3, text="Bearing ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Bearing3.place(x=10, y=80)

    def _place_posLabels(self):
        Lat0 = Label(self.subFrame0, text="Latitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lat0.place(x=10, y=160)
        Lat1 = Label(self.subFrame1, text="Latitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lat1.place(x=10, y=160)
        Lat2 = Label(self.subFrame2, text="Latitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lat2.place(x=10, y=160)
        Lat3 = Label(self.subFrame3, text="Latitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lat3.place(x=10, y=160)

        Lon0 = Label(self.subFrame0, text="Longitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lon0.place(x=10, y=120)
        Lon1 = Label(self.subFrame1, text="Longitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lon1.place(x=10, y=120)
        Lon2 = Label(self.subFrame2, text="Longitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lon2.place(x=10, y=120)
        Lon3 = Label(self.subFrame3, text="Longitude: ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
        Lon3.place(x=10, y=120)

    def _place_dataLabelsVel(self):
        self.velData0 = Label(self.subFrame0, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.velData0.place(x=160, y=40)
        self.velData1 = Label(self.subFrame1, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.velData1.place(x=160, y=40)
        self.velData2 = Label(self.subFrame2, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.velData2.place(x=160, y=40)
        self.velData3 = Label(self.subFrame3, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.velData3.place(x=160, y=40)

    def _place_dataLabelsBer(self):
        self.berData0 = Label(self.subFrame0, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.berData0.place(x=160, y=80)
        self.berData1 = Label(self.subFrame1, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.berData1.place(x=160, y=80)
        self.berData2 = Label(self.subFrame2, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.berData2.place(x=160, y=80)
        self.berData3 = Label(self.subFrame3, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.berData3.place(x=160, y=80)

    def _place_dataLabelsLat(self):
        self.latData0 = Label(self.subFrame0, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.latData0.place(x=160, y=120)
        self.latData1 = Label(self.subFrame1, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.latData1.place(x=160, y=120)
        self.latData2 = Label(self.subFrame2, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.latData2.place(x=160, y=120)
        self.latData3 = Label(self.subFrame3, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.latData3.place(x=160, y=120)


    def _place_dataLabelsLon(self):
        self.lonData0 = Label(self.subFrame0, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.lonData0.place(x=160, y=160)
        self.lonData1 = Label(self.subFrame1, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.lonData1.place(x=160, y=160)
        self.lonData2 = Label(self.subFrame2, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.lonData2.place(x=160, y=160)
        self.lonData3 = Label(self.subFrame3, text=0.0, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
        self.lonData3.place(x=160, y=160)


class threadClient:
    '''responsible for threading, launch of GUI an workers'''
    def __init__(self, master):
        '''Start the GUI'''
        self.master = master

        self.queue = queue.Queue() 

        self.lock = threading.Lock()

        self.gui = GUI(master, self.queue, self._endApplication)

        self.running = 1
        #self.thread1 = threading.Thread(target=self._workerThread1)
        self.thread2 = threading.Thread(target=self._workerThread2)
        #self.thread1.start()
        self.thread2.start()

        self._periodicCall()

    def _periodicCall(self):
        ''' Checks the queue every 100ms'''
        self.gui.processIncoming()
        if not self.running:
            import sys
            sys.exit(1)
        self.master.after(10, self._periodicCall)

    # def _workerThread1(self):
    #     '''starts Udp Multicast listener'''
    #     while self.running:
    #         reciever.run()

    def _workerThread2(self):
        '''Handles asynchronous I/O ''' 
        print("Using multicast group: {}:{}".format(MCAST_GROUP, MCAST_PORT))
        listener = MulticastListener(MCAST_GROUP, MCAST_PORT, timeout=TIMEOUT)
        
        while self.running:
            try:
                msg = listener.listen()
                print(msg)
                msgType = Interpreter.header2GCS(msg)
                print(msgType)
                if msgType == MsgType.ODOMETRY:
                    data = Interpreter.odometry2GCS(msg)
                    self.queue.put(data)
                # if msgType == MsgType.BOAT_STATUS:
                #     command = Interpreter.status2GCS
                #add fetch of data here
                #msg = {'id': rand.randrange(4), 'velocity': rand.random(), 'bearing': rand.random(), 'latitude': rand.random(), 'longitude': rand.random()}
            except KeyboardInterrupt as e:
                print("Exiting with error: {}".format(e))
                self._endApplication()
                
            except socket.timeout:
                pass

    def _endApplication(self):
        self.running = 0

rand = random.Random()
root = tkinter.Tk()

def main():

    client = threadClient(root)

    root.mainloop()


if __name__ == "__main__":
    main()