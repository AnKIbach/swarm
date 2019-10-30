import tkinter
from tkinter import Frame, Label, RAISED
import time
import threading
import queue

import random

BOATS_IN_SWARM = 4
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

        self._place_subFrames()

        self.statusLabel = Label(self.subFrameBottom, text="NOT VERIFIED", fg="orange", bg="#808080", width="20", height="2")
        self.statusLabel.place(x=self.hxw / 2 + self.hxw / 8, y=80)
        # Label to mark status
        self.statusTextLabel = Label(self.subFrameBottom, text="Current Status:", fg="white", bg=BACKGROUND_COLOR)
        self.statusTextLabel.place(x=self.hxw / 2 + self.hxw / 8 + 30, y=50)

        #----------SubFrameLeft Label:-----------#

        for boat in range(BOATS_IN_SWARM):
            print(boat)
            # self.subFrame[boat].place(x=boat*300, y=5)

            # self.frameLabel = Label(self.subFrame[boat], text="B0"+str(boat), fg="white", bg=SUB_FRAME_COLOR)
            # self.frameLabel.place(x=(self.hxw / 2) / 4 + 15, y=5)

            # self.Velocity = Label(self.subFrame[boat], text="Velocity (m/s) ", fg="white", bg=BACKGROUND_COLOR, width=STAND_TEXT_WIDTH)
            # self.Velocity.place(x=10, y=40)

    def processIncoming(self):
        '''Handles all messages in the queue'''
        while self.queue.qsize():
            try:
                msg = self.queue.get(0)
                #check content here
                # Velocity_01DataLabel = Label(self.subFrame[0], text=msg, fg="white", bg=BACKGROUND_COLOR, width=STAND_DATA_WIDTH)
                # Velocity_01DataLabel.place(x=160, y=40)

            except queue.Empty:
                pass

    def _place_subFrames(self):
        self.subFrame1 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame1.place(x=0*300, y=5)
        self.subFrame1 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame1.place(x=1*300, y=5)
        self.subFrame2 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame2.place(x=2*300, y=5)
        self.subFrame3 = Frame(self.master, bg="#3C3F41", height="300", width=self.hxw / 2 - 5, relief=RAISED)
        self.subFrame3.place(x=3*300, y=5)
        #add more for more baots
        self.subFrameBottom = Frame(master, bg="#3C3F41", height="300", width=self.hxw)
        self.subFrameBottom.place(x=300, y=self.hxw - 285)

class threadClient:
    '''responsible for threading, launch of GUI an workers'''
    def __init__(self, master):
        '''Start the GUI'''
        self.master = master

        self.queue = queue.Queue() 

        self.gui = GUI(master, self.queue, self._endApplication)

        self.running = 1
        self.thread1 = threading.Thread(target=self._workerThread1)
        self.thread1.start()

        self._periodicCall()

    def _periodicCall(self):
        ''' Checks the queue every 100ms'''
        self.gui.processIncoming()
        if not self.running:
            import sys
            sys.exit(1)
        self.master.after(100, self._periodicCall)

    def _workerThread1(self):
        '''Handles asynchronous I/O '''
        while self.running:
            #add fetch of data here
            time.sleep(0.7)

            msg = rand.random()
            print(msg   )
            self.queue.put(msg)

    def _endApplication(self):
        self.running = 0

rand = random.Random()
root = tkinter.Tk()

def main():

    client = threadClient(root)

    root.mainloop()


if __name__ == "__main__":
    main()



