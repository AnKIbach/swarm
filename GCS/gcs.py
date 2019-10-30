#!/usr/bin/env python
import datetime
import os
import time
import serial

from threading import Thread

from tkinter import *
from tkinter import messagebox

from Classes.Objects import Status
from Classes.Objects import Odometry
from Classes.Objects import Command

def main():
    GCS = Thread(target=visu)
    data = Thread(target=fetcher)

    data.start()
    GCS.start()

def fetcher():
    print(3)

def visu():
    #Variables to get
    Velocity=0,0
    Bearing=0,0
    Long=0,0
    Lat=0,0



    # Variables B02
    Velocity_02 = 1
    Bearing_02 = 2
    Long_02 = 2
    Lat_02 = 2

    # Variables B03
    Velocity_03 = 3
    Bearing_03 = 3
    Long_03 = 3
    Lat_03 = 3

    # Variables B04
    Velocity_04 = 4
    Bearing_04 = 4
    Long_04 = 4
    Lat_04 = 4

    # Set window options
    top = Tk()
    top.geometry("1200x600")

    while True:
        # Close menu window method
        def close_window(window):
            window.destroy()

        # ===== GLOBAL VARIABLES ===== #
        is_launched = False  # has the rocket launched?
        has_aborted = False  # has the process been aborted?
        

        #-----------Velocity Entry Label---------------------#
        VelocityEntryLabel = Label(subFrameBottom, text="Positive speed (0-10)", fg="white", bg=subFrameColor,width=26)
        VelocityEntryLabel.place(x=40, y=130)
        VelocityEntry = Entry(subFrameBottom, bd=5, bg=bgColor, fg="white", width=standardDataWidth, textvariable=Velocity)
        VelocityEntry.place(x=40, y=160)

        def getVelocity():
            global Velocity
            Velocity = VelocityEntry.get()
            if 0 < float(Velocity) < 10:
                
                Velocity_02DataLabel.config(text=Velocity)
                Velocity_03DataLabel.config(text=Velocity)
                Velocity_04DataLabel.config(text=Velocity)

        VelocityInputButton = Button(subFrameBottom, text="ENTER", width=8, command=getVelocity)
        VelocityInputButton.place(x=160, y=160)

        #-----------Bearing Entry Label-----------------#
        BearingEntryLabel = Label(subFrameBottom, text="Positive angle (0-360)", fg="white", bg=subFrameColor,width=26)
        BearingEntryLabel.place(x=40, y=200)
        BearingEntry = Entry(subFrameBottom, bd=5, bg=bgColor, fg="white", width=standardDataWidth, textvariable=Bearing)
        BearingEntry.place(x=40, y=230)

        def getBearing():
            global Bearing
            Bearing = BearingEntry.get()
            if 0 < float(Bearing) < 360:
                Bearing_01DataLabel.config(text=Bearing)
                Bearing_02DataLabel.config(text=Bearing)
                Bearing_03DataLabel.config(text=Bearing)
                Bearing_04DataLabel.config(text=Bearing)

        BearingInputButton = Button(subFrameBottom, text="ENTER", width=8, command=getBearing)
        BearingInputButton.place(x=160, y=230)

        #-----------Long Entry Label-----------------#
        LongEntryLabel = Label(subFrameBottom, text="Longitude coordinate(-180-180): ", fg="white", bg=subFrameColor,width=26)
        LongEntryLabel.place(x=340, y=130)
        LongEntry = Entry(subFrameBottom, bd=6, bg=bgColor, fg="white", width=standardDataWidth, textvariable=Long)
        LongEntry.place(x=340, y=160)

        def getLong():
            global Long
            Long = LongEntry.get()
            if -180 < float(Long) < 180:
                Long_01DataLabel.config(text=Long)
                Long_02DataLabel.config(text=Long)
                Long_03DataLabel.config(text=Long)
                Long_04DataLabel.config(text=Long)

        LongInputButton = Button(subFrameBottom, text="ENTER", width=8, command=getLong)
        LongInputButton.place(x=420, y=160)

        #-------Destroy window function-------#
        def _delete_window():
            if messagebox.askokcancel("Close GUI", "Do you want to close GUI?"):
                top.destroy()

        #--------Lat Entry Label-----------------#
        LatEntryLabel = Label(subFrameBottom, text="Latitude coordinate(-90-90): ", fg="white", bg=subFrameColor,width=26)
        LatEntryLabel.place(x=340, y=200)
        LatEntry = Entry(subFrameBottom, bd=5, bg=bgColor, fg="white", width=standardDataWidth, textvariable=Lat)
        LatEntry.place(x=340, y=230)

        def getLat():
            global Lat
            Lat = LatEntry.get()
            if -90 < float(Lat) < 90:
                Lat_01DataLabel.config(text=Lat)
                Lat_02DataLabel.config(text=Lat)
                Lat_03DataLabel.config(text=Lat)
                Lat_04DataLabel.config(text=Lat)

        LatInputButton = Button(subFrameBottom, text="ENTER", width=8, command=getLat)
        LatInputButton.place(x=420, y=230)

        # == UPDATE LABEL FUNCTIONS == #
        def statusLabelChange(change_to):
            statusLabel.config(text=change_to)
            if change_to == "NOT VERIFIED":
                statusLabel.config(fg="orange")
            elif change_to == "MISSION ABORTED":
                statusLabel.config(fg="red")

        def abortMessageCallBack():
            abort_response = messagebox.askyesno("Abort Mission?", "Do you really want to abort the mission?")
            if abort_response:
                global has_aborted
                has_aborted = True
                statusLabelChange("MISSION ABORTED")
                abortButton.config(state=DISABLED)
                #setter farten til 0 her
            else:
                has_aborted = False

        # ==== BUTTONS AND ENTRIES === #
        infoText = Label(subFrameBottom, fg="white", bg=bgColor, width=40)
        infoText.place(x=160, y=15)

        def on_enter_abort(event):
            infoText.config(text="Abort Mission Button", fg="red")

        def on_leave(event):
            global infoText
            infoText.config(text=" ")

        abortButton = Button(subFrameBottom, text="ABORT MISSION", state=DISABLED, bg="red", command=abortMessageCallBack,
                            width="20")
        abortButton.place(x=100, y=55)
        abortButton.bind("<Enter>", on_enter_abort)
        abortButton.bind("<Leave>", on_leave)

        top.protocol("WM_DELETE_WINDOW", _delete_window)
        # Start window
        top.update()


if __name__=="__main__":
    main()