#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 日本語コメント使うには先頭に　# coding: UTF-8を書く。
#
# 1) cygwinコンソールから、sim_vehicle.pyを実行してSITLを起動する。
# 2) SITLで起動したMAVProxyコンソールにてポートを追加する。
#    ※デフォルトポートはMissionPlannerとの接続で使用するため。
#    $output add 127.0.0.1:14552
# 3) このコードを実行する。
"""
© Copyright 2015-2016, 3D Robotics.
"""
from __future__ import print_function
#
# This is a small example of the python drone API - an ultra minimal GCS
#

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from Tkinter import *

# The tkinter root object
global root

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Tracks GPS position of your computer (Linux only). Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# 引数が与えられない場合の接続先
if not connection_string:
    connection_string = "127.0.0.1:14552"

# dronekit-sitlに接続
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    print("*** not connection string!!! ***" )
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def setMode(mode):
    # Now change the vehicle into auto mode
    vehicle.mode = VehicleMode(mode)

# Arm/Disarmする
def Arming(state):
    vehicle.armed = state
    while not (vehicle.armed == state) and not api.exit:
        time.sleep(1) 


def updateGUI(label, value):
    label['text'] = value

def addObserverAndInit(name, cb):
    """We go ahead and call our observer once at startup to get an initial value"""
    vehicle.add_attribute_listener(name, cb)

root = Tk()
root.wm_title("microGCS - the worlds crummiest GCS")
frame = Frame(root)
frame.pack()

locationLabel = Label(frame, text = "No location", width=60)
locationLabel.pack()
attitudeLabel = Label(frame, text = "No Att", width=60)
attitudeLabel.pack()
modeLabel = Label(frame, text = "mode")
modeLabel.pack()

addObserverAndInit('attitude', lambda vehicle, name, attitude: updateGUI(attitudeLabel, vehicle.attitude))
addObserverAndInit('location', lambda vehicle, name, location: updateGUI(locationLabel, str(location.global_frame)))
addObserverAndInit('mode', lambda vehicle,name,mode: updateGUI(modeLabel, mode))

Button(frame, text = "Auto", command = lambda : setMode("AUTO")).pack()
Button(frame, text = "RTL", command = lambda : setMode("RTL")).pack()
Button(frame, text = "Guided", command = lambda : setMode("GUIDED")).pack()
Button(frame, text = "Manual", command = lambda : setMode("MANUAL")).pack()
Button(frame, text = "Arm", command = lambda : Arming(True)).pack()



root.mainloop()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()