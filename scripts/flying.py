"""

set_attitude_target.py: (Copter Only)

This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.

Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.

Tested in Python 2.7.10

"""

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import geopy.distance

import threading

# Set up option parsing to get connection string
import argparse


sitl = None

ALTITUDE = 100

current_thrust = 0.5
connection_string = 'udp:127.0.0.1:14550'

# Connect to the Vehicle
print('Connecting to vehicle on: udp:127.0.0.1:14550 %s' % connection_string)
vehicle = connect('udp:127.0.0.1:14550', baud=57600)


def arm_and_takeoff(tgt_altitude):
    print("Arming motors")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(1)

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Takeoff", vehicle.mode)
    vehicle.simple_takeoff(tgt_altitude)

    # -- wait to reach the target altitude

    while True:
        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)
        altitude = vehicle.location.global_relative_frame.alt
        print(vehicle.location.global_relative_frame, vehicle.mode)
        if altitude >= tgt_altitude - 1:

            print("Altitude reached")
            break
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)


def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def fly_to_wpl(wpl):
    # -- Go to wpl
    print("go to wpl")
    vehicle.simple_goto(wpl, groundspeed=30)

    while True:

        position = vehicle.location.global_relative_frame
        left = geopy.distance.geodesic([wpl.lat, wpl.lon], [position.lat, position.lon])

        print(position, vehicle.mode,
              f"Left: {left.m} m")
        if left.m < 0.2:

            # vehicle.mode = VehicleMode("STABILIZE")
            break
        if vehicle.mode.name != "GUIDED":
            vehicle.mode = vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)




# Take off 2.5m in GUIDED_NOGPS mode.

TARGET_ALTITUDE = 100

arm_and_takeoff(TARGET_ALTITUDE)

wpl = LocationGlobalRelative(50.443326, 30.448078, 100)

fly_to_wpl(wpl)

condition_yaw(350)
time.sleep(30)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("RTL")
time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")