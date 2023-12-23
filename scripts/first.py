from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import geopy.distance

import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

start = [50.450739, 30.461242]
end = [50.443326, 30.448078, 100]

connection_string = args.connect

distance = geopy.distance.geodesic(start[:2], end[:2])

print(f"Distance: {distance.km} km")
print("Connection to the vehicle on %s" % connection_string)
# vehicle = connect(connection_string, wait_ready=True, baud=57600)
vehicle = connect('udp:127.0.0.1:14550', baud=57600)

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



# -- Define the function for takeoff
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


# ------MAIN PROGRAM--------
arm_and_takeoff(10)
condition_yaw(0, relative=True)
time.sleep(10)
condition_yaw(180, relative=True)
time.sleep(10)
condition_yaw(90)
time.sleep(10)

# -- set the default speed
vehicle.airspeed = 7

# -- Go to wpl
print("go to wpl")
wpl = LocationGlobalRelative(50.443326, 30.448078, 100)

vehicle.simple_goto(wpl, groundspeed=30)

#-- Here you can do all your magic...
old_point = vehicle.location.global_relative_frame
left = distance
while True:
    old_left = left
    new_point = vehicle.location.global_relative_frame
    left = geopy.distance.geodesic(end[:2], [old_point.lat, old_point.lon])

    print(new_point, vehicle.mode,
          f"Left: {left.m} m",
          f"Speed: {geopy.distance.geodesic([new_point.lat, new_point.lon], [old_point.lat, old_point.lon]).m} m/s")
    if left.m < 0.2:
        condition_yaw(350)
        # vehicle.mode = VehicleMode("STABILIZE")
        break
    if vehicle.mode.name != "GUIDED":
        vehicle.mode = vehicle.mode = VehicleMode("GUIDED")
    if left > old_left:
        print("Recursing!")
        vehicle.simple_goto(wpl, groundspeed=30)
    old_point = new_point
    time.sleep(1)


time.sleep(10)

time.sleep(20)

#-- Close connection
vehicle.close()


