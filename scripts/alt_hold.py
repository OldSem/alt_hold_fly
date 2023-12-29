from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import geopy.distance

connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('udp:127.0.0.1:14550', baud=57600)
start_position = vehicle.location.global_relative_frame
ROLL = 1500
PITCH = 1500
THROTTLE = 1500
YAW = 1500

def dummy_goto(wpl):

    left = geopy.distance.geodesic([wpl.lat, wpl.lon], [start_position.lat, start_position.lon])

    while True:
        old_left = left
        time.sleep(1)
        position = vehicle.location.global_relative_frame
        left = geopy.distance.geodesic([wpl.lat, wpl.lon], [position.lat, position.lon])


vehicle.close()
