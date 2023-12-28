from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time

connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('udp:127.0.0.1:14550', baud=57600)


# Get all original channel values (before override)
while True:
    print("Channel values from RC Tx:", vehicle.channels, vehicle.mode, vehicle.location.global_relative_frame)
    time.sleep(1)
