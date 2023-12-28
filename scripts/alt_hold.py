from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative


connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('udp:127.0.0.1:14550', baud=57600, wait_ready=True)

# Get all original channel values (before override)
print("Channel values from RC Tx:", vehicle.channels)