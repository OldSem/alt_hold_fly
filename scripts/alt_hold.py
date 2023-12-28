from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative


connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Get all original channel values (before override)
print("Channel values from RC Tx:", vehicle.channels)