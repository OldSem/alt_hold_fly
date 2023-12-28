from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time

connection_string = 'udp:127.0.0.1:14550'

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('udp:127.0.0.1:14550', baud=57600)


# Get all original channel values (before override)
# Override channels
print("\nChannel overrides: %s" % vehicle.channels.overrides)

print("Set Ch2 override to 200 (indexing syntax)")
vehicle.channels.overrides['2'] = 200
print(" Channel overrides: %s" % vehicle.channels.overrides)
print(" Ch2 override: %s" % vehicle.channels.overrides['2'])

print("Set Ch3 override to 300 (dictionary syntax)")
vehicle.channels.overrides = {'3':300}
print(" Channel overrides: %s" % vehicle.channels.overrides)

print("Set Ch1-Ch8 overrides to 110-810 respectively")
vehicle.channels.overrides = {'1': 110, '2': 210,'3': 310,'4':4100, '5':510,'6':610,'7':710,'8':810}
print(" Channel overrides: %s" % vehicle.channels.overrides)


# Clear override by setting channels to None
print("\nCancel Ch2 override (indexing syntax)")
vehicle.channels.overrides['2'] = None
print(" Channel overrides: %s" % vehicle.channels.overrides)

print("Clear Ch3 override (del syntax)")
del vehicle.channels.overrides['3']
print(" Channel overrides: %s" % vehicle.channels.overrides)

print("Clear Ch5, Ch6 override and set channel 3 to 500 (dictionary syntax)")
vehicle.channels.overrides = {'5':None, '6':None,'3':500}
print(" Channel overrides: %s" % vehicle.channels.overrides)

print("Clear all overrides")
vehicle.channels.overrides = {}
print(" Channel overrides: %s" % vehicle.channels.overrides)

#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()
