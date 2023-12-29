from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import geopy.distance


class Action:
    delta = 10

    def __init__(self, channel, value):
        self.channel = channel
        self.value = value

    def correct(self):
        self.delta = -self.delta
        self.value += self.delta * 2

    def faster(self):
        self.value += self.value

    def slower(self):
        self.value -= self.value


class Drone:
    roll = Action(channel=1, value=1500)
    pitch = Action(channel=2, value=1500)
    throttle = Action(channel=3, value=1500)
    yaw = Action(channel=4, value=1500)

    wpl: LocationGlobalRelative
    last_distance: float
    last_delta: float

    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        try:
            self.vehicle = connect(connection_string, baud=57600)
        except:
            self.vehicle = None

    @property
    def position(self):
        return self.vehicle.location.global_relative_frame

    @property
    def left(self):
        return geopy.distance.geodesic([self.wpl.lat, self.wpl.lon],
                                       [self.position.lat, self.position.lon]).m

    @property
    def delta(self):
        return self.last_distance - self.left

    @property
    def channels(self):
        return {action.channel: action.value for action in [self.roll, self.pitch, self.throttle, self.yaw]}

    def set_direction(self, action):
        self.last_distance = self.left
        print(self.left)
        self.last_delta = self.last_delta
        action.faster()
        time.sleep(0.5)
        if self.left > self.last_distance or self.delta < self.last_delta:
            action.correct()
        self.last_distance = self.left
        self.last_delta = self.last_delta

    def dummy_goto(self, wpl):
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.turn_off(10)
        self.wpl = LocationGlobalRelative(*wpl)
        print(self.left, self.left < 0.2)
        while self.left < 0.2:
            self.set_direction(self.roll)
            self.set_direction(self.pitch)

    def turn_off(self, alt):
        while True:
            self.throttle.value = 1900
            self.vehicle.channels.overrides = self.channels
            time.sleep(1)
            if self.position.alt >= alt:
                self.throttle.value = 1500
                self.vehicle.channels.overrides = self.channels
                break

