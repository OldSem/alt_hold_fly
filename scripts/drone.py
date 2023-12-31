from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import geopy.distance
import math


def radians(degrees):
    return math.radians(degrees if degrees < 180 else degrees - 360)


def degrees(radians):
    return math.degrees(radians if radians > 0 else 2 * math.pi + radians)


class Action:
    delta = 0
    delta_grad = 50

    def __init__(self, channel, value=1500):
        self.channel = channel
        self.default = value

    @property
    def value(self):
        return self.default + self.delta

    def correct(self):
        print("correct ", self.delta)
        self.delta = int(-(self.delta / abs(self.delta) * self.delta_grad)) if self.delta != 0 else 0
        print("corrected ", self.delta)

    def faster(self):
        self.delta = int(self.delta / abs(self.delta) * (abs(self.delta + self.delta_grad))
                         ) if self.delta != 0 else self.delta_grad

    def slower(self):
        self.delta = int(self.delta / abs(self.delta) * (abs(self.delta - self.delta_grad))
                         ) if self.delta != 0 else -self.delta_grad


class Drone:
    roll = Action(channel=1, value=1500)
    pitch = Action(channel=2, value=1500)
    throttle = Action(channel=3, value=1500)
    yaw = Action(channel=4, value=1500)

    wpl: LocationGlobalRelative
    last_distance: float
    last_delta: float
    need_yaw: float

    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        try:
            self.vehicle = connect(connection_string, baud=57600)
        except:
            self.vehicle = None

    @property
    def position(self):
        return self.vehicle.location.global_relative_frame

    @property
    def attitude(self):
        return self.vehicle.attitude

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

    def push_channels(self):
        print('push ', self.channels, self.left, self.position, degrees(self.attitude.yaw))
        self.vehicle.channels.overrides = self.channels

    def stabilize(self):
        for movement in [self.roll, self.pitch, self.throttle, self.yaw]:
            movement.delta = 0
        self.push_channels()

    def arming(self):
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

    def set_direction(self, action):
        self.last_distance = self.left
        self.last_delta = self.delta
        action.faster()
        for i in range(5):
            self.push_channels()
            time.sleep(1)
        print(self.left, self.last_distance, self.delta, self.last_delta)
        if self.left > self.last_distance or self.delta < self.last_delta:
            action.correct()
            self.push_channels()
            time.sleep(1)

    def dummy_goto(self, wpl):
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.turn_off(10)
        self.wpl = LocationGlobalRelative(*wpl)
        print(self.left, self.left < 0.2)
        while self.left > 0.2:
            self.set_direction(self.roll)
            self.set_direction(self.pitch)

    def turn_off(self, alt):
        while True:
            self.throttle.delta = 400
            self.push_channels()
            time.sleep(1)
            if self.position.alt >= alt:
                self.throttle.delta = 0
                self.push_channels()
                break

    @staticmethod
    def correct_movement(movement, start, stop, value=1):
        values = {100: 1,
                  10: 3,
                  1: 6,
                  0: 9}
        difference = stop - start
        value = int(value / values.get(max([i for i in values if abs(difference) >= i])))
        if 0 < abs(difference) < 0.1:
            movement.delta = 0
        elif difference > 0:
            movement.delta = value
        else:
            movement.delta = -value
        print(difference, movement.delta, start, stop)

    def correct_yaw(self, yaw):
        # self.correct_movement(self.yaw, self.attitude.yaw * 10, yaw * 10,  25
        self.yaw.delta = 25 if yaw > 0 else -25
        self.push_channels()
        time.sleep(0.2)
        self.yaw.delta = 0
        self.push_channels()
        time.sleep(1)


    def correct_direction(self):
        movements = {
            self.roll: {'movement': 'lon', 'factor': 100000, 'delta': 300},
            self.pitch: {'movement': 'lat', 'factor': 100000, 'delta': -300},
            self.throttle: {'movement': 'alt', 'factor': 50, 'delta': 400},
        }
        for movement, value in movements.items():
            self.correct_movement(movement,
                                  getattr(self.position, value.get('movement')) * value.get('factor', 1),
                                  getattr(self.wpl, value.get('movement')) * value.get('factor', 1),
                                  value.get('delta'))
        self.push_channels()
        time.sleep(0.5)

    def careful_goto(self, wpl, yaw=0):
        self.arming()
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        self.wpl = LocationGlobalRelative(*wpl)

        while self.left > 0.5 or abs((self.wpl.alt - self.position.alt)) > 0.2:
            self.correct_direction()
        self.stabilize()

        while abs(yaw - degrees(self.attitude.yaw)) > 1:
            self.correct_yaw(radians(yaw))
        self.stabilize()
