# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:

    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        self._kp = float(kp)
        self._ki = float(ki)
        self._kd = float(kd)
        self._max_windup = float(max_windup)
        self._sum_error = 0.0
        self._prev_error = 0.0
        self._prev_timestamp = 0.0

        self._target = 0.0

    def reset(self):
        self._sum_error = 0.0
        self._prev_error = 0.0
        self._prev_timestamp = 0.0

        self._target = 0.0

    def setTarget(self, target):
        self._target = float(target)

    def setKP(self, kp):
        self._kp = float(kp)

    def setKI(self, ki):
        self._ki = float(ki)

    def setKD(self, kd):
        self._kd = float(kd)

    def setMaxWindup(self, max_windup):
        self._max_windup = float(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self._prev_timestamp
        if delta_time <= 0:
            return None
        
        error = self._target - measured_value
        
        self._sum_error += error * delta_time
        # restrict to -max_windup <= sum_error <= max_windup
        self._sum_error = max(-self._max_windup, min(self._sum_error ,self._max_windup))

        delta_error = error - self._prev_error

        self._prev_error = error
        self._prev_timestamp = timestamp

        # control value u
        p = self._kp * error
        i = self._ki * self._sum_error
        d = self._kd * delta_error / delta_time
        u = p + i + d


        return u



