class PID():
    def __init__(self):
        self.kp_acc = 0.25
        self.ki_acc = 0.15
        self.kd_acc = 0
        self.kp_brake = 0.25
        self.ki_brake = 0.15
        self.kd_brake = 0
        self.min_acc = 0
        self.max_acc = 1
        self.min_brake = 0
        self.max_brake = 1
        self.max_steer = 1
        self.max_steerwheel = 500
        self.deadzone_acc = 0
        self.deadzone_brake = 0.1
        self.deadzone_steer = 0

        self.throttle = 0
        self.brake = 0.1
        self.steer = 0

        self.time_step = 0.03
        self.interg = 0
        self.error_buffer = []
        self.sep_ratio = 0.5

    def tick(self, error_lon, aim_steer):
        interg = self._intergration(error_lon)
        diff = self._differential(error_lon)

        self.brake = - self.kp_brake * error_lon \
            - self.ki_brake * interg \
            - self.kd_brake * diff
        self.throttle = self.kp_acc * error_lon \
            + self.ki_acc * interg \
            + self.kd_acc * diff
        self.throttle = 0 if self.brake > 0 else self.throttle

        self.throttle = self.min_acc if self.throttle < self.deadzone_acc else self.throttle
        self.throttle = self.max_acc if self.throttle > self.max_acc else self.throttle

        self.brake = self.min_brake if self.brake < self.deadzone_brake else self.brake
        self.brake = self.max_brake if self.brake > self.max_brake else self.brake

        self.steer = aim_steer / self.max_steerwheel
        self.steer = self.max_steer if self.steer > self.max_steer else self.steer
        self.steer = - self.max_steer if self.steer < - self.max_steer else self.steer

    def _differential(self, error):
        self.error_buffer.append(error)
        while len(self.error_buffer) >= 3:
            self.error_buffer.pop(0)

        rbuffer = self.error_buffer[::-1]
        length = len(rbuffer)
        if length >= 3:
            result = (rbuffer[length - 3] - 4 * rbuffer[length - 2] +
                      3 * rbuffer[length - 1]) / (2 * self.time_step)
        elif length >= 2:
            result = (rbuffer[length - 1] -
                      rbuffer[length - 2]) / self.time_step
        else:
            result = 0

        return result

    def _intergration(self, error):
        temp = self.interg + error * self.time_step
        if error > 0:
            if temp * self.kp_acc > self.max_acc * self.sep_ratio:
                pass
            else:
                self.interg = temp
        else:
            if temp * self.kp_brake < -self.max_brake * self.sep_ratio:
                pass
            else:
                self.interg = temp

        return self.interg
