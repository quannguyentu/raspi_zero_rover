import pigpio

_pi = pigpio.pi()
if not _pi.connected:
    raise IOError("Can't connect to pigpio")

# Motor speeds for this library are specified as numbers between -MAX_SPEED and
# MAX_SPEED, inclusive.
# This has a value of 480 for historical reasons/to maintain compatibility with
# older libraries for other Pololu boards (which used WiringPi to set up the
# hardware PWM directly).
_max_speed = 480
MAX_SPEED = _max_speed

_pin_nEN = 10
_pin_nFAULT1 = 6
_pin_nFAULT2 = 5
_pin_M1PWM = 12
_pin_M2PWM = 13
_pin_M1DIR = 24
_pin_M2DIR = 25


class Motor(object):
    MAX_SPEED = _max_speed

    def __init__(self, pwm_pin, dir_pin):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        _pi.write(self.dir_pin, dir_value)
        _pi.hardware_PWM(self.pwm_pin, 20000, int(speed * 6250 / 3));
        # 20 kHz PWM, duty cycle in range 0-1000000 as expected by pigpio


class Motors(object):
    MAX_SPEED = _max_speed

    def __init__(self):
        self.motor1 = Motor(_pin_M1PWM, _pin_M1DIR)
        self.motor2 = Motor(_pin_M2PWM, _pin_M2DIR)

        _pi.set_pull_up_down(_pin_nFAULT1, pigpio.PUD_UP)
        _pi.set_pull_up_down(_pin_nFAULT2, pigpio.PUD_UP)  # make sure nFAULT is pulled up
        _pi.write(_pin_nEN, 0)  # enable drivers by default

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)

    def getFault(self):
        return not _pi.read(_pin_nFAULT1)

    def enable(self):
        _pi.write(_pin_nEN, 0)

    def disable(self):
        _pi.write(_pin_nEN, 1)

    def forceStop(self):
        # reinitialize the pigpio interface in case we interrupted another command
        # (so this method works reliably when called from an exception handler)
        global _pi
        _pi.stop()
        _pi = pigpio.pi()
        self.setSpeeds(0, 0)


motors = Motors()
