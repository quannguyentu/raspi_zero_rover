# import curses and GPIO
import asyncio
import math
import asyncio
import math
import os
import signal
import subprocess
import sys
import threading
import time
import RPi.GPIO as GPIO
from evdev import InputDevice, ecodes, ff, list_devices
from controller import gamepad

#set GPIO numbering mode and define output pins
GPIO.setmode(GPIO.BOARD)
DIR1 = 7
GPIO.setup(DIR1, GPIO.OUT)
DIR2 = 11
GPIO.setup(DIR2, GPIO.OUT)
PWM1_PIN = 13
GPIO.setup(PWM1_PIN, GPIO.OUT)
PWM1 = GPIO.PWM(PWM1_PIN, 1000)
PWM1.start(0)
PWM2_PIN = 15
GPIO.setup(PWM2_PIN, GPIO.OUT)
PWM2 = GPIO.PWM(PWM2_PIN, 1000)
PWM2.start(0)
NOT_FAULT = 29
GPIO.setup(NOT_FAULT, GPIO.OUT)

#Enable motor
GPIO.output(NOT_FAULT, True)

#Get controller
def connect():  # asyncronus read-out of events
    xbox_path = None
    remote_control = None
    devices = [InputDevice(path) for path in list_devices()]
    print('Connecting to xbox controller...')
    for device in devices:
        if str.lower(device.name) == 'xbox wireless controller':
            xbox_path = str(device.path)
            remote_control = gamepad.gamepad(file=xbox_path)
            remote_control.rumble_effect = 2
            return remote_control
    return None

Controller = connect()
async def read_gamepad_inputs():
    # user_input = Controller.read()
    x = Controller.LeftJoystickX
    y = Controller.LeftJoystickY
    direction = lambda d: True if x > 0 else False
    GPIO.output(DIR1, direction(-x))
    GPIO.output(DIR2, direction(-x))
    if y > 0:
        PWM1.ChangeDutyCycle(100)
        PWM2.ChangeDutyCycle(y)
    if y < 0:
        PWM1.ChangeDutyCycle(abs(y))
        PWM2.ChangeDutyCycle(100)




try:
        loop = asyncio.get_event_loop()
        tasks = [Controller.read_gamepad_input()]
        loop.run_until_complete(asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED))

            # elif char == curses.KEY_DOWN:
            #     GPIO.output(DIR1, True)
            #     GPIO.output(DIR2, False)
            #     GPIO.output(PWM1, True)
            #     GPIO.output(PWM2, False)
            # elif char == curses.KEY_RIGHT:
            #     GPIO.output(DIR1, True)
            #     GPIO.output(DIR2, False)
            #     GPIO.output(PWM1, False)
            #     GPIO.output(PWM2, True)
            # elif char == curses.KEY_LEFT:
            #     GPIO.output(DIR1, False)
            #     GPIO.output(DIR2, True)
            #     GPIO.output(PWM1, True)
            #     GPIO.output(PWM2, False)
            # elif char == 10:
            #     GPIO.output(DIR1, False)
            #     GPIO.output(DIR2, False)
            #     GPIO.output(PWM1, False)
            #     GPIO.output(PWM2, False)
             
finally:
    GPIO.cleanup()
    

