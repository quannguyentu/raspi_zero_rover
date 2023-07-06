# import curses and GPIO
import asyncio
from subprocess import call
import os
import signal
import subprocess
import sys
import threading
import time
import RPi.GPIO as GPIO
from evdev import InputDevice, ecodes, ff, list_devices
from controller import gamepad
from Motor import motors, MAX_SPEED


# Define a custom exception to raise if a fault is detected.
class DriverFault(Exception):
    pass


def raiseIfFault():
    if motors.getFault():
        raise DriverFault


# # set GPIO numbering mode and define output pins
# GPIO.setmode(GPIO.BOARD)
# DIR1 = 18
# GPIO.setup(DIR1, GPIO.OUT)
# DIR2 = 22
# GPIO.setup(DIR2, GPIO.OUT)
# PWM1_PIN = 32
# GPIO.setup(PWM1_PIN, GPIO.OUT)
# PWM1 = GPIO.PWM(PWM1_PIN, 20000)
# PWM1.start(0)
# PWM2_PIN = 33
# GPIO.setup(PWM2_PIN, GPIO.OUT)
# PWM2 = GPIO.PWM(PWM2_PIN, 20000)
# PWM2.start(0)
# NOT_FAULT_M1 = 36
# NOT_FAULT_M2 = 37
# GPIO.setup(NOT_FAULT_M1, GPIO.OUT)
# GPIO.setup(NOT_FAULT_M2, GPIO.OUT)
#
# # Enable motor
# GPIO.output(NOT_FAULT_M1, True)
# GPIO.output(NOT_FAULT_M2, True)


# Get controller
def connect():  # asyncronus read-out of events
    xbox_path = None
    Gamepad = None
    devices = [InputDevice(path) for path in list_devices()]
    print('Connecting to xbox controller...')
    for device in devices:
        if str.lower(device.name) == 'xbox wireless controller':
            xbox_path = str(device.path)
            Gamepad = gamepad(file=xbox_path)
            print("connected")
            return Gamepad
    return None


def has_duplicates_boolean(value, lst):
    count = lst.count(value)
    if count >= 2:
        return True
    else:
        return False


async def read_gamepad_inputs(Controller):
    while Controller.button_b is False:
        up = Controller.dpad_up
        down = Controller.dpad_down
        left = Controller.dpad_left
        right = Controller.dpad_right
        direction = [up, down, left, right]
        print(direction)
        if direction != [False, False, False, False]:
            if has_duplicates_boolean(True, direction):
                motors.setSpeeds(0, 0)
            else:
                if direction[0] is True:
                    motors.setSpeeds(MAX_SPEED, MAX_SPEED)
                elif direction[1] is True:
                    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED)
                elif direction[2]:
                    motors.setSpeeds(-MAX_SPEED, MAX_SPEED)
                elif direction[3]:
                    motors.setSpeeds(MAX_SPEED, -MAX_SPEED)
        else:
            motors.setSpeeds(0, 0)

        await asyncio.sleep(0.005)


async def removetasks(loop):
    tasks = [t for t in asyncio.all_tasks() if t is not
             asyncio.current_task()]

    for task in tasks:
        # skipping over shielded coro still does not help
        if task._coro.__name__ == "cant_stop_me":
            continue
        task.cancel()
    print("Cancelling outstanding tasks")
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()


async def shutdown_signal(signal, loop):
    print(f"Received exit signal {signal.name}...")
    await removetasks(loop)


if __name__ == '__main__':

    loop = asyncio.get_event_loop()
    signals = (signal.SIGHUP, signal.SIGTERM, signal.SIGINT)

    for s in signals:
        loop.add_signal_handler(
            s, lambda s=s: asyncio.create_task(shutdown_signal(s, loop)))
    try:
        controller = None
        while controller is None:
            controller = connect()

        tasks = [controller.read_gamepad_input(), read_gamepad_inputs(controller)
                 ]

        loop.run_until_complete(asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED))
        loop.run_until_complete(removetasks(loop))
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
        GPIO.cleanup()
        print("Exit")
        call("sudo shutdown -P now", shell=True)
