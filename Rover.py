# import curses and GPIO
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

# set GPIO numbering mode and define output pins
GPIO.setmode(GPIO.BOARD)
DIR1 = 18
GPIO.setup(DIR1, GPIO.OUT)
DIR2 = 22
GPIO.setup(DIR2, GPIO.OUT)
PWM1_PIN = 32
GPIO.setup(PWM1_PIN, GPIO.OUT)
PWM1 = GPIO.PWM(PWM1_PIN, 20000)
PWM1.start(0)
PWM2_PIN = 33
GPIO.setup(PWM2_PIN, GPIO.OUT)
PWM2 = GPIO.PWM(PWM2_PIN, 20000)
PWM2.start(0)
NOT_FAULT_M1 = 36
NOT_FAULT_M2 = 37
GPIO.setup(NOT_FAULT_M1, GPIO.OUT)
GPIO.setup(NOT_FAULT_M2, GPIO.OUT)

# Enable motor
GPIO.output(NOT_FAULT_M1, True)
GPIO.output(NOT_FAULT_M2, True)


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


def is_connected():  # asyncronus read-out of events
    path = None
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        if str.lower(device.name) == 'xbox wireless controller':
            path = str(device.path)
    if (path == None):
        print('Xbox controller disconnected!!')
        return False
    return True


def has_duplicates_boolean(value, lst):
    # Initialize a variable to keep track of the XOR result
    xor_result = 0

    for bool_val in lst:
        # Convert boolean value to integer (True -> 1, False -> 0)
        int_val = int(bool_val)

        # Perform XOR operation with each integer value in the list
        xor_result ^= int_val

        # If the XOR result becomes the same as the integer representation of the value,
        # a duplicate is found
        if xor_result == int(value):
            return True

    # No duplicates found
    return False


async def read_gamepad_inputs(Controller):
    while is_connected():
        up = Controller.dpad_up
        down = Controller.dpad_down
        left = Controller.dpad_left
        right = Controller.dpad_right
        direction = [up, down, left, right]
        print(direction)
        if direction != [False, False, False, False]:
            if has_duplicates_boolean(True, direction):
                PWM1.ChangeDutyCycle(0)
                PWM2.ChangeDutyCycle(0)
            else:
                PWM1.ChangeDutyCycle(100)
                PWM2.ChangeDutyCycle(100)
                if direction[0] == True:
                    GPIO.output(DIR1, True)
                    GPIO.output(DIR2, True)
                elif direction[1] == True:
                    GPIO.output(DIR1, False)
                    GPIO.output(DIR2, False)
                elif direction[2] == True:
                    GPIO.output(DIR1, False)
                    GPIO.output(DIR2, True)
                elif direction[3] == True:
                    GPIO.output(DIR1, True)
                    GPIO.output(DIR2, False)
        else:
            PWM1.ChangeDutyCycle(0)
            PWM2.ChangeDutyCycle(0)

        await asyncio.sleep(0.01)


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
        controller = connect()
        if (controller == None):
            print('Please connect an Xbox controller then restart the program!')
            sys.exit()
        tasks = [loop.create_task(read_gamepad_inputs(controller)),
                 loop.create_task(controller.read_gamepad_input())]

        loop.run_until_complete(asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED))
        loop.run_until_complete(removetasks(loop))
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
        GPIO.cleanup()
        print("Exit")
