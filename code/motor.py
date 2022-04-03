from signal import pause
from buildhat import Motor
import time

left_wheel = Motor('A')
right_wheel = Motor('B')
hook = Motor('C')

def open_hook():
    hook.run_to_position(35, speed=100, blocking=True)

def close_hook():
    hook.run_to_position(-35, speed=100, blocking=True)

def stop():
    left_wheel.stop()
    right_wheel.stop()

def forward(left_speed=-100, right_speed=100):
    left_wheel.start(left_speed)
    right_wheel.start(right_speed)


def backward():
    left_wheel.start(100)
    right_wheel.start(-100)


def right(turn_speed=100):
    left_wheel.start(-turn_speed)
    right_wheel.start(-turn_speed)


def left(turn_speed=100):
    left_wheel.start(turn_speed)
    right_wheel.start(turn_speed)

def left_seconds(turn_speed=100, seconds=1):
    left_wheel.run_for_seconds(seconds, turn_speed)
    time.sleep(seconds)
