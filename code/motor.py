from signal import pause
from buildhat import Motor
import time

right_wheel = Motor('A')
left_wheel = Motor('B')
hook = Motor('C')

def open_hook():
    hook.run_to_position(35, speed=100, blocking=True)

def close_hook():
    hook.run_to_position(-30, speed=100, blocking=True)

def stop():
    left_wheel.stop()
    right_wheel.stop()

def forward(left_speed=100, right_speed=-100):
    left_wheel.start(left_speed)
    right_wheel.start(right_speed)


def backward():
    left_wheel.start(-100)
    right_wheel.start(100)


def right():
    left_wheel.start(-100)
    right_wheel.start(-100)


def left():
    left_wheel.start(100)
    right_wheel.start(100)