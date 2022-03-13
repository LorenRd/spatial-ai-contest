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