from hub import light_matrix, port
import runloop
# import numpy as np

# import sim
# import matplotlib.pyplot as plt
# from mindstorms import Motor, Direction, ColorSensor
# import numpy as np
from collections import deque
# import threading

import motor
import color_sensor
import math
import motor_pair

velocity_left = 5
velocity_right = 5
last_10_errors = [30, 30, 30]

motor_pair.pair(motor_pair.PAIR_1, port.E, port.F)

# write your code here
def follow_line(integral):
    global last_10_errors
    reflection = color_sensor.reflection(port.D)
    # print(reflection)

    # last_10_errors.popleft()
    # last_10_errors.append(reflection)
    last_10_errors.pop(0)
    last_10_errors.append(reflection)
    # last_10_errors = last_10_errors + [reflection]


    global velocity_left
    global velocity_right

    average = sum(last_10_errors) / len(last_10_errors)

    left, right = pid_controller(reflection, average, last_10_errors)
    left = round(left)
    right = round(right)

    motor.run_for_degrees(port.E, 360, left)
    motor.run_for_degrees(port.F, 360, right)
    # left_thread = threading.Thread(target=motor.run(port.E, left))
    # right_thread = threading.Thread(target=motor.run(port.F, right))

    # left_thread.start()
    # right_thread.start()

    # left_thread.join()
    # right_thread.join()


# we can try modifying these and length of the last_10_list
# CONSTANT_ERROR = 0.5
# CONSTANT_D_ERROR = 0.02
# CONSTANT_B_ERROR = 3
# LEFT_THRESHOLD = 40
# RIGHT_THRESHOLD = 40

CONSTANT_ERROR = 0.5
CONSTANT_D_ERROR = 0.02
CONSTANT_B_ERROR = 3
LEFT_THRESHOLD = 40
RIGHT_THRESHOLD = 40



def created_new_function(x):
    y = -400 * 2.718*(-(x * 2) / 300) + 400 * 2.718*(-((x - 100) * 2) / 500)
    return y


def pid_controller(error, average, last_10_errors):
    mod_error = created_new_function(error)
    
    I_fix = sum(last_10_errors) * 0.000000001

    left_motor_speed = -(-mod_error + 100) + I_fix
    right_motor_speed = mod_error + 100 + I_fix

    selecting_boost_reflecting = 3/(0.1*2.5)1/(0.02) 2.718*(-((error - average) * 2)) * (1 / (1 + 2.718*(100 * (40 - error)))) * (1 / (1 + 2.718*(100 * (-60 + error))))
    
    boost = selecting_boost_reflecting 

    left_motor_speed -= boost
    right_motor_speed += boost

    return left_motor_speed, right_motor_speed

async def main():
    # last_10_errors = 30
    integral = 0
    while True:
        follow_line(integral)

runloop.run(main())
