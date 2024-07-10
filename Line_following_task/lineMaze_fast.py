import sim
import matplotlib.pyplot as plt
from mindstorms import Motor, Direction, ColorSensor
import numpy as np
from collections import deque
import threading
import datetime

CONSTANT_ERROR = 0.5
CONSTANT_D_ERROR = 0.02
CONSTANT_B_ERROR = 3
LEFT_THRESHOLD = 40
RIGHT_THRESHOLD = 40

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

def show_image(image):
    plt.imshow(image)
    plt.show()


def is_red_detected(color_sensor):
    """
    Calculates the relative intensity of the red channel compared to
    other channels
    """
    red_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    print(red, green, blue)
    red_intensity = red / (green + blue)

    return red_intensity > red_ratio_threshold


def is_blue_detected(color_sensor):
    """
       Calculates the relative intensity of the blue channel compared to
       other channels
       """
    blue_ratio_threshold = 1.5
    red, green, blue = color_sensor.rgb()
    blue_intensity = blue / (red + green)

    return blue_intensity > blue_ratio_threshold


def follow_line(color_sensor, left_motor, right_motor):
    """
    A very simple line follower that should be improved.
    """

    color_sensor.image = color_sensor._get_image_sensor()
    reflection = color_sensor.reflection()

    last_10_errors.popleft()
    last_10_errors.append(reflection)

    average = sum(last_10_errors) / len(last_10_errors)
    
    left, right = pid_controller(reflection, average)

    left_thread = threading.Thread(target=left_motor.run, args=(left,))
    right_thread = threading.Thread(target=right_motor.run, args=(right,))

    left_thread.start()
    right_thread.start()

    left_thread.join()
    right_thread.join()

def get_proportional_error(x):
    return -(1 / (0.5)) * np.exp(-(x ** 2) / 300) + (1 / (0.5)) * np.exp(-((x - 100) ** 2) / 500)

def pid_controller(error, average):
    mod_error = get_proportional_error(error)
    i_fix = sum(last_10_errors) * 0.000000001

    left_motor_speed = -mod_error + 1 + i_fix
    right_motor_speed = mod_error + 1 + i_fix
   
    selecting_boost_reflecting = (1 / (0.1 * 2.5)) * np.exp(-((error - average) ** 2) / (2 * 0.01)) * \
                                 (1 / (1 + np.exp(100 * (30 - error)))) * (1 / (1 + np.exp(100 * (-70 + error))))
    boost = selecting_boost_reflecting * CONSTANT_B_ERROR

    left_motor_speed += boost
    right_motor_speed += boost

    return left_motor_speed, right_motor_speed


# MAIN CONTROL LOOP
if clientID != -1:

    print('Connected')

    left_motor = Motor(motor_port='A', direction=Direction.CLOCKWISE, clientID=clientID)
    right_motor = Motor(motor_port='B', direction=Direction.CLOCKWISE, clientID=clientID)
    color_sensor = ColorSensor(clientID=clientID)

    last_10_errors = deque([30, 30])

    while True:
        follow_line(color_sensor, left_motor, right_motor)

else:
    print('Failed connecting to remote API server')
print('Program ended')

# MAIN CONTROL LOOP
