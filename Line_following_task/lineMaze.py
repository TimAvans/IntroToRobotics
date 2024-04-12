import sim
import matplotlib.pyplot as plt
from mindstorms import Motor, Direction, ColorSensor
import numpy as np
import datetime
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
    old_reflection = 0
    # print(reflection)
    threshold = 40  # Midpoint between black and white
    left, right = pid_controller(reflection, old_reflection)
    print("LEFT: " + left.__str__() + " RIGHT: " + right.__str__())
    left_motor.run(speed=left)
    right_motor.run(speed=right)
    old_reflection = reflection

    # if reflection < threshold:
    #     left_motor.run(speed=-1)
    #     right_motor.run(speed=4)
    # else:
    #     left_motor.run(speed=4)
    #     right_motor.run(speed=-1)

CONSTANT_ERROR = 7
CONSTANT_D_ERROR = 0.0005
LEFT_THRESHOLD = 40
RIGHT_THRESHOLD = 60

def pid_controller(error, old_error):
    # Proportional
    left_motor_speed = CONSTANT_ERROR * ((LEFT_THRESHOLD - error) / 100) + 1
    right_motor_speed = CONSTANT_ERROR * ((error - RIGHT_THRESHOLD) / 100) + 1

    # Derivative
    derivative = error - old_error
    left_motor_speed -= derivative * CONSTANT_D_ERROR
    right_motor_speed += derivative * CONSTANT_D_ERROR

    # Turn off motor in case value is negative
    if left_motor_speed < 0:
        left_motor_speed = -1
    if right_motor_speed < 0:
        right_motor_speed = -1
    return (left_motor_speed, right_motor_speed)

# MAIN CONTROL LOOP
if clientID != -1:

    print('Connected')

    left_motor = Motor(motor_port='A', direction=Direction.CLOCKWISE, clientID=clientID)
    right_motor = Motor(motor_port='B', direction=Direction.CLOCKWISE, clientID=clientID)
    color_sensor = ColorSensor(clientID=clientID)

    while True:
        # End connection
        follow_line(color_sensor, left_motor, right_motor)

else:
    print('Failed connecting to remote API server')
print('Program ended')

# MAIN CONTROL LOOP
