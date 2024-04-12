import sim
import matplotlib.pyplot as plt
from mindstorms import Motor, Direction, ColorSensor

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
    print(reflection)
    threshold = 40  # Midpoint between black and white

    if reflection < threshold:
        left_motor.run(speed=-1)
        right_motor.run(speed=4)
    else:
        left_motor.run(speed=4)
        right_motor.run(speed=-1)


# MAIN CONTROL LOOP
if clientID != -1:

    print('Connected')

    left_motor = Motor(motor_port='A', direction=Direction.CLOCKWISE, clientID=clientID)
    right_motor = Motor(motor_port='B', direction=Direction.CLOCKWISE, clientID=clientID)
    color_sensor = ColorSensor(clientID=clientID)

    while True:
        # End connection

        left_motor.run(speed=5)
        right_motor.run(speed=5)

else:
    print('Failed connecting to remote API server')
print('Program ended')

# MAIN CONTROL LOOP
