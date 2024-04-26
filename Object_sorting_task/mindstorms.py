import sim
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


class Direction(Enum):
    """
       Enum representing the direction of the motor rotation.

       Attributes:
           CLOCKWISE: Represents clockwise rotation.
           COUNTERCLOCKWISE: Represents counterclockwise rotation.
       """

    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


# ACTUATORS

class CoppeliaComponent:
    def __init__(self, clientID):
        self.clientID = clientID


class Motor(CoppeliaComponent):
    """
      Simplified version of the pyBricks motor class, especially adapted to Coppelia.

      Attributes:
          motor_port (int): The motor port number, either 1 or 2.
          direction (Direction): The direction of the motor rotation, either CLOCKWISE or COUNTERCLOCKWISE.
      """

    other_motor = None

    def __init__(self, motor_port, direction, clientID):
        """
            Initializes a Motor instance with the specified motor port and direction.

            Args:
            motor_port (int): The motor port number, either 1 or 2 (that's applicable only for the coppelia sim).
            direction (Direction): The direction of the motor rotation, either CLOCKWISE or COUNTERCLOCKWISE.
        """

        super().__init__(clientID)
        assert motor_port in ('A', 'B'), "Motor port must be either A or B in the coppelia simulation!"
        assert isinstance(direction, Direction), "Direction must be an instance of Direction enum"

        self.motor_port = motor_port
        self.direction = direction
        self.speed = 0

        if Motor.other_motor is None:
            Motor.other_motor = self
        else:
            self.other_motor = Motor.other_motor
            Motor.other_motor.other_motor = self

    def __set_speed(self, speed_l, speed_r):
        """
            Private method to set the speed of the left and right motors in the simulation.

            Args:
                speed_l (float): The desired speed for the left motor.
                speed_r (float): The desired speed for the right motor.
        """

        velocities = sim.simxPackFloats([speed_l, speed_r])
        sim.simxSetStringSignal(clientID=self.clientID, signalName="motors", signalValue=velocities,
                                operationMode=sim.simx_opmode_blocking)

    def run(self, speed):
        """
            Sets the speed of the motor based on the motor port and direction.

            Args:
            speed (float): The desired speed for the motor.
        """

        self.speed = speed
        speed_l = self.speed * self.direction.value
        speed_r = self.other_motor.speed * self.other_motor.direction.value

        if self.motor_port == 'A':
            self.__set_speed(speed_l, speed_r)
        else:
            self.__set_speed(speed_r, speed_l)


# SENSORS

class ColorSensor(CoppeliaComponent):
    """Color Sensor for the CoppeliaSim environment."""

    def __init__(self, clientID):
        """Initialize a ColorSensor instance."""
        super().__init__(clientID)

        self.image = self._get_image_sensor()

    def _get_image_sensor(self):
        return_code, return_value = sim.simxGetStringSignal(clientID=self.clientID, signalName="Sensors",
                                                            operationMode=sim.simx_opmode_blocking)
        if return_code == 0:
            image = sim.simxUnpackFloats(return_value)
            res = int(np.sqrt(len(image) / 3))
            return self.image_correction(image, res)
        else:
            return return_code

    def image_correction(self, image, res):
        """
        This function can be applied to images coming directly out of CoppeliaSim.
        It turns the 1-dimensional array into a more useful res*res*3 array, with the first
        two dimensions corresponding to the coordinates of a pixel and the third dimension to the
        RGB values. Aspect ratio of the image is assumed to be square (1x1).

        :param image: the image as a 1D array
        :param res: the resolution of the image, e.g. 64
        :return: an array of shape res*res*3
        """

        image = [int(x * 255) for x in image]
        image = np.array(image).reshape((res, res, 3))
        image = np.flip(m=image, axis=0)
        return image

    def color(self):
        pass  # TODO

    def ambient(self):
        """
              Calculate the ambient light intensity of the image.

              Returns:
                  intensity (float): The ambient light intensity, ranging from 0% (dark) to 100% (bright)
              """

        return np.mean(self.image) / 255 * 100

    def reflection(self):
        """
        Measures the reflection of a surface using a red light.

        Returns:
            Reflection, ranging from 0% (no reflection) to
            100% (high reflection).
        """
        return np.mean(self.image[:, :, 0] / 255 * 100)

    def rgb(self):
        """
        Measure the reflection of a surface using red, green, and blue channels of the image.

        Returns:
            Tuple of reflections for red, green, and blue light, each
            ranging from 0.0% (no reflection) to 100.0% (high reflection).
        """

        red = np.mean(self.image[:, :, 0]) / 255 * 100
        green = np.mean(self.image[:, :, 1]) / 255 * 100
        blue = np.mean(self.image[:, :, 2]) / 255 * 100

        return red, green, blue


# HELPER FUNCTIONS


def show_image(image):
    plt.imshow(image)
    plt.show()
