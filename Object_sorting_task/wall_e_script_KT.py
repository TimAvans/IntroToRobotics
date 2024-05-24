import colorsys
import sim
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)


# FUNCTIONS TO INTERFACE WITH THE ROBOT
def compress():
    sim.simxSetIntegerSignal(clientID=clientID, signalName="compress", signalValue=1,
                             operationMode=sim.simx_opmode_blocking)


def set_speed(speed_l, speed_r):
    sim.simxSetStringSignal(clientID=clientID, signalName="motors", signalValue=sim.simxPackFloats([speed_l, speed_r]),
                            operationMode=sim.simx_opmode_blocking)


def get_battery():
    return sim.simxGetStringSignal(clientID=clientID, signalName="battery",
                                   operationMode=sim.simx_opmode_blocking)


def get_bumper_sensor():
    # Bumper reading as 3-dimensional force vector
    bumper_force_vector = [0, 0, 0]
    return_code, bumper_force_vector_packed = sim.simxGetStringSignal(clientID=clientID, signalName="bumper_sensor",
                                                                      operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        bumper_force_vector = sim.simxUnpackFloats(bumper_force_vector_packed)
    return bumper_force_vector


def get_sonar_sensor():
    # Sonar reading as distance to closest object detected by it, -1 if no data
    sonar_dist = -1
    return_code, sonar_dist_packed = sim.simxGetStringSignal(clientID=clientID, signalName="sonar_sensor",
                                                             operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        sonar_dist = sim.simxUnpackFloats(sonar_dist_packed)
    return sonar_dist


def get_image_small_cam():
    # Image from the small camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="small_cam_image",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


def get_image_top_cam():
    # Image from the top camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="top_cam_image",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


# HELPER FUNCTIONS
def image_correction(image, res):
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


def show_image(image):
    plt.imshow(image)
    plt.show()

# END OF FUNCTIONS

def perform_random_walk():
    left_motor = np.random.randint(1, 15)
    right_motor = np.random.randint(1,15)
    set_speed(left_motor, right_motor)

def in_range(pixel, color_bottom, color_top):
    if  pixel[0] < color_bottom[0] or pixel[0] > color_top[0]:
        return False
    if  pixel[1] < color_bottom[1] or pixel[1] > color_top[1]:
        return False
    if  pixel[2] < color_bottom[2] or pixel[2] > color_top[2]:
        return False
    return True

def spot_cube(image, color_bottom, color_top):
    pixel_locations_x = []
    pixel_locations_y = []
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if  in_range(pixel, color_bottom, color_top):
                print("found")
                pixel_locations_x.append(x)
                pixel_locations_y.append(y) 
    # show_image(image)
    if  pixel_locations_x == [] and pixel_locations_y == []:
        return 0, 0
    return np.mean(pixel_locations_x), np.mean(pixel_locations_y)

def move_to_target(x, y):
    left_motor = 2
    right_motor = 2

    if  x > CONST_SCREEN_CENTER[0]:
        left_motor += 15
        print("Left")
    else:
        right_motor += 15
        print("Right")
    #TODO: Adjust speed to distance of object        
    set_speed(left_motor, right_motor)

    
CONST_NEAREST_DISTANCE = 0.22
CONST_YELLOW_TOP = [255, 255, 80]
CONST_YELLLOW_BOTTOM = [235, 235, 0]
CONST_BROWN_TOP = [150, 100, 50]
CONST_BROWN_BOTTOM = [100, 50, 0]
CONST_GREEN_TOP = [50, 255, 50]
CONST_GREEN_BOTTOM = [0, 150, 0]
CONST_SCREEN_CENTER = (32,32)
# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    while True:
        # your code goes here
        target_location = spot_cube(get_image_top_cam(), CONST_BROWN_BOTTOM, CONST_BROWN_TOP)
        print(target_location)
        if  target_location == (0, 0):
            perform_random_walk()
        else:  
            move_to_target(target_location[0], target_location[1])
    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
