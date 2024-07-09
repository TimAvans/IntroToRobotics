from mindstorms import *
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

client = RemoteAPIClient()
sim = client.require("sim")

# HANDLES FOR ACTUATORS AND SENSORS
robot = Robot_OS(sim, DeviceNames.ROBOT_OS)

top_image_sensor = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
small_image_sensor = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)

# HELPER FUNCTION
def show_image(image):
    plt.imshow(image)
    plt.show()

def get_image_top_cam():
    # Image from the top camera
    return top_image_sensor.get_image()

def get_image_bottom_cam():
    # Image from the bottom camera
    return small_image_sensor.get_image()

def set_speed(speed_l, speed_r):
    left_motor.run(speed_l)
    right_motor.run(speed_r)

def perform_random_walk():
    if np.random.randint(1, 10) <= 3:
        if np.random.choice([True, True, False]):
            set_speed(0, 2)
        else:
            set_speed(2, 0)

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
                pixel_locations_x.append(x)
                pixel_locations_y.append(y) 
    return pixel_locations_x, pixel_locations_y

def avoid_wall(image, color_bottom, color_top):
    pixel_locations_x = []
    pixel_locations_y = []
    for y, row in enumerate(image):
        for x, pixel in enumerate(row):
            if  in_range(pixel, color_bottom, color_top):
                pixel_locations_x.append(x)
                pixel_locations_y.append(y)  
    if	len(pixel_locations_x) > 50 or len(pixel_locations_y) > 50:
        set_speed(10, 0)

def move_to_target(x, y):
    left_motor = 2
    right_motor = 2

    if  x > CONST_SCREEN_CENTER[0]:
        left_motor += 2
        print("Left")
    else:
        right_motor += 2
        print("Right")
    #TODO: Adjust speed to distance of object        
    set_speed(left_motor, right_motor)

CONST_NEAREST_DISTANCE = 0.22
CONST_YELLOW_TOP = [255, 255, 80]
CONST_YELLLOW_BOTTOM = [235, 235, 0]
CONST_BROWN_TOP = [255, 100, 70]
CONST_BROWN_BOTTOM = [20, 0, 0]
CONST_GREEN_TOP = [50, 255, 50]
CONST_GREEN_BOTTOM = [0, 150, 0]
CONST_RED_TOP = [255, 20, 20]
CONST_RED_BOTTOM = [100, 0, 0]
CONST_WALL_TOP = [140, 135, 160]
CONST_WALL_BOTTOM = [130, 125, 150]
CONST_SCREEN_CENTER = (32,32)
# MAIN CONTROL LOOP

# Starts coppeliasim simulation if not done already
sim.startSimulation()
found_cube = False
print('Connected')
while True:
        # your code goes here
        image = top_image_sensor.get_image()
        top_image_sensor._update_image()
        if found_cube:
            pixel_locations_x, pixel_locations_y = spot_cube(image, CONST_RED_BOTTOM, CONST_RED_TOP)
            if pixel_locations_x == [] and pixel_locations_y == []:
                 target_location =  0, 0
            else:
                 target_location = np.mean(pixel_locations_x), np.mean(pixel_locations_y)
            print(target_location)
            if  target_location == (0, 0):
                 perform_random_walk()
            else:  
                 move_to_target(target_location[0], target_location[1])
        image = small_image_sensor.get_image()
        small_image_sensor._update_image()
        #avoid_wall(image, CONST_WALL_BOTTOM, CONST_WALL_TOP)
        pixel_locations_x, pixel_locations_y = spot_cube(image, CONST_BROWN_BOTTOM, CONST_BROWN_TOP)
        # show_image(image)
        if len(pixel_locations_x) > 4000 or len(pixel_locations_y) > 4000:
            # print(len(pixel_locations_x))
            found_cube = True
            # print(found_cube)
            continue
        else:
            found_cube = False
        if found_cube is False:
            red_x, red_y = spot_cube(image, CONST_RED_BOTTOM, CONST_RED_TOP)
            if len(red_x) > 2000 or len(red_y) > 2000:
                set_speed(-15, -15)

        if pixel_locations_x == [] and pixel_locations_y == []:
            target_location =  0, 0
        else:
             target_location = np.mean(pixel_locations_x), np.mean(pixel_locations_y)
        print(target_location)
        if  target_location == (0, 0):
            perform_random_walk()
        else:  
            move_to_target(target_location[0], target_location[1])