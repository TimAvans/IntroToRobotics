from mindstorms import *
from coppeliasim_zmqremoteapi_client import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

CONST_NEAREST_DISTANCE = 0.22
CONST_YELLOW_TOP = [255, 255, 80]
CONST_YELLLOW_BOTTOM = [235, 235, 0]
CONST_BROWN_TOP = [255, 100, 70]
CONST_BROWN_BOTTOM = [20, 20, 0]
CONST_GREEN_TOP = [75, 255, 50]
CONST_GREEN_BOTTOM = [0, 75, 0]
CONST_RED_TOP = [255, 20, 20]
CONST_RED_BOTTOM = [75, 0, 0]
CONST_WALL_TOP = [140, 135, 160]
CONST_WALL_BOTTOM = [130, 125, 150]
CONST_SCREEN_CENTER = (32,32)
CONST_BATTERY_BREAKPOINT = 20

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
    set_speed(0, 1)

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
    if x == [] and y == []:
        perform_random_walk()
        return
    
    left_motor = 4
    right_motor = 4

    if  x > CONST_SCREEN_CENTER[0]:
        left_motor += 2
        # print("Left")
    else:
        right_motor += 2
        # print("Right")
    set_speed(left_motor, right_motor)

# MAIN CONTROL LOOP
def handle_battery(image):
    if  robot.get_battery() <= CONST_BATTERY_BREAKPOINT:
        # if the battery is low enough we have to go search for charging platform (yellow field)
        pixel_locations_x, pixel_locations_y = spot_cube(image, CONST_YELLLOW_BOTTOM, CONST_YELLOW_TOP)
        if pixel_locations_x == [] and pixel_locations_y == []:         
            target_location =  0, 0
        else:
            target_location = np.mean(pixel_locations_x), np.mean(pixel_locations_y)

        if  target_location == (0,0):
            perform_random_walk()
        else:
            move_to_target(target_location[0], target_location[1])

def move_to_and_crush_brown_cube(image):
    pixel_locations_x, pixel_locations_y = spot_cube(image, CONST_BROWN_BOTTOM, CONST_BROWN_TOP)
    length_loc_x = len(pixel_locations_x)
    length_loc_y = len(pixel_locations_y)
    
    if  length_loc_x > 4000 or  length_loc_y > 4000:
        set_speed(15, 15)
        robot.compress()

    return pixel_locations_x, pixel_locations_y

def move_to_green_cube(image):
    # Searching for a green cube
    pixel_locations_x, pixel_locations_y = spot_cube(image, CONST_GREEN_BOTTOM, CONST_GREEN_TOP)
    return pixel_locations_x, pixel_locations_y


def find_burn_zone(top_image):
    pixel_locations_x, pixel_locations_y = spot_cube(top_image, CONST_RED_BOTTOM, CONST_RED_TOP)
    if pixel_locations_x == [] and pixel_locations_y == []:
        target_location =  0, 0
    else:
        target_location = np.mean(pixel_locations_x), np.mean(pixel_locations_y)
    return (target_location[0], target_location[1])

def make_action_decision(top_image, lower_image) -> str:
    # red_pixels_x, red_pixels_y = spot_cube(top_image, CONST_RED_BOTTOM, CONST_RED_TOP)
    br_pixels_x, br_pixels_y = spot_cube(lower_image, CONST_BROWN_BOTTOM, CONST_BROWN_TOP)
    gr_pixels_x, gr_pixels_y = spot_cube(lower_image, CONST_GREEN_BOTTOM, CONST_GREEN_TOP)
    bl_pixels_x, bl_pixels_y = spot_cube(lower_image, [0, 0, 0], [20,20,20])

    br_total = len(br_pixels_x) + len(br_pixels_y)
    gr_total = len(gr_pixels_x) + len(gr_pixels_y)
    bl_total = len(bl_pixels_x) + len(bl_pixels_y)
    
    if gr_total > 7500:
        return "holding_green"
    elif bl_total > 8000:
        return "holding_black"
    elif  gr_total > br_total:
        return "move_green"
    elif br_total > gr_total:
        return "move_brown"
    else:
        return "random_walk"

def move_away_from_burnzone(lower_image):
    red_pixels_x, red_pixels_y = spot_cube(lower_image, CONST_RED_BOTTOM, CONST_RED_TOP)
    red_total = len(red_pixels_x) + len(red_pixels_y)

    if red_total > 500:
        for i in range(5):
            set_speed(-15) 

# Starts coppeliasim simulation if not done already
sim.startSimulation()
found_cube = False
holding_b_box = False

print('Connected')
while True:
        top_image = top_image_sensor.get_image()
        top_image_sensor._update_image()
        lower_image = small_image_sensor.get_image()
        small_image_sensor._update_image()

        # Find the decision we have to make
        choice = make_action_decision(top_image, lower_image)
        print(choice)
        # Priority 1: functionality (Load when needed)
        # handle_battery() <-- function doesn't work batter returns string?

        # Priority 1.5: When holding a green cube find and go to burn zone and burn it
        if choice == "holding_green":
            target_location_x, target_location_y = find_burn_zone(top_image)
            move_to_target(np.mean(target_location_x), np.mean(target_location_y))
            move_away_from_burnzone(lower_image)

        # Priority 1.7: When holding a black box find and go to burn zone and burn it
        elif choice == "holding_black":
            target_location_x, target_location_y = find_burn_zone(top_image)
            move_to_target(np.mean(target_location_x), np.mean(target_location_y))
            move_away_from_burnzone(lower_image)

        # Priority 2: Find and crush brown cube
        elif  choice == "move_brown":
            target_location_x, target_location_y = move_to_and_crush_brown_cube(lower_image)
            move_to_target(np.mean(target_location_x), np.mean(target_location_y))

        # Priority 3: Find and burn green cube
        elif choice == "move_green":
            target_location_x, target_location_y = move_to_green_cube(lower_image)
            move_to_target(np.mean(target_location_x), np.mean(target_location_y))

        # base function: (random walk)
        else:
            perform_random_walk()