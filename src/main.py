import time
import rclpy
from rclpy.node import Node 
import math
from sensor_msgs.msg import Image
from enum import Enum
import struct
import numpy as np
from utils import Position, Quadrant
from arena import Arena, Basket

# CHANGE DEPENDING ON ROBOT
from library.robot_small import Robot # Change to robot_big for the MIRTE Master

from library.detector import Detector
from library.communication import Communication
import library.utils as utils

# CHANGE DEPENDING ON ROBOT
PIONEER_ROBOT = True # True for pioneer robots, false for the MIRTE Gripper robot


###### Setup ######
rclpy.init()

team_id, robot_id = utils.get_team_robot_id()
password = utils.get_password()

robot = Robot()
communication = Communication("172.18.0.2:8000", team_id, robot_id, password)
detector = Detector(False)

stopped = False
robot_position = None
objectives = []

###### Callbacks #####
def on_receive_location(x, y, angle, visible, last_seen):
    # This is called whenever the server sends the location of the robot
    # print("location received!")
    robot_position = {"x": x, "y": y, "angle": angle, "visible": visible, "last_seen": last_seen}

def on_receive_start():
    # This is called when the competition is started (and every 10 seconds during the competition)
    # print("start received :)")
    stopped = False

def on_receive_stop():
    # This is called when the competition is paused (and every 10 seconds while it is paused). You MUST stop your robot when this is called!
    # print("stop received :(")
    stopped = True
    robot.drive(0, 0)

def on_receive_objective(from_team_id, from_robot_id, tag_id, x, y, angle, visible, last_seen):
    # This is called when another robot tells you they discovered an objective (id tag_id). It includes the position of the other robot
    print("new objective?")
    new_objective = {
        "tag_id": tag_id,
        "x": x,
        "y": y,
        "found_by_team": team_id,
        "found_by_robot": robot_id,
        "angle": angle,
        "visible": visible,
        "last_seen": last_seen
    }
    objectives.append(new_objective)

def on_receive_custom(from_team_id, from_robot_id, internal_type, bytes):
    # This is called whenever another robot sends a custom message
    print("custom received!", internal_type)
    if internal_type == 12:
        try:
            a, b = struct.unpack("<BB", bytes)
            print(a,b)
        except Exception as e:
            print("Could not parse message")

communication.register_callback_location(on_receive_location)
communication.register_callback_start(on_receive_start)
communication.register_callback_stop(on_receive_stop)
communication.register_callback_objective(on_receive_objective)
communication.register_callback_custom(on_receive_custom)


##### Main loop #####
# Send a custom message to itself (internal type 99, contents is two 8-bit integers (BB): 123 and 45)
# You should see the robot receive this message as well
communication.send_custom_msg(team_id, robot_id, 99, struct.pack("<BB", 123, 45))

# Drive forward for 2 seconds
driving_time = 30.0
speed = 100
t_end = time.time() + driving_time
while time.time() < t_end:
    # Drive by specifying the speed of the left and right motor
    robot.drive(speed, speed)
    # For the MIRTE Master: robot.drive(linear_x, linear_y, angular_z), all in m/s
    # Print the detected april tags
    tags = detector.detect_objective_tags()
    for tag in tags:
        print(f"Tag {tag.tag_id}, within distance: {utils.is_tag_within_distance(tag)}")






##############################
###### SHARED VARIABLES ######
##############################

starting_time = time.time()

current_quadrant = None # the quadrant the robot is supposed to be inside
inside_quadrant = False # 1/4th of the map
inside_quadrant_buffer = False # slightly larger than quadrant
quadrant_assigned_time = time.time() # The time when the current quadrant was assigned
quadrant_change_time = 240 # time in seconds before switching to a new quadrant

target_basket = None # robot's current target basket (pathfinding towards here)
target_position = None # robot's current target position (pathfinding towards here usually if no target basket)
target_assigned_time = time.time() # The time the current target was assigned (a robot can give up on a target if needed)

# The field has coordinates from (0, 0) to (6, 7.5).
starting_line_y = 1.0 # Needs to be measured precisely!
field_max_x = 6.0
field_max_y = 7.5
buffer_width = 0.5
# quadrants. The first one is the one closest to our zone, then they proceed clockwise
quadrant_0 = Quadrant(Position(field_max_x / 2, starting_line_y), Position(field_max_x, starting_line_y + (field_max_y - starting_line_y) / 2))
quadrant_1 = Quadrant(Position(field_max_x / 2, field_max_x, starting_line_y + (field_max_y - starting_line_y) / 2), Position(field_max_x, field_max_y))
quadrant_2 = Quadrant(Position(0.0, field_max_x, starting_line_y + (field_max_y - starting_line_y) / 2), Position(field_max_x / 2, field_max_y))
quadrant_3 = Quadrant(Position(0.0, starting_line_y), Position(field_max_x / 2, field_max_x, starting_line_y + (field_max_y - starting_line_y) / 2))

quadrants = [quadrant_0, quadrant_1, quadrant_2, quadrant_3]

# buffered quadrants (except where this would exceed field limits)
quadrant_0_buffered = Quadrant(Position(quadrant_0.lower.x - buffer_width, quadrant_0.lower.y - buffer_width), Position(quadrant_0.upper.x, quadrant_0.upper.y + buffer_width))
quadrant_1_buffered = Quadrant(Position(quadrant_1.lower.x - buffer_width, quadrant_1.lower.y - buffer_width), Position(quadrant_1.upper.x, quadrant_1.upper.y))
quadrant_2_buffered = Quadrant(Position(quadrant_2.lower.x, quadrant_2.lower.y - buffer_width), Position(quadrant_2.upper.x + buffer_width, quadrant_2.upper.y + buffer_width))
quadrant_3_buffered = Quadrant(Position(quadrant_3.lower.x, quadrant_3.lower.y - buffer_width), Position(quadrant_3.upper.x + buffer_width, quadrant_3.upper.y))

buffered_quadrants = [quadrant_0_buffered, quadrant_1_buffered, quadrant_2_buffered, quadrant_3_buffered]

#quadrants_test = [((3.0, 1.0), (6.0, 4.25)), ((3.0, 4.25), (6.0, 7.5)), ((0.0, 4.25), (3.0, 7.5)), ((0.0, 1.0), (3.0, 4.25))]

# Our starting zone is around (5.0 to 6.0, 0.0 to 1.0), but this needs to be checked
starting_zone_x = 5.0

# Stores all known baskets (scanned or not)
all_known_baskets = []



############################################################
#### PLACEHOLDER FUNCTIONS. IMPLEMENT HERE OR ELSEWHERE ####
############################################################

# This should add the basket to the known baskets if it doesn't yet exist
# Use proximity, team, and detection distance to determine if this is a new basket
# or if an existing basket should be updated. Baskets that have been scanned
# or have had items delivered should never be updated again.
def update_baskets_with_basket(basket: Basket) -> None:
    pass

# Selects the basket that is inside the given quadrant, has not yet been scanned, and
# maximises the objective function:
# f = visible + current_target + known_april_tag - euclidean_distance
# where:
# visible = 0.3 (visible in current camera frame)
# current_target = 0.5 (prioritises pathfinding to current target rather than a new basket)
# known_april_tag = 0.5 (april tag is already known, but hasn't been sent because not inside 50 cm range)
# euclidean_distance = distance in meters from target
def choose_highest_value_basket(quadrant: Quadrant) -> Basket:
    return all_known_baskets[0]


# Checks if there is at least one basket in the given quadrant that has not been scanned yet
def is_basket_available_in_quadrant(quadrant: Quadrant) -> bool:
    pass


# Picks the closest basket that satisfies the conditions:
# - belongs to our team (Basket team = Red)
# - item not delivered yet (Basket item_delivered = False)
def choose_closest_uncompleted_team_basket() -> Basket:
    return all_known_baskets[0]

# update everything about the robot's state:
# current_quadrant, inside_quadrant, inside_quadrant_buffer
# if quadrant changes, update target_position and target_basket to None
# update current_robot_position
def update_state():
    pass

# Pathfinding towards a basket
# This function should not block until reaching the target basket, but instead
# make a 'step' towards it, e.g., 100 ms of movement.
# The function should take care of obstacle avoidance, but remember to get close
# to the target basket (not 'avoid' it)
def pathfind_towards_basket(basket: Basket) -> None:
    pass


# Pathfinding towards a target
# Same as above, but the target is some position rather than a basket
# it shouldn't crash into an obstacle even if the position is inside one!
def pathfind_towards_target(target_position: Position) -> None:
    pass


# Select random position in quadrant that is atleast 1.0 meters away from the robot
def new_random_position(quadrant: Quadrant) -> Position:
    pass


# Forces the robot to make a 90 degree turn (to see new baskets with camera)
# (approximately is enough)
def turn_90_clockwise() -> None:
    pass


# Calculates the distance from the robot to the target
def distance_to_target(target_position: Position) -> float:
    pass

###############################
###### ACTUAL MAIN LOOPS ######
###############################

def pioneer_main_loop():

    update_state()

    turning_attempts = 0

    # MISSING TIMEOUTS (if targets are unreachable or too much time is wasted trying to get to one)
    # then the robot should give up and pick a new target
    if inside_quadrant:
        if is_basket_available_in_quadrant():
            target_basket = choose_highest_value_basket(current_quadrant)
            target_position = None
            pathfind_towards_basket(target_basket)
        else:
            if target_position == None:
                target_position = new_random_position()

            if distance_to_target(target_position) < 0.5:
                if turning_attempts < 3:
                    turn_90_clockwise()
                    time.sleep(0.2) # Sleep 200 ms for camera to stabilise
                    turning_attempts += 1
                else:
                    turning_attempts = 0
                    target_position = new_random_position()
                    pathfind_towards_target(target_position)
            else:
                pathfind_towards_target(target_position)

    elif inside_quadrant_buffer:
        if target_basket != None:
            pathfind_towards_basket(target_basket)
        else:
            target_position = new_random_position()
            pathfind_towards_target(target_position)
    else:
        if target_basket != None:
            pathfind_towards_basket(target_basket)
        else:
            target_position = new_random_position()
            pathfind_towards_target(target_position)



def gripper_main_loop():

    pass


if PIONEER_ROBOT:
    # Pick starting quadrant based on robot_id
    # I am assuming robot_id is either 1, 2, 3, but this can be adjusted later
    if robot_id == 1:
        current_quadrant = quadrant_1
    elif robot_id == 2:
        current_quadrant = quadrant_2
    elif robot_id == 3:
        current_quadrant = quadrant_3

    pioneer_main_loop()
else:
    current_quadrant = quadrant_0

    gripper_main_loop()



##### Shutdown nicely #####
robot.drive(0,0)
time.sleep(0.5)
rclpy.shutdown()
