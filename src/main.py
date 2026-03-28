import time
import rclpy
from rclpy.node import Node 
import math
import random
from sensor_msgs.msg import Image
from enum import Enum
import struct
import numpy as np
from utils import Position, Quadrant
from arena import Arena, Basket
import math
from src.robot import Robot, Team

import sys
import os
from mirte_robot import robot

# Test the wheel moving
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pioneer-robot'))
import hardware as hd


# Main entry point to the program
# CHANGE DEPENDING ON ROBOT
from library.robot_small import Robot # Change to robot_big for the MIRTE Master

from library.detector import Detector
from library.communication import Communication, OUR_BASKET_MSG
import library.utils as utils
from arena import Basket, Position, Team

# CHANGE DEPENDING ON ROBOT
PIONEER_ROBOT = True # True for pioneer robots, false for the MIRTE Gripper robot

from rclpy.executors import MultiThreadedExecutor
from threading import Thread

###### Setup ######
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
    global robot_position
    # This is called whenever the server sends the location of the robot
    # print("location received!")
    robot_position = {"x": x, "y": y, "angle": angle, "visible": visible, "last_seen": last_seen}
    detector.robot_pose = [robot_position['x'],
                           robot_position['y'],
                           robot_position['angle']]

def on_receive_start():
    global stopped
    # This is called when the competition is started (and every 10 seconds during the competition)
    # print("start received :)")
    stopped = False

def on_receive_stop():
    global stopped
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

def on_receive_basket(from_team_id, from_robot_id, basket):
    print("Basket received", basket)

def on_receive_custom(from_team_id, from_robot_id, internal_type, contents):
    if internal_type == 99:
        try:
            a, b = struct.unpack("<BB", contents)
            print(a,b)
        except Exception as e:
            print("Could not parse message")

def main():
    rclpy.init()
    communication.register_callback_location(on_receive_location)
    communication.register_callback_start(on_receive_start)
    communication.register_callback_stop(on_receive_stop)
    communication.register_callback_objective(on_receive_objective)
    communication.register_callback_custom(on_receive_custom)
    communication.register_callback_our_basket()


    ##### Main loop #####
    # Send a custom message to itself (internal type 99, contents is two 8-bit integers (BB): 123 and 45)
    # You should see the robot receive this message as well
    communication.send_custom_msg(team_id, robot_id, 99, struct.pack("<BB", 123, 45))

# Test basket message
communication.send_basket(
    Basket(Position(1, 1), 5, 2.0)
)

# Drive forward for 2 seconds
#driving_time = 30.0
#speed = 100
#t_end = time.time() + driving_time
#while time.time() < t_end:
#    # Drive by specifying the speed of the left and right motor
#    robot.drive(speed, speed)
#    # For the MIRTE Master: robot.drive(linear_x, linear_y, angular_z), all in m/s
#    # Print the detected april tags
#    tags = detector.detect_objective_tags()
#    for tag in tags:
#        print(f"Tag {tag.tag_id}, within distance: {utils.is_tag_within_distance(tag)}")






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
quadrant_1 = Quadrant(Position(field_max_x / 2, starting_line_y + (field_max_y - starting_line_y) / 2), Position(field_max_x, field_max_y))
quadrant_2 = Quadrant(Position(0.0, starting_line_y + (field_max_y - starting_line_y) / 2), Position(field_max_x / 2, field_max_y))
quadrant_3 = Quadrant(Position(0.0, starting_line_y), Position(field_max_x / 2, starting_line_y + (field_max_y - starting_line_y) / 2))

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
# or have had items delivered should never be updated again. use baskets exposition if its smaller than const than its a more reliable reading then the ones with a larger expos
def update_baskets_with_basket(new_basket: Basket) -> None:
    global all_known_baskets
    
    MATCH_THRESHOLD = 0.30 
    
    for existing in all_known_baskets:
        dx = existing.pos.x - new_basket.pos.x
        dy = existing.pos.y - new_basket.pos.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < MATCH_THRESHOLD:

            if existing.team is not None and new_basket.team is not None:
                if existing.team != new_basket.team:
                    return
            
            # 1. LOCKOUT: Never update completed objectives
            if existing.scanned or existing.item_delivered:
                return
            
            # 2. RELIABILITY CHECK: Only process readings with good exposure
            if not new_basket.clipped_image:
                
                # 3. OVERWRITE CONDITIONS: Old reading was bad OR new reading is closer
                if existing.clipped_image or new_basket.measurement_distance < existing.measurement_distance:
                    existing.pos.x = new_basket.pos.x
                    existing.pos.y = new_basket.pos.y
                    existing.measurement_distance = new_basket.measurement_distance
                    existing.clipped_image = False
            
                if existing.team is None and new_basket.team is not None:
                    existing.team = new_basket.team
                    
            return 
            
    # 5. NEW DISCOVERY: No match found within threshold
    all_known_baskets.append(new_basket)

# Selects the basket that is inside the given quadrant, has not yet been scanned, and
# maximises the objective function:
# f = visible + current_target + known_april_tag - euclidean_distance
# where:
# visible = 0.3 (visible in current camera frame)
# current_target = 0.5 (prioritises pathfinding to current target rather than a new basket)
# known_april_tag = 0.5 (april tag is already known, but hasn't been sent because not inside 50 cm range)
# euclidean_distance = distance in meters from target
def choose_highest_value_basket(quadrant: Quadrant):
    global all_known_baskets, target_basket, robot_position
    
    # Failsafe: Cannot calculate distance without robot position
    if robot_position is None:
        return None

    best_basket = None
    # Initialize to negative infinity because the score will often be negative 
    # (e.g., distance is 3.0m, but bonuses only add up to 0.8)
    max_score = float('-inf')

    rx = robot_position["x"]
    ry = robot_position["y"]

    for basket in all_known_baskets:
        # 1. Filter: Ignore already scanned baskets
        if basket.scanned:
            continue

        # 2. Filter: Must be strictly inside the assigned quadrant
        in_x = quadrant.lower.x <= basket.pos.x <= quadrant.upper.x
        in_y = quadrant.lower.y <= basket.pos.y <= quadrant.upper.y
        if not (in_x and in_y):
            continue

        # 3. Calculate Objective Function Components
        score_visible = 0.3 if not basket.clipped_image else 0.0
        score_target = 0.5 if basket == target_basket else 0.0
        score_tag = 0.5 if basket.tag_id is not None else 0.0
        
        dist = math.sqrt((basket.pos.x - rx)**2 + (basket.pos.y - ry)**2)

        # 4. f = visible + current_target + known_april_tag - euclidean_distance
        current_score = score_visible + score_target + score_tag - dist

        # 5. Maximize f
        if current_score > max_score:
            max_score = current_score
            best_basket = basket

    return best_basket


# Checks if there is at least one basket in the given quadrant that has not been scanned yet
def is_basket_available_in_quadrant(quadrant: Quadrant) -> bool:
    global all_known_baskets
    
    for basket in all_known_baskets:
        # Check boundary conditions
        in_x = quadrant.lower.x <= basket.pos.x <= quadrant.upper.x
        in_y = quadrant.lower.y <= basket.pos.y <= quadrant.upper.y
        
        # Check if it is inside the quadrant AND has not been scanned yet
        if in_x and in_y and not basket.scanned:
            return True
            
    return False




def choose_closest_uncompleted_team_basket():
    global all_known_baskets, robot_position, team_id
    
    # Failsafe: Cannot calculate distance without robot position
    if robot_position is None:
        return None
        
    closest_basket = None
    min_dist = float('inf')
    
    rx = robot_position["x"]
    ry = robot_position["y"]
    
    for basket in all_known_baskets:
        # Filter: Must match our team AND not have an item delivered yet.
        if basket.team == Team.RED and not basket.item_delivered:
            
            dist = math.sqrt((basket.pos.x - rx)**2 + (basket.pos.y - ry)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_basket = basket
                
    return closest_basket

# update everything about the robot's state:
# current_quadrant, inside_quadrant, inside_quadrant_buffer
# if quadrant changes, update target_position and target_basket to None
# update current_robot_position
def update_state():
    global current_quadrant, inside_quadrant, inside_quadrant_buffer
    global target_position, target_basket
    global quadrant_assigned_time, quadrant_change_time
    global robot_position, quadrants, buffered_quadrants

    if current_quadrant is None:
        return

    # 1. Timer Check: Rotate quadrants every 240 seconds
    current_time = time.time()
    if current_time - quadrant_assigned_time > quadrant_change_time:
        
        # Find current index and move to the next one
        current_index = quadrants.index(current_quadrant)
        next_index = (current_index + 1) % len(quadrants)
        
        current_quadrant = quadrants[next_index]
        quadrant_assigned_time = current_time
        
        # Reset targets because the exploration zone just changed
        target_position = None
        target_basket = None
        print(f"Timer triggered. Rotating to Quadrant {next_index}")

    # 2. Location Check: Verify physical position against assigned quadrant
    if robot_position is not None:
        rx = robot_position["x"]
        ry = robot_position["y"]

        # Strict boundary check
        inside_quadrant = (current_quadrant.lower.x <= rx <= current_quadrant.upper.x) and \
                          (current_quadrant.lower.y <= ry <= current_quadrant.upper.y)

        # Buffered boundary check
        idx = quadrants.index(current_quadrant)
        buf_quad = buffered_quadrants[idx]
        
        inside_quadrant_buffer = (buf_quad.lower.x <= rx <= buf_quad.upper.x) and \
                                 (buf_quad.lower.y <= ry <= buf_quad.upper.y)
    else:
        # Failsafe if GPS is lost
        inside_quadrant = False
        inside_quadrant_buffer = False

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
    if robot_position is None:
        return None
    
    currentXPosition = robot_position["x"]
    currentYPosition = robot_position["y"]

    while True:
        rand_x = random.uniform(quadrant.lower.x, quadrant.upper.x)
        rand_y = random.uniform(quadrant.lower.y, quadrant.upper.y)

        # Compute distance
        distance = math.sqrt(
            (currentXPosition - rand_x) ** 2 +
            (currentYPosition - rand_y) ** 2
        )

        # Check constraint
        if distance >= 1.0:
            return Position(rand_x, rand_y)

    


# Forces the robot to make a 90 degree turn (to see new baskets with camera)
# (approximately is enough)
def turn_90_clockwise() -> None:
    pass


# Calculates the distance from the robot to the target
def distance_to_target(target_position: Position) -> float:
    if robot_position is None:
        return float(10)
    currentXPosition = robot_position["x"]
    currentYPosition = robot_position["y"]

    toGoXPosition = target_position.x
    toGoYPosition = target_position.y

    distance = math.sqrt(
        (currentXPosition - toGoXPosition) ** 2 +
        (currentYPosition - toGoYPosition) ** 2
    )
    return distance

###############################
###### ACTUAL MAIN LOOPS ######
###############################
turning_attempts = 0
def pioneer_main_loop():

    global target_basket
    global target_position
    global target_assigned_time
    global turning_attempts


    update_state()

    

    # MISSING TIMEOUTS (if targets are unreachable or too much time is wasted trying to get to one)
    # then the robot should give up and pick a new target
    if inside_quadrant:
        if is_basket_available_in_quadrant(current_quadrant):
            target_basket = choose_highest_value_basket(current_quadrant)
            target_position = None
            pathfind_towards_basket(target_basket)
        else:
            if target_position == None:
                target_position = new_random_position(current_quadrant)

            if distance_to_target(target_position) < 0.5:
                if turning_attempts < 3:
                    turn_90_clockwise()
                    time.sleep(0.2) # Sleep 200 ms for camera to stabilise
                    turning_attempts += 1
                else:
                    turning_attempts = 0
                    target_position = new_random_position(current_quadrant)
                    pathfind_towards_target(target_position)
            else:
                if target_assigned_time + 60 < time.time():
                    target_position = new_random_position(current_quadrant)
                    target_assigned_time = time.time()
                else:
                    pathfind_towards_target(target_position)

    elif inside_quadrant_buffer:
        if target_basket != None:
            pathfind_towards_basket(target_basket)
        else:
            target_position = new_random_position(current_quadrant)
            pathfind_towards_target(target_position)
    else:
        if target_basket != None:
            pathfind_towards_basket(target_basket)
        else:
            target_position = new_random_position(current_quadrant)
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

    # Initializes hardware n shit
    mirte = robot.createRobot()
    hd.set_motor_speed(mirte, 0, 0)
    
    while(1):
        print("Moving the motors")
        hd.set_motor_speed(mirte, 60, 60)
        sleep(1)
        print("Stopping the motors")
        hd.set_motor_speed(mirte, 0, 0)
        sleep(2)
        rclpy.spin_once(robot, timeout_sec=0.1) # Process messages
        pioneer_main_loop()
else:
    current_quadrant = quadrant_0

    while(1):
        gripper_main_loop()



##### Shutdown nicely #####
robot.drive(0,0)
time.sleep(0.5)
rclpy.shutdown()

    # Drive forward for 2 seconds
    # driving_time = 30.0
    # speed = 100
    # t_end = time.time() + driving_time
    # while time.time() < t_end:
    #     # Drive by specifying the speed of the left and right motor
    #     robot.drive(speed, speed)
    #     # For the MIRTE Master: robot.drive(linear_x, linear_y, angular_z), all in m/s
    #     # Print the detected april tags
    #     tags = detector.detect_objective_tags()
    #     for tag in tags:
    #         print(f"Tag {tag.tag_id}, within distance: {utils.is_tag_within_distance(tag)}")

if __name__ == "__main__":
    main()