import sys
import os
import math
import time
import threading
import queue
import rclpy
import random
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Range

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library import utils
from library.robot_big import Robot
from library.communication import Communication
from library.detector import Detector

# ==========================================
# 0. TARGET LIST CONFIGURATION
# ==========================================
# Add as many points as you want here! 
POTENTIAL_TARGETS = [
    {"x": 1.0, "y": 1.5, "angle": -1.55, "tag_id": 90},
    {"x": 2.0, "y": 2.0, "angle": -1.55, "tag_id": 91},
    {"x": 3.0, "y": 3.0, "angle": -1.55,  "tag_id": 92},
    {"x": 3.5, "y": 3.5, "angle": -1.55, "tag_id": 93},
    {"x": 5.0, "y": 3.0, "angle": 3.14, "tag_id": 94},
    {"x": 5.0, "y": 6.0, "angle": 1.55, "tag_id": 94},
    {"x": 4.0, "y": 7.0, "angle": 3.14, "tag_id": 94},
    {"x": 3.0, "y": 5.5, "angle": 3.14, "tag_id": 94},
    {"x": 0.5, "y": 1.0, "angle": 3.14, "tag_id": 94},
    {"x": 5.0, "y": 5.0, "angle": 3.14, "tag_id": 94},
    {"x": 2.0, "y": 3.0, "angle": 3.14, "tag_id": 94},
    {"x": 1.5, "y": 3.0, "angle": 3.14, "tag_id": 94},
    {"x": 2.3, "y": 3.0, "angle": -1.55, "tag_id": 94},
    {"x": 4.3, "y": 3.0, "angle": 3.14, "tag_id": 94},
    {"x": 4.0, "y": 7.0, "angle": 3.14, "tag_id": 94},
    {"x": 3.0, "y": 5.5, "angle": 3.14, "tag_id": 94},
    {"x": 0.5, "y": 1.4, "angle": 3.14, "tag_id": 94},
    {"x": 1.2, "y": 6.0, "angle": -1.55, "tag_id": 94},
    {"x": 3.4, "y": 7.0, "angle": 3.14, "tag_id": 94},
    {"x": 5.2, "y": 6.2, "angle": -1.55, "tag_id": 94},
    {"x": 1.6, "y": 6.5, "angle": 3.14, "tag_id": 94},
    {"x": 2.8, "y": 6.4, "angle": 3.14, "tag_id": 94},
    {"x": 4.9, "y": 5.8, "angle": -1.55, "tag_id": 94},
    {"x": 3.4, "y": 4.2, "angle": 3.14, "tag_id": 94},
    {"x": 0.6, "y": 3.2, "angle": 3.14, "tag_id": 94},
    {"x": 5.0, "y": 6.1, "angle": -1.55, "tag_id": 94},
    {"x": 4.7, "y": 7.0, "angle": 3.14, "tag_id": 94},
    {"x": 5.0, "y": 2.4, "angle": 3.14, "tag_id": 94},
    {"x": 5.5, "y": 3.4, "angle": -1.55, "tag_id": 94},
    {"x": 5.0, "y": 6.1, "angle": -1.55, "tag_id": 94},
    {"x": 4.6, "y": 7.0, "angle": 3.14, "tag_id": 94},
    {"x": 5.6, "y": 3.4, "angle": 3.14, "tag_id": 94},
    {"x": 5.6, "y": 4.4, "angle": -1.55, "tag_id": 94},
    {"x": 3.0, "y": 4.1, "angle": -1.55, "tag_id": 94},
    {"x": 3.6, "y": 3.0, "angle": 3.14, "tag_id": 94},
    {"x": 3.6, "y": 3.4, "angle": 3.14, "tag_id": 94},
    {"x": 4.6, "y": 4.4, "angle": -1.55, "tag_id": 94},
    {"x": 5.6, "y": 3.7, "angle": 3.14, "tag_id": 94}
]


# ==========================================
# 0. TEST MODE CONFIG
# ==========================================
TEST_MODE_NO_CAMERA = False  

current_x, current_y, current_angle = 0.0, 0.0, 0.0
is_moving_allowed = False 
is_visible = False 

# ==========================================
# 1. INITIALIZATION & SETUP
# ==========================================
TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "uncertain-decoy-dandruff"

rclpy.init()

mirte = Robot()
HOST = "172.18.0.2:8000"  
comm = None

if TEST_MODE_NO_CAMERA:
    try:
        print(f"Attempting to connect to {HOST} (Test Mode)...")
        comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)
        print("Server connected.")
    except Exception as e:
        print(f"Test Mode: Server not found ({e}). Continuing with local simulation.")

else:
    print(f"Connecting to Competition Server at {HOST}...")

    comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)
    print("Competition Server Connected.")

objective_queue = queue.Queue()

# ==========================================
# 2. STATE VARIABLES & APF TUNING
# ==========================================

target_x, target_y, target_angle = None, None, None

navigation_state = "IDLE" 

# --- STARTING SNAPSHOT ---
initial_x, initial_y, initial_angle = 0.0, 0.0, 0.0
is_start_captured = False # We'll use this to wait for the first camera frame


# --- LIDAR DATA ---
# APF Tuning Parameters
K_ATT = 1.0       
K_REP = 0.30     
MAX_TOTAL_REP = 3.0  
D_OBSTACLE = 0.4
MAX_SPEED = 0.4   
ROBOT_RADIUS = 0.20
CHASSIS_IGNORE_ZONE = 0.25     

global_lidar_data = []
LIDAR_OFFSET_X = 0.10   
LIDAR_MOUNT_ANGLE = math.pi / 2  # 90 degrees

last_lidar_print = 0.0
last_debug_time = 0.0

# --- IR Sensor Data ---
ir_left_dist = 1.0  # hardcoded 1m distance
ir_right_dist = 1.0
IR_THRESHOLD = 0.12 # The zone where IR takes over 
K_IR_REP = 2.5 # High gain to ensure this overrides the attractive force


# --- PICKUP COORDINATES ---
PICKUP_X = 5.25
PICKUP_Y = 1.05
PICKUP_ANGLE = -1.55

# --- MISSION COORDINATES ---
TARGET_1_X = 2.0
TARGET_1_Y = 2.0
TARGET_1_ANGLE = 1.55

handled_baskets = set()
detector = Detector(True)
reported_tags = set()

# ==========================================
# 3. CALLBACK FUNCTIONS
# ==========================================
def on_receive_location(x, y, angle, visible, last_seen):
    global current_x, current_y, current_angle, is_visible
    global initial_x, initial_y, initial_angle, is_start_captured
    
    if visible:
        if not is_start_captured:
            initial_x, initial_y, initial_angle = x, y, angle
            is_start_captured = True
            print(f"Initial Global POS Captured: X:{x:.2f}, Y:{y:.2f}")

        # RAW GLOBAL values for navigation
        current_x, current_y, current_angle = x, y, angle
        
        is_visible = True
    else:
        if not is_start_captured:
            # This tells you the server is working, but the tag is hidden!
            print("Server connected, but AprilTag is hidden/not detected...", end="\r")
        is_visible = False

def on_receive_start():
    global is_moving_allowed
    if not is_moving_allowed:
        is_moving_allowed = True
        print("Competition Started! Moving allowed.")

def on_receive_stop():
    global is_moving_allowed
    is_moving_allowed = False
    stop_motors()
    print("STOP MESSAGE RECEIVED. Stop immediately.")

def on_receive_objective(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
    global handled_baskets
    # We only care about baskets for our team (Team 5)
    if team_id == TEAM_ID and tag_id not in handled_baskets:
        # We check if this is a new basket we haven't handled yet
        # (You could add a 'handled_tags' set here to avoid duplicates)
        objective_queue.put({
            "x": x, 
            "y": y, 
            "angle": angle, 
            "tag_id": tag_id
        })
        print(f"📥 Pioneer Found Basket {tag_id} at X:{x:.2f}, Y:{y:.2f}. Queue Size: {objective_queue.qsize()}")

def on_receive_lidar(msg):
    global global_lidar_data, last_lidar_print
    global_lidar_data = list(msg.ranges)

def on_receive_ir_left(msg):
    global ir_left_dist
    ir_left_dist = msg.range

def on_receive_ir_right(msg):
    global ir_right_dist
    ir_right_dist = msg.range

if comm:
    comm.register_callback_location(on_receive_location)
    comm.register_callback_start(on_receive_start)
    comm.register_callback_stop(on_receive_stop)
    comm.register_callback_objective(on_receive_objective)
mirte.node.create_subscription(LaserScan, "/scan", on_receive_lidar, 10, callback_group=MutuallyExclusiveCallbackGroup())

# ==========================================
# 4. APF NAVIGATION LOGIC
# ==========================================
def get_lidar_repulsive_forces():
    global global_lidar_data, MAX_TOTAL_REP, CHASSIS_IGNORE_ZONE
    rep_force_local_x, rep_force_local_y = 0.0, 0.0
    
    if not global_lidar_data:
        return 0.0, 0.0 

    num_points = len(global_lidar_data)
    angle_increment = (2 * math.pi) / num_points 

    for i, distance in enumerate(global_lidar_data):
        # Apply the hardware shift AND the mount rotation
        beam_angle = -math.pi + (i * angle_increment) + LIDAR_MOUNT_ANGLE
        
        norm_angle = beam_angle
        while norm_angle > math.pi: norm_angle -= 2 * math.pi
        while norm_angle < -math.pi: norm_angle += 2 * math.pi

        # BLINDFOLD: Now correctly ignores the back 180 relative to the FRONT
        if abs(norm_angle) > (math.pi / 2):
            continue

        
        if math.isinf(distance) or math.isnan(distance) or distance <= CHASSIS_IGNORE_ZONE: 
            continue
            
        obs_lidar_x = distance * math.cos(norm_angle)
        obs_lidar_y = distance * math.sin(norm_angle)
        
        obs_center_x = obs_lidar_x + LIDAR_OFFSET_X
        obs_center_y = obs_lidar_y 
        
        true_dist_to_center = math.sqrt(obs_center_x**2 + obs_center_y**2)
        true_angle = math.atan2(obs_center_y, obs_center_x)

        if CHASSIS_IGNORE_ZONE < true_dist_to_center < D_OBSTACLE: 
            force_mag = K_REP * ((1.0 / true_dist_to_center) - (1.0 / D_OBSTACLE)) * (1.0 / (true_dist_to_center**2))
            
            rep_force_local_x -= force_mag * math.cos(true_angle)
            rep_force_local_y -= force_mag * math.sin(true_angle)
    
    total_rep_mag = math.sqrt(rep_force_local_x**2 + rep_force_local_y**2)

    if total_rep_mag > MAX_TOTAL_REP:
        scaling_factor = MAX_TOTAL_REP / total_rep_mag
        rep_force_local_x *= scaling_factor
        rep_force_local_y *= scaling_factor
    
    return rep_force_local_x, rep_force_local_y

def calculate_local_apf_vector():
    global last_debug_time

    if target_x is None or target_y is None:
        return 0.0, 0.0 
        
    global_dx = target_x - current_x
    global_dy = target_y - current_y
    dist_to_goal = math.sqrt(global_dx**2 + global_dy**2)
    
    if dist_to_goal < 0.15: 
        return 0.0, 0.0
        
    global_angle_to_goal = math.atan2(global_dy, global_dx)
    local_angle_to_goal = utils.angle_difference(global_angle_to_goal, current_angle) 
    
    effective_dist = min(dist_to_goal, 1.5)
    att_force_local_x = K_ATT * effective_dist * math.cos(local_angle_to_goal)
    att_force_local_y = K_ATT * effective_dist * math.sin(local_angle_to_goal)
    
    # LiDAR repulsion (Long-range)
    rep_lidar_x, rep_lidar_y = get_lidar_repulsive_forces()
    
    # IR repulsion (Short-range/Safety)
    rep_ir_x, rep_ir_y = get_ir_repulsive_vector()

    # Calculating total force
    total_x = att_force_local_x + rep_lidar_x + rep_ir_x 
    total_y = att_force_local_y + rep_lidar_y + rep_ir_y 

    # --- DEBUGGING DASHBOARD
    current_time = time.time()
    if current_time - last_debug_time > 0.5:
        closest_obs_dist = 999.0
        if global_lidar_data:
            angle_inc = (2 * math.pi) / len(global_lidar_data)
            for i, dist in enumerate(global_lidar_data):
                # Match the hardware shift and rotation
                ang = -math.pi + (i * angle_inc) + LIDAR_MOUNT_ANGLE
                
                while ang > math.pi: ang -= 2 * math.pi
                while ang < -math.pi: ang += 2 * math.pi
                
                if abs(ang) <= (math.pi / 2) and not math.isinf(dist) and not math.isnan(dist) and dist > 0.25:
                    if dist < closest_obs_dist:
                        closest_obs_dist = dist
        
        obs_string = f"{closest_obs_dist:.2f}m" if closest_obs_dist != 999.0 else "CLEAR"
        
        print(f"\n--- [{navigation_state}] ---")
        print(f"POS:  X:{current_x:.2f}  Y:{current_y:.2f}  |  TGT: X:{target_x:.2f}  Y:{target_y:.2f}  |  DIST: {dist_to_goal:.2f}m")
        print(f"ATT:  x:{att_force_local_x:>5.2f}  y:{att_force_local_y:>5.2f}")
        print(f"REP LIDAR:  x:{rep_lidar_x:>5.2f}  y:{rep_lidar_y:>5.2f}")
        print(f"REP IR:  x:{rep_ir_x:>5.2f}  y:{rep_ir_y:>5.2f}")
        print(f"OUT:  x:{total_x:>5.2f}  y:{total_y:>5.2f}")
        print(f"CLOSEST VALID OBSTACLE: {obs_string}")
        last_debug_time = current_time
    
    return total_x, total_y

def out_of_local_minima():
    """
    Reliable 3-stage escape: 
    1. Back up 10cm 
    2. Rotate ~45 degrees Right 
    3. Move forward 10cm
    """
    print("\nESCAPE SEQUENCE: Back-Turn-Forward Maneuver...")

    # Stage 1: Move Backwards 10cm
    for _ in range(20):
        mirte.drive(-0.3, 0.0, 0.0) # Faster reverse
        time.sleep(0.05)

    # Stage 2: Hard Right Turn (25 cycles)
    print("Stage 2: Hard Right...")
    for _ in range(25):
        mirte.drive(0.0, 0.0, -1.2) # Sharper turn
        time.sleep(0.05)

    # Stage 3: Forward Leap (15 cycles)
    print("Stage 3: Forward Leap...")
    for _ in range(30):
        mirte.drive(0.3, 0.0, 0.0)
        time.sleep(0.05)

    stop_motors()
    print("Escape complete. Resuming navigation.")


# ==========================================
# FINAL APPROACH
# ==========================================

def execute_final_approach():
    """
    Bypasses the APF and uses raw LiDAR to center the basket and dock smoothly.
    Returns True when the robot is 22cm away from the basket.
    """
    global global_lidar_data, current_x, current_y, current_angle
    
    if not global_lidar_data:
        stop_motors()
        return False

    num_points = len(global_lidar_data)
    angle_increment = (2 * math.pi) / num_points 
    
    min_dist = 999.0
    basket_angle = 0.0
    
    # We only care about objects directly in front of us (30 degrees left/right)
    FRONT_CONE = math.pi / 6 
    
    for i, distance in enumerate(global_lidar_data):
        beam_angle = -math.pi + (i * angle_increment) + LIDAR_MOUNT_ANGLE
        norm_angle = beam_angle
        while norm_angle > math.pi: norm_angle -= 2 * math.pi
        while norm_angle < -math.pi: norm_angle += 2 * math.pi
        
        # Ignore our own chassis
        if distance <= CHASSIS_IGNORE_ZONE:
            continue
            
        # If the object is in our front cone, track the closest point
        if abs(norm_angle) < FRONT_CONE:
            if distance < min_dist:
                min_dist = distance
                basket_angle = norm_angle

    if min_dist != 999.0:
        # We see the basket!
        print(f"FINAL APPROACH: Basket seen at {min_dist:.2f}m, Aligning...", end="\r")
        
        if min_dist <= 0.28: # 28cm stopping distance!
            stop_motors()
            print("\n Basket reached and aligned! Stopping.")
            return True
        else:
            # Drive forward slowly at 0.15 m/s
            forward_speed = 0.15 
            
            # Steer to keep the basket perfectly centered
            # (Using the -1.5 inverted steering fix you already applied!)
            angular_z = utils.clamp(-1.5 * basket_angle, -0.5, 0.5)
            
            mirte.drive(forward_speed, 0.0, angular_z)
            if TEST_MODE_NO_CAMERA:
                simulate_camera_feedback(forward_speed, 0.0, angular_z)
                
            return False
            
    else:
        # Fallback: If we don't see the basket, just crawl to the global coordinates
        print("FINAL APPROACH: Searching for basket...", end="\r")
        
        global_dx = target_x - current_x
        global_dy = target_y - current_y
        dist_to_global = math.sqrt(global_dx**2 + global_dy**2)
        
        if dist_to_global < 0.15:
            stop_motors()
            print("\nReached global coordinates blindly.")
            return True
            
        global_heading = math.atan2(global_dy, global_dx)
        angle_error = utils.angle_difference(global_heading, current_angle)
        
        angular_z = utils.clamp(-1.5 * angle_error, -0.5, 0.5)
        mirte.drive(0.15, 0.0, angular_z)
        
        if TEST_MODE_NO_CAMERA:
            simulate_camera_feedback(0.15, 0.0, angular_z)
            
        return False

# ==========================================
# IR SENSORS
# ==========================================

def is_obstacle_too_close(threshold):
    """
    Returns True if an object is detected within 'threshold' meters.
    Default threshold is 10cm.
    """
    global ir_left_dist, ir_right_dist
    
    # Check if either sensor is below the safety limit
    if ir_left_dist < threshold or ir_right_dist < threshold:
        # Optional: Print which side triggered it for debugging
        side = "LEFT" if ir_left_dist < threshold else "RIGHT"
        # print(f"IR WARNING: Object detected on {side} side!")
        return True
        
    return False

def get_ir_repulsive_vector():
    """Calculates a high-gain repulsive vector for the 0-15cm range."""
    global ir_left_dist, ir_right_dist, IR_THRESHOLD, K_IR_REP
    push_x, push_y = 0.0, 0.0 

    # Left Sensor (pointing roughly 45 degrees left)
    if ir_left_dist < IR_THRESHOLD:
        # Calculate how 'deep' into the danger zone we are
        force_mag = K_IR_REP * (IR_THRESHOLD - ir_left_dist)
        push_x -= force_mag * 0.707  # Push Back
        push_y -= force_mag * 0.707  # Push Right

    # Right Sensor (pointing roughly 45 degrees right)
    if ir_right_dist < IR_THRESHOLD:
        force_mag = K_IR_REP * (IR_THRESHOLD - ir_right_dist)
        push_x -= force_mag * 0.707  # Push Back
        push_y += force_mag * 0.707  # Push Left

    return push_x, push_y

# ==========================================
# OTHERS
# ==========================================

def get_clearer_side_direction():
    """Returns 1.0 for Left, -1.0 for Right based on LiDAR averages."""
    global global_lidar_data
    if not global_lidar_data:
        return 1.0 # Default to left
    
    num_points = len(global_lidar_data)
    angle_inc = (2 * math.pi) / num_points
    
    left_sum, right_sum = 0, 0
    left_count, right_count = 0, 0

    for i, dist in enumerate(global_lidar_data):
        if math.isinf(dist) or math.isnan(dist): dist = 2.0 # Treat 'inf' as clear path
        
        # Calculate angle relative to front
        ang = -math.pi + (i * angle_inc) + LIDAR_MOUNT_ANGLE
        while ang > math.pi: ang -= 2 * math.pi
        while ang < -math.pi: ang += 2 * math.pi
        
        # Left Sector (0 to 90 degrees)
        if 0 < ang < (math.pi / 2):
            left_sum += dist
            left_count += 1
        # Right Sector (-90 to 0 degrees)
        elif -(math.pi / 2) < ang < 0:
            right_sum += dist
            right_count += 1
            
    avg_left = left_sum / left_count if left_count > 0 else 0
    avg_right = right_sum / right_count if right_count > 0 else 0
    
    return 1.0 if avg_left >= avg_right else -1.0



def calculate_angular_velocity():
    if target_x is None or target_y is None:
        return 0.0
    
    # Calculate angle to the actual target
    global_dx = target_x - current_x
    global_dy = target_y - current_y
    target_heading = math.atan2(global_dy, global_dx)

    angle_error = utils.angle_difference(target_heading, current_angle)

    return utils.clamp(-1.5 * angle_error, -0.8, 0.8)

def apply_motor_commands(local_x, local_y):
    angular_z = calculate_angular_velocity() 

    if local_x == 0 and local_y == 0 and angular_z == 0:
        stop_motors()
        return

    magnitude = math.sqrt(local_x**2 + local_y**2)
    
    # Cap the maximum speed, but allow it to crawl as slow as it wants
    if magnitude > MAX_SPEED:
        scaling_factor = MAX_SPEED / magnitude
        local_x *= scaling_factor
        local_y *= scaling_factor

    mirte.drive(local_x, local_y, angular_z) 

    if TEST_MODE_NO_CAMERA:
        simulate_camera_feedback(local_x, local_y, angular_z)

def stop_motors():
    mirte.drive(0.0, 0.0, 0.0)

def simulate_camera_feedback(local_vx, local_vy, angular_vz, dt=0.05):
    global current_x, current_y, current_angle, is_visible
    is_visible = True 
    current_x += (local_vx * math.cos(current_angle) - local_vy * math.sin(current_angle)) * dt
    current_y += (local_vx * math.sin(current_angle) + local_vy * math.cos(current_angle)) * dt
    current_angle = utils.angle_difference(current_angle + angular_vz * dt, 0.0)

# ==========================================
# 5. ARM SEQUENCES
# ==========================================
def execute_pickup_sequence():
    print("Executing Pickup Sequence...")

    print("Step 1: Opening gripper...")
    mirte.open_gripper()
    time.sleep(2)
    
    print("Step 2: Moving to vertical position...")
    mirte.move_arm_to(0.0, 0.0, 0.0, -1.0, 1) 
    time.sleep(2)

    print("Step 3: Pick-up position...")
    # completli horizontal
    mirte.move_arm_to(0.0, -1.5, 0.0, -1.0, 2) # shoulder_pan, shoulder_lift, elbow, wrist, duration
    time.sleep(3)

    print("Step 5: Closing gripper...")
    mirte.close_gripper(max_effort=25.0) 
    time.sleep(2)

    print("Step 6: Lifting to secure position...")
    # safe position
    mirte.move_arm_to(0.0, -0.5, 0.0, -1.0, 2)
    time.sleep(3)

    print("Pickup complete!")

def execute_dropoff_sequence():
    print("Executing Dropoff Sequence...")

    print("Step 2: Extending arm to drop position...")
    # same as safe position
    mirte.move_arm_to(0.0, -0.5, 0.0, -1.0, 2)
    time.sleep(3)

    print("Step 3: Opening gripper...")
    mirte.open_gripper()
    time.sleep(2)

    print("Dropoff complete!")

# ==========================================
# 6. MAIN STATE MACHINE
# ==========================================
def main():
    global target_x, target_y, target_angle, navigation_state, is_moving_allowed   
    global stuck_ref_x, stuck_ref_y, last_stuck_check_time 
    global is_start_captured, initial_x, initial_y, initial_angle
    global current_x, current_y, current_angle
    global TARGET_1_X, TARGET_1_Y, TARGET_1_ANGLE
    
    # START THE ROS ENGINE IMMEDIATELY
    executor = MultiThreadedExecutor()
    executor.add_node(mirte.node)
    ros_spin_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    ros_spin_thread.start()

    # IR Sensor Subscriptions
    mirte.node.create_subscription(Range, "/mirte/ir_left", on_receive_ir_left, 10)
    mirte.node.create_subscription(Range, "/mirte/ir_right", on_receive_ir_right, 10)

    # 1. WAIT FOR CAMERA CALIBRATION
    if TEST_MODE_NO_CAMERA:
        print("TEST MODE ACTIVE: Faking initial camera detection...")
        initial_x, initial_y, initial_angle = 0.0, 0.0, 0.0 # Set a fake start point
        current_x, current_y, current_angle = initial_x, initial_y, initial_angle
        is_start_captured = True
        is_moving_allowed = True
    else:
        print("Waiting for initial camera detection...")
        while not is_start_captured:
            time.sleep(0.1)
            
    print(f"Success! Robot located at X:{current_x:.2f}, Y:{current_y:.2f}")

    # ---------------------------------------------------------
    # 2. RANDOM TARGET SELECTION
    # ---------------------------------------------------------
    selected = random.choice(POTENTIAL_TARGETS)
    print(f"\nRANDOM TARGET SELECTED: Tag {selected['tag_id']}")
    print(f" Coordinates: ({selected['x']}, {selected['y']}) at {selected['angle']} rad")
    
    objective_queue.put(selected)
    
    is_moving_allowed = True # MANUAL OVERRIDE
    
    # Initialize the stuck detection reference point 
    stuck_ref_x = current_x
    stuck_ref_y = current_y
    last_stuck_check_time = time.time()


    # 2. TARGET & START OVERRIDE (Hardcoded for testing)
    #objective_queue.put({
    #    "x": TARGET_1_X, 
    #    "y": TARGET_1_Y, 
    #    "angle": TARGET_1_ANGLE, # Use your variable here!
    #    "tag_id": 99
    #})

    try:    
        # --- Stuck Detection Init ---
        stuck_ref_x, stuck_ref_y = current_x, current_y
        last_stuck_check_time = time.time()
        STUCK_TIME_THRESHOLD = 10.0 # Seconds
        STUCK_DIST_THRESHOLD = 0.10 # Meters (5cm)

        while rclpy.ok():

            # --- 1. STUCK DETECTION (Always running) ---
            dist_moved = math.sqrt((current_x - stuck_ref_x)**2 + (current_y - stuck_ref_y)**2)
            
            # Reset timer if we move more than 15cm
            if dist_moved > STUCK_DIST_THRESHOLD:
                stuck_ref_x, stuck_ref_y = current_x, current_y
                last_stuck_check_time = time.time()

            # Check if 8 seconds have passed without progress
            if (time.time() - last_stuck_check_time) > 8.0:
                if navigation_state in ["DELIVERING", "RETURNING"]:
                    out_of_local_minima() # The 40-cycle sequence runs now
                    
                    # IMPORTANT: Reset timer after escape
                    stuck_ref_x, stuck_ref_y = current_x, current_y
                    last_stuck_check_time = time.time()

            if is_moving_allowed and is_visible:

                if navigation_state == "IDLE":
                    if not objective_queue.empty():
                        next_objective = objective_queue.get() 
                        target_x, target_y = next_objective["x"], next_objective["y"]
                        target_angle = next_objective["angle"]
                        navigation_state = "DELIVERING"
                        print(f"Navigating to X:{target_x:.2f}, Y:{target_y:.2f}")
                    
                    else:
                        # NO TASKS: Are we already home?
                        dist_to_home = math.sqrt((PICKUP_X - current_x)**2 + (PICKUP_Y - current_y)**2)
                        
                        if dist_to_home > 0.25:
                            # If we are lost or just finished a delivery, go to pickup
                            target_x, target_y = PICKUP_X, PICKUP_Y
                            target_angle = PICKUP_ANGLE
                            navigation_state = "RETURNING"
                            print("Queue empty. Returning to Home Base...")
                        else:
                            # We are already home and holding an object. Just stop and wait.
                            stop_motors()
                            # Use end="\r" to keep the terminal clean
                            print("At Home Base. Waiting for Pioneer robots...", end="\r")
                        
                    
                elif navigation_state == "DELIVERING":
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

                    if dist_to_goal < 0.50: 
                        print("50cm from target! Entering Final Approach Mode...")
                        navigation_state = "FINAL_APPROACH"
                    else:
                        # Continue normal APF navigation
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)

                elif navigation_state == "FINAL_APPROACH":
                    # returns True when it touches the 22cm mark
                    is_docked = execute_final_approach()
                    
                    if is_docked:
                        execute_dropoff_sequence()
                        
                        # Set coordinates to the known Pickup Zone
                        target_x, target_y = PICKUP_X, PICKUP_Y
                        target_angle = PICKUP_ANGLE
                        navigation_state = "RETURNING"
                        print(f"Returning to Pickup Zone: X:{target_x:.2f}, Y:{target_y:.2f}")
                
                elif navigation_state == "RETURNING":
                    # We use 0.15m to safely arrive at the zone without Zeno's Paradox
                    if math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2) < 0.15: 
                        stop_motors()
                        print(f"Reached Pickup Zone. Aligning to angle {target_angle}...")
                        navigation_state = "ALIGNING_PICKUP"
                    else:
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
                   
                elif navigation_state == "ALIGNING_PICKUP":
                    # Calculate how far off we are from -1.55 radians
                    angle_error = utils.angle_difference(target_angle, current_angle)
                    
                    # If we are within ~3 degrees (0.05 rad), stop!
                    if abs(angle_error) < 0.05:
                        stop_motors()
                        print("\n Perfectly aligned at Pickup Position! Waiting for next task.")

                        # --- Start the pickup ---
                        time.sleep(2.0)
                        execute_pickup_sequence() 
                
                        navigation_state = "IDLE"
                        print("Pickup complete. Waiting for next objective.")
                    else:
                        # Spin in place! 
                        # We use your -1.5 inverted steering fix here too!
                        angular_z = utils.clamp(-0.7 * angle_error, -0.4, 0.4)
                        
                        mirte.drive(0.0, 0.0, angular_z) # 0.0 forward/sideways speed
                        
                        if TEST_MODE_NO_CAMERA:
                            simulate_camera_feedback(0.0, 0.0, angular_z)
            
            elif not is_visible:
                stop_motors()
                print("VISION LOST: Camera cannot see the AprilTag! Waiting for vision...", end="\r")

            time.sleep(0.05)
        
    except KeyboardInterrupt:
        print("\nManual shutdown. Stopping.")
        stop_motors()
    finally:
        executor.shutdown()
        ros_spin_thread.join()
        mirte.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()