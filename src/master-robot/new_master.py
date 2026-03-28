import sys
import os
import math
import time
import threading
import queue
import rclpy
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
# 1. INITIALIZATION & SETUP
# ==========================================
TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "uncertain-decoy-dandruff"
HOST = "172.18.0.2:8000"

rclpy.init()
mirte = Robot()
detector = Detector(True)
comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)

# --- SCORING GLOBALS ---
reported_tags = set()
last_detection_time = 0.0
DETECTION_INTERVAL = 0.5 # Scan twice per second

# ==========================================
# 2. STATE VARIABLES & APF TUNING
# ==========================================
current_x, current_y, current_angle = 0.0, 0.0, 0.0
is_moving_allowed = False 
is_visible = False 
is_start_captured = False
objective_queue = queue.Queue()

# TUNING FOR SLOW & STABLE VISION
MAX_SPEED = 0.25      # Lowered from 0.4 for better camera stability
K_ATT = 0.8           # Smoother attraction
K_REP = 0.15          # Reduced jitter
MAX_TOTAL_REP = 1.0   # Cap repulsion
D_OBSTACLE = 0.45
CHASSIS_IGNORE_ZONE = 0.25

# COORDINATES
PICKUP_X, PICKUP_Y, PICKUP_ANGLE = 5.25, 1.05, -1.55
TARGET_1_X, TARGET_1_Y, TARGET_1_ANGLE = 3.23, 4.6, 2.26

global_lidar_data = []
LIDAR_OFFSET_X = 0.10   
LIDAR_MOUNT_ANGLE = math.pi / 2
ir_left_dist, ir_right_dist = 1.0, 1.0
IR_THRESHOLD, K_IR_REP = 0.12, 2.5
navigation_state = "IDLE"
last_debug_time = 0.0

# ==========================================
# 3. CALLBACK FUNCTIONS
# ==========================================
def on_receive_location(x, y, angle, visible, last_seen):
    global current_x, current_y, current_angle, is_visible, is_start_captured
    if visible:
        if not is_start_captured:
            is_start_captured = True
        current_x, current_y, current_angle = x, y, angle
        is_visible = True
    else:
        is_visible = False

def on_receive_start():
    global is_moving_allowed
    is_moving_allowed = True

def on_receive_stop():
    global is_moving_allowed
    is_moving_allowed = False
    mirte.drive(0.0, 0.0, 0.0)

def on_receive_objective(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
    if team_id == TEAM_ID:
        objective_queue.put({"x": x, "y": y, "angle": angle, "tag_id": tag_id})

def on_receive_lidar(msg):
    global global_lidar_data
    global_lidar_data = list(msg.ranges)

def on_receive_ir_left(msg): global ir_left_dist; ir_left_dist = msg.range
def on_receive_ir_right(msg): global ir_right_dist; ir_right_dist = msg.range

# Register Callbacks
comm.register_callback_location(on_receive_location)
comm.register_callback_start(on_receive_start)
comm.register_callback_stop(on_receive_stop)
comm.register_callback_objective(on_receive_objective)
mirte.node.create_subscription(LaserScan, "/scan", on_receive_lidar, 10, callback_group=MutuallyExclusiveCallbackGroup())
mirte.node.create_subscription(Range, "/mirte/ir_left", on_receive_ir_left, 10)
mirte.node.create_subscription(Range, "/mirte/ir_right", on_receive_ir_right, 10)

# ==========================================
# 4. HELPER FUNCTIONS (Stubs for brevity, use your existing logic)
# ==========================================
def stop_motors(): mirte.drive(0.0, 0.0, 0.0)

def get_lidar_repulsive_forces():
    # Use your existing lidar force math here...
    return 0.0, 0.0 # Placeholder

def get_ir_repulsive_vector():
    # Use your existing IR force math here...
    return 0.0, 0.0 # Placeholder

def calculate_local_apf_vector():
    # Use your existing APF math here...
    return 0.0, 0.0 # Placeholder

# ==========================================
# 5. MAIN LOGIC
# ==========================================
def main():
    global target_x, target_y, target_angle, navigation_state, is_moving_allowed   
    global last_detection_time, reported_tags

    # --- THE EXECUTOR FIX ---
    # Combine everything into one MultiThreadedExecutor to stop IndexError
    executor = MultiThreadedExecutor()
    executor.add_node(mirte.node)
    executor.add_node(detector.node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("Waiting for initial camera localization...")
    while not is_start_captured:
        time.sleep(0.1)
            
    print(f"Robot Active at X:{current_x:.2f}, Y:{current_y:.2f}")

    # Manual Test Target
    objective_queue.put({"x": TARGET_1_X, "y": TARGET_1_Y, "angle": TARGET_1_ANGLE, "tag_id": 99})
    is_moving_allowed = True 

    try:    
        stuck_ref_x, stuck_ref_y = current_x, current_y
        last_stuck_check_time = time.time()

        while rclpy.ok():
            # --- 1. AGGRESSIVE SCORING ---
            # This runs regardless of navigation state
            now = time.time()
            if (now - last_detection_time) > DETECTION_INTERVAL:
                tags = detector.detect_objective_tags()
                if tags:
                    for tag in tags:
                        tag_id = int(tag.tag_id) # Type cast to fix "not an integer" error
                        if tag_id not in reported_tags:
                            # Use exactly 2 arguments as per your communication.py
                            if comm.send_objective_msg(TEAM_ID, tag_id):
                                reported_tags.add(tag_id)
                                print(f"🎯 SCORE! Discovered Tag {tag_id}")
                last_detection_time = now

            # --- 2. NAVIGATION ---
            if is_moving_allowed and is_visible:
                # [YOUR EXISTING STATE MACHINE LOGIC HERE]
                # Keep navigation_state logic (IDLE -> DELIVERING -> FINAL_APPROACH etc.)
                pass

            elif not is_visible:
                stop_motors()
                print("VISION LOST: Waiting for overhead camera...", end="\r")

            time.sleep(0.05)
        
    except KeyboardInterrupt:
        print("\nShutdown.")
    finally:
        stop_motors()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()