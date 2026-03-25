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

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library import utils
from library.robot_big import Robot
from library.communication import Communication

# ==========================================
# 1. INITIALIZATION & SETUP
# ==========================================
rclpy.init()

try:
    TEAM_ID, ROBOT_ID = utils.get_team_robot_id()
    PASSWORD = utils.get_password().strip()
    print(f"Logged in as Team {TEAM_ID}, Robot {ROBOT_ID}")
except Exception as e:
    print(f"Warning: Could not auto-load credentials. Defaulting to Team 5.")
    TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "uncertain-decoy-dandruff"

# Try adding a small delay before connecting to let the network stabilize
print("Waiting for network...")
time.sleep(1)

mirte = Robot()
HOST = "172.18.0.2:8000"  
comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)

objective_queue = queue.Queue()

# ==========================================
# 2. STATE VARIABLES & APF TUNING
# ==========================================
TEST_MODE_NO_CAMERA = False  

current_x, current_y, current_angle = 0.0, 0.0, 0.0
is_moving_allowed = False 
is_visible = True 

target_x, target_y, target_angle = None, None, None

start_x, start_y, start_angle = 0.5, 1.0, 0.0   
navigation_state = "IDLE" 

# APF Tuning Parameters
K_ATT = 1.0       
K_REP = 0.5       
D_OBSTACLE = 0.4
MAX_SPEED = 0.4   
ROBOT_RADIUS = 0.15     
LIDAR_OFFSET_X = 0.10   
LIDAR_MOUNT_ANGLE = math.pi / 2  # 90 degrees

# --- STARTING SNAPSHOT ---
initial_x = 0.0
initial_y = 0.0
initial_angle = 0.0
is_start_captured = False # We'll use this to wait for the first camera frame


global_lidar_data = []

# Global Timers for cleanly printing to the terminal
last_lidar_print = 0.0
last_debug_time = 0.0



# ==========================================
# 3. CALLBACK FUNCTIONS
# ==========================================
def on_receive_location(x, y, angle, visible, last_seen):
    global current_x, current_y, current_angle, is_visible
    global initial_x, initial_y, initial_angle, is_start_captured
    
    if visible:
        if not is_start_captured:
            initial_x = x
            initial_y = y
            initial_angle = angle
            is_start_captured = True
            print(f"📍 Initial Global POS Captured: X:{x:.2f}, Y:{y:.2f}")

        # RAW GLOBAL values for navigation
        current_x = x
        current_y = y
        current_angle = angle
        
        is_visible = True
    else:
        is_visible = False

def on_receive_start():
    global is_moving_allowed
    is_moving_allowed = True
    print("Competition Started! Moving allowed.")

def on_receive_stop():
    global is_moving_allowed
    is_moving_allowed = False
    stop_motors()
    print("STOP MESSAGE RECEIVED. Stop immediately.")

def on_receive_objective(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
    objective_queue.put({"x": x, "y": y, "tag_id": tag_id})
    print(f"Objective {tag_id} found at X:{x}, Y:{y}. Added to queue.")

def on_receive_lidar(msg):
    global global_lidar_data, last_lidar_print
    global_lidar_data = list(msg.ranges)


comm.register_callback_location(on_receive_location)
comm.register_callback_start(on_receive_start)
comm.register_callback_stop(on_receive_stop)
comm.register_callback_objective(on_receive_objective)
mirte.node.create_subscription(LaserScan, "/scan", on_receive_lidar, 10, callback_group=MutuallyExclusiveCallbackGroup())

# ==========================================
# 4. APF NAVIGATION LOGIC
# ==========================================
def get_lidar_repulsive_forces():
    global global_lidar_data
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

        # 1. BLINDFOLD: Now correctly ignores the back 180 relative to the FRONT
        if abs(norm_angle) > (math.pi / 2):
            continue

        CHASSIS_IGNORE_ZONE = 0.25
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
    
    att_force_local_x = K_ATT * dist_to_goal * math.cos(local_angle_to_goal)
    att_force_local_y = K_ATT * dist_to_goal * math.sin(local_angle_to_goal)
    
    rep_force_local_x, rep_force_local_y = get_lidar_repulsive_forces()

    total_x = att_force_local_x + rep_force_local_x
    total_y = att_force_local_y + rep_force_local_y

    # ==========================================
    # DEBUGGING DASHBOARD
    # ==========================================
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
        print(f"REP:  x:{rep_force_local_x:>5.2f}  y:{rep_force_local_y:>5.2f}")
        print(f"OUT:  x:{total_x:>5.2f}  y:{total_y:>5.2f}")
        print(f"CLOSEST VALID OBSTACLE: {obs_string}")
        last_debug_time = current_time
    
    return total_x, total_y

def calculate_angular_velocity():
    if target_angle is None:
        return 0.0
    angle_error = utils.angle_difference(target_angle, current_angle) 
    return utils.clamp(0.8 * angle_error, -0.5, 0.5) 

def apply_motor_commands(local_x, local_y):
    angular_z = calculate_angular_velocity() 

    if local_x == 0 and local_y == 0 and angular_z == 0:
        stop_motors()
        return

    magnitude = math.sqrt(local_x**2 + local_y**2)
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
    global_dx = (local_vx * math.cos(current_angle) - local_vy * math.sin(current_angle)) * dt
    global_dy = (local_vx * math.sin(current_angle) + local_vy * math.cos(current_angle)) * dt
    current_x += global_dx
    current_y += global_dy
    current_angle += angular_vz * dt
    current_angle = utils.angle_difference(current_angle, 0.0) 

# ==========================================
# 5. ARM SEQUENCES
# ==========================================
def execute_pickup_sequence():
    print("Executing Pickup Sequence...")
    time.sleep(2) 
    print("Pickup complete!")

def execute_dropoff_sequence():
    print("Executing Dropoff Sequence...")
    time.sleep(2) 
    print("Dropoff complete!")

# ==========================================
# 6. MAIN STATE MACHINE
# ==========================================
def main():
    global target_x, target_y, target_angle, navigation_state, is_moving_allowed   
    
    # START THE ROS ENGINE IMMEDIATELY
    executor = MultiThreadedExecutor()
    executor.add_node(mirte.node)
    ros_spin_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    ros_spin_thread.start()

    # 1. WAIT FOR CAMERA CALIBRATION
    print("Waiting for initial camera detection...")
    while not is_start_captured:
        time.sleep(0.1)
    print(f"Succesfull, Robot located at Global X:{current_x:.2f}, Y:{current_y:.2f}")

    # 2. TARGET & START OVERRIDE (Hardcoded for testing)
    objective_queue.put({"x": 1.5, "y": 1.5, "tag_id": 99}) 
    is_moving_allowed = True # <--- MANUAL START FOR TESTING

    try:    
        while rclpy.ok():
            if is_moving_allowed:
                if not is_visible:
                    stop_motors()
                    time.sleep(0.1)
                    continue

                if navigation_state == "IDLE":
                    if not objective_queue.empty():
                        execute_pickup_sequence()
                        next_objective = objective_queue.get() 
                        target_x = next_objective["x"]
                        target_y = next_objective["y"]
                        navigation_state = "DELIVERING"
                        print(f"🚀 Navigating to X:{target_x:.2f}, Y:{target_y:.2f}")

                elif navigation_state == "DELIVERING":
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    
                    if dist_to_goal < 0.15: 
                        stop_motors()
                        print("Arrived at Global Target!")
                        execute_dropoff_sequence()
                        
                        # --- CHANGE: Return to the initial snapshot (for testing, after: hardcode this) ---
                        target_x, target_y = initial_x, initial_y
                        navigation_state = "RETURNING"
                        print(f"Returning to Start: X:{target_x:.2f}, Y:{target_y:.2f}")
                    else:
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
                
                elif navigation_state == "RETURNING":
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    if dist_to_goal < 0.10: 
                        stop_motors()
                        navigation_state = "IDLE"
                        print("Back at Home Base.")
                    else:
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
            else:
                stop_motors()
            
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