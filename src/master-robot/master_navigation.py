import sys
import os
import math
import time
import threading
import queue
import rclpy
from sensor_msgs.msg import LaserScan

# Import from the libraries
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Now we can safely import from their library folder!
from library import utils
from library.robot_big import Robot
from library.communication import Communication

# ==========================================
# 1. INITIALIZATION & SETUP
# ==========================================

# CRITICAL: We must initialize ROS 2 before calling Robot()
rclpy.init()

# Dynamically grab credentials from the robot's OS using utils!
try:
    TEAM_ID, ROBOT_ID = utils.get_team_robot_id()
    PASSWORD = utils.get_password().strip()
    print(f"Logged in as Team {TEAM_ID}, Robot {ROBOT_ID}")
except Exception as e:
    print(f"Warning: Could not auto-load credentials (are you testing on your laptop?). Defaulting to Team 5.")
    TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "test3"

# Initialize Robot Hardware and Network
mirte = Robot()
HOST = "192.168.1.20:8000" # Update tomorrow if the organizers change the IP!
comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)

# Custom Queue for Objectives
objective_queue = queue.Queue()


# ==========================================
# 2. STATE VARIABLES & APF TUNING
# ==========================================

TEST_MODE_NO_CAMERA = True  # Set to True ONLY if testing without the ceiling camera!

current_x, current_y, current_angle = 0.0, 0.0, 0.0
is_moving_allowed = False 
is_visible = True 

target_x, target_y, target_angle = None, None, None

# Hardcoded Starting Zone (Update these tomorrow during practice!)
start_x = 0.5   
start_y = 1.0   
start_angle = 0.0   

navigation_state = "IDLE" # States: "IDLE", "DELIVERING", "RETURNING"

# APF Tuning Parameters
K_ATT = 1.0       
K_REP = 1.0       
D_OBSTACLE = 0.4
MAX_SPEED = 0.4   
ROBOT_RADIUS = 0.15

global_lidar_data = []

# --- DEBUG TIMER ---
last_debug_time = 0.0


# ==========================================
# 3. CALLBACK FUNCTIONS
# ==========================================

def on_receive_location(x, y, angle, visible, last_seen):
    global current_x, current_y, current_angle, is_visible
    current_x = x
    current_y = y
    current_angle = angle
    is_visible = visible

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
    global global_lidar_data
    global_lidar_data = list(msg.ranges)

# Register callbacks with the network and ROS
comm.register_callback_location(on_receive_location)
comm.register_callback_start(on_receive_start)
comm.register_callback_stop(on_receive_stop)
comm.register_callback_objective(on_receive_objective)
mirte.node.create_subscription(LaserScan, "/scan", on_receive_lidar, 10)


# ==========================================
# 4. APF NAVIGATION LOGIC
# ==========================================

def get_lidar_repulsive_forces():
    global global_lidar_data
    rep_force_local_x = 0.0
    rep_force_local_y = 0.0
    
    if not global_lidar_data:
        return 0.0, 0.0 

    num_points = len(global_lidar_data)
    if num_points == 0:
        return 0.0, 0.0

    angle_increment = (2 * math.pi) / num_points 

    for i, distance in enumerate(global_lidar_data):
        # Ignore empty space (inf/NaN) and the chassis itself (<0.05m)
        if math.isinf(distance) or math.isnan(distance):
            continue

        if distance <= ROBOT_RADIUS: 
            continue
            
        if ROBOT_RADIUS < distance < D_OBSTACLE: 
            beam_angle = i * angle_increment
            force_mag = K_REP * ((1.0 / distance) - (1.0 / D_OBSTACLE)) * (1.0 / (distance**2))
            
            rep_force_local_x -= force_mag * math.cos(beam_angle)
            rep_force_local_y -= force_mag * math.sin(beam_angle)
            
    return rep_force_local_x, rep_force_local_y

def calculate_local_apf_vector():
    global last_debug_time  # for debug printing

    if target_x is None or target_y is None:
        return 0.0, 0.0 
        
    global_dx = target_x - current_x
    global_dy = target_y - current_y
    dist_to_goal = math.sqrt(global_dx**2 + global_dy**2)
    
    if dist_to_goal < 0.15: 
        return 0.0, 0.0
        
    global_angle_to_goal = math.atan2(global_dy, global_dx)
    local_angle_to_goal = utils.angle_difference(global_angle_to_goal, current_angle) # Using utils!
    
    att_force_local_x = K_ATT * dist_to_goal * math.cos(local_angle_to_goal)
    att_force_local_y = K_ATT * dist_to_goal * math.sin(local_angle_to_goal)
    
    rep_force_local_x, rep_force_local_y = get_lidar_repulsive_forces()

    # Total forces
    total_x = att_force_local_x + rep_force_local_x
    total_y = att_force_local_y + rep_force_local_y

    # DEBUGGING: Print forces every 2 seconds
    current_time = time.time()
    if current_time - last_debug_time > 0.5:
        print(f"\n--- [{navigation_state}] ---")
        print(f"POS:  X:{current_x:.2f}  Y:{current_y:.2f}  |  TGT: X:{target_x:.2f}  Y:{target_y:.2f}  |  DIST: {dist_to_goal:.2f}m")
        print(f"ATT:  x:{att_force_local_x:>5.2f}  y:{att_force_local_y:>5.2f}")
        print(f"REP:  x:{rep_force_local_x:>5.2f}  y:{rep_force_local_y:>5.2f}")
        print(f"OUT:  x:{total_x:>5.2f}  y:{total_y:>5.2f}")
        last_debug_time = current_time
    
    return total_x, total_y

def calculate_angular_velocity():
    if target_angle is None:
        return 0.0
        
    angle_error = utils.angle_difference(target_angle, current_angle) # Using utils!
    K_ANGULAR = 0.8 
    angular_z = K_ANGULAR * angle_error
    return utils.clamp(angular_z, -0.5, 0.5) # Using utils!

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
    current_angle = utils.angle_difference(current_angle, 0.0) # Normalize


# ==========================================
# 5. ARM SEQUENCES
# ==========================================

def execute_pickup_sequence():
    print("Executing Pickup Sequence...")
    # mirte.open_gripper()
    # mirte.move_arm_to(0.0, 1.0, -1.0, 0.5, 2.0)
    # mirte.close_gripper()
    time.sleep(2) 
    print("Pickup complete!")

def execute_dropoff_sequence():
    print("Executing Dropoff Sequence...")
    # mirte.move_arm_to(0.0, 0.5, -0.5, 0.0, 2.0)
    # mirte.open_gripper()
    time.sleep(2) 
    print("Dropoff complete!")


# ==========================================
# 6. MAIN STATE MACHINE
# ==========================================

def main():
    global target_x, target_y, target_angle, navigation_state, is_moving_allowed
    
    print("\n" + "="*40)
    print(" MIRTE MASTER NAV: READY")
    print("="*40)
    print("Starting in 3 seconds... STAND CLEAR!")
    time.sleep(3) 
    
    # ⚠️ OFFLINE TEST OVERRIDE: 
    is_moving_allowed = True
    objective_queue.put({"x": 1.5, "y": 1.5, "tag_id": 99}) 
    print("Offline Test Objective Injected.\n")
    
    try:
        while True:
            if is_moving_allowed:
                if not is_visible:
                    stop_motors()
                    time.sleep(0.05)
                    continue

                if navigation_state == "IDLE":
                    if not objective_queue.empty():
                        execute_pickup_sequence()
                        next_objective = objective_queue.get() 
                        target_x = next_objective["x"]
                        target_y = next_objective["y"]
                        navigation_state = "DELIVERING"
                        print(f"Navigating to Objective at X:{target_x:.2f}, Y:{target_y:.2f}")

                elif navigation_state == "DELIVERING":
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    if dist_to_goal < 0.15: 
                        stop_motors()
                        execute_dropoff_sequence()
                        target_x, target_y = start_x, start_y
                        navigation_state = "RETURNING"
                        print(f"Returning to Home Base.")
                    else:
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
                
                elif navigation_state == "RETURNING":
                    target_angle = start_angle 
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    if dist_to_goal < 0.05: 
                        stop_motors()
                        target_x, target_y, target_angle = None, None, None
                        navigation_state = "IDLE"
                        print("Arrived back at Home Base. Waiting in IDLE.")
                    else:
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
            else:
                stop_motors()
                
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nManual shutdown (Ctrl+C). Stopping motors immediately.")
        stop_motors()

if __name__ == "__main__":
    # Spin the ROS 2 node in the background so LiDAR receives data!
    ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(mirte.node), daemon=True)
    ros_spin_thread.start()
    
    time.sleep(1) 
    main()