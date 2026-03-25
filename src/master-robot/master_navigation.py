import math
import time
import threading
import asyncio
from big_mirte_library import Robot, Communication

# Initialize the robot 
mirte = Robot(team_id=5, authorization=None, dev_mode=TEST_MODE)
comm = Communication()

# --- STATE VARIABLES ---
TEST_MODE = True
TEST_MODE_NO_CAMERA = False # Set to True for simulation without camera
current_x = 0.0
current_y = 0.0
current_angle = 0.0
is_moving_allowed = False 
is_visible = True # Assume we start with vision. 

target_x = None
target_y = None
target_angle = None

# Defining starting state
start_x = 0.5   # CHANGE!! Placeholder: 0.5 meters from the X edge
start_y = 1.0   # CHANGE!! Placeholder: 1.0 meters from the Y edge
start_angle = 0.0   # Define angle in starting line (in radians: 0.0 Forward (x-axis), pi/2 Left (y-axis), pi Backward, -pi/2 Right)
navigation_state = "IDLE" # States: "IDLE", "DELIVERING", "RETURNING"

# APF Tuning Parameters
K_ATT = 1.0       
K_REP = 0.5       
D_OBSTACLE = 0.5  
MAX_SPEED = 0.4   # Strictly enforced rule: max 0.4 m/s


# Register the callbacks
comm.register_callback_location(on_receive_location)
comm.register_callback_start(on_receive_start)
comm.register_callback_stop(on_receive_stop)
comm.register_callback_objective(mirte.on_receive_objective) # The library's Robot class already has an 'on_receive_objective' method that safely puts the data into 'mirte.objective_queue'. We just map it here!


# --- CALLBACK FUNCTIONS ---

def on_receive_location(x, y, angle, visible, last_seen):
    """Updates the robot's current position."""
    global current_x, current_y, current_angle, is_visible
    current_x = x
    current_y = y
    current_angle = angle
    is_visible = visible    # safe stop in case we lose vision of the robot

    # Passing location data to the Robot class to keep its internal state updated
    mirte.on_receive_location(x, y, angle, visible, last_seen)

def on_receive_start():
    """Starts the robot's movement."""
    global is_moving_allowed
    is_moving_allowed = True
    mirte.on_receive_start() 
    print("Competition Started! Moving allowed.")

def on_receive_stop():
    """Must stop the robot within 3 seconds."""
    global is_moving_allowed
    is_moving_allowed = False
    stop_motors()
    mirte.on_receive_stop()
    print("STOP MESSAGE RECEIVED. Stop immediately.")

def on_receive_objective(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
    """Triggered when a Pioneer finds an objective."""
    global target_x, target_y
    # In a real scenario, you'd check if this objective matches your team
    # and if you are currently holding the right item.
    target_x = x
    target_y = y
    print(f"Objective found at X:{x}, Y:{y}. Setting as new target.")

# --- NAVIGATION LOGIC (APF) ---

def get_lidar_repulsive_forces():
    """Calculates the push away from obstacles using the 360 LiDAR array."""
    rep_force_local_x = 0.0
    rep_force_local_y = 0.0
    
    # Safely check if the lidar_data array exists and is populated
    if not hasattr(mirte, 'lidar_data') or not mirte.lidar_data:
        return 0.0, 0.0 

    lidar_array = mirte.lidar_data 
    num_points = len(lidar_array)
    
    if num_points == 0:
        return 0.0, 0.0

    angle_increment = (2 * math.pi) / num_points 

    for i, distance in enumerate(lidar_array):
        # Ignore infinity and NaN values from empty space
        if math.isinf(distance) or math.isnan(distance):
            continue

        # 0.05 ignores junk data from inside the robot chassis
        if 0.05 < distance < D_OBSTACLE: 
            beam_angle = i * angle_increment
            force_mag = K_REP * ((1.0 / distance) - (1.0 / D_OBSTACLE)) * (1.0 / (distance**2))
            
            rep_force_local_x -= force_mag * math.cos(beam_angle)
            rep_force_local_y -= force_mag * math.sin(beam_angle)
            
    return rep_force_local_x, rep_force_local_y


def calculate_local_apf_vector():
    """Calculates the final X and Y forces entirely in the robot's local view."""
    if target_x is None or target_y is None:
        return 0.0, 0.0 
        
    # 1. LOCAL ATTRACTIVE FORCE
    global_dx = target_x - current_x
    global_dy = target_y - current_y
    dist_to_goal = math.sqrt(global_dx**2 + global_dy**2)
    
    if dist_to_goal < 0.15: # Arrived within 15cm
        return 0.0, 0.0
        
    global_angle_to_goal = math.atan2(global_dy, global_dx)
    local_angle_to_goal = global_angle_to_goal - current_angle
    
    # Normalize angle
    while local_angle_to_goal > math.pi: local_angle_to_goal -= 2 * math.pi
    while local_angle_to_goal < -math.pi: local_angle_to_goal += 2 * math.pi
    
    att_force_local_x = K_ATT * dist_to_goal * math.cos(local_angle_to_goal)
    att_force_local_y = K_ATT * dist_to_goal * math.sin(local_angle_to_goal)
    
    # 2. LOCAL REPULSIVE FORCE
    rep_force_local_x, rep_force_local_y = get_lidar_repulsive_forces()
    
    # 3. TOTAL LOCAL FORCE
    total_x = att_force_local_x + rep_force_local_x
    total_y = att_force_local_y + rep_force_local_y
    
    return total_x, total_y

def calculate_angular_velocity():
    """Calculates the rotation speed needed to face the target_angle."""
    if target_angle is None:
        return 0.0
        
    # Calculate difference between current heading and desired heading
    angle_error = target_angle - current_angle
    
    # Normalize the error so the robot takes the shortest spinning path (-pi to pi)
    while angle_error > math.pi: angle_error -= 2 * math.pi
    while angle_error < -math.pi: angle_error += 2 * math.pi
    
    # Proportional control: turn faster if far away, slower if close
    K_ANGULAR = 0.8 
    angular_z = K_ANGULAR * angle_error
    
    # Cap the maximum spin speed so it doesn't get dizzy
    MAX_SPIN = 0.5 
    if angular_z > MAX_SPIN: angular_z = MAX_SPIN
    elif angular_z < -MAX_SPIN: angular_z = -MAX_SPIN
        
    return angular_z

def apply_motor_commands(local_x, local_y):
    """Safely limits speed and sends local forces directly to the Mecanum wheels."""

    angular_z = calculate_angular_velocity()    # Get rotation to face the target

    if local_x == 0 and local_y == 0 and angular_z == 0:
        stop_motors()
        return

    magnitude = math.sqrt(local_x**2 + local_y**2)
    
    if magnitude > MAX_SPEED:
        scaling_factor = MAX_SPEED / magnitude
        local_x *= scaling_factor
        local_y *= scaling_factor

    # Drive command: linear_x (forward/back), linear_y (left/right), angular_z (rotation)
    mirte.drive(local_x, local_y, angular_z) 

    # UPDATE THE FAKE CAMERA (If in testing mode)
    if TEST_MODE_NO_CAMERA:
        simulate_camera_feedback(local_x, local_y, angular_z)


def stop_motors():
    mirte.drive(0.0, 0.0, 0.0)

# FAKE CAMERA SIMULATION (For Testing Without Actual Vision)
def simulate_camera_feedback(local_vx, local_vy, angular_vz, dt=0.05):
    """Guesses the robot's new global position based on its speed."""
    global current_x, current_y, current_angle, is_visible
    
    # In test mode, we are always "visible"
    is_visible = True 
    
    # Convert local wheel velocities to global map movements
    global_dx = (local_vx * math.cos(current_angle) - local_vy * math.sin(current_angle)) * dt
    global_dy = (local_vx * math.sin(current_angle) + local_vy * math.cos(current_angle)) * dt
    
    # Update the robot's internal tracker
    current_x += global_dx
    current_y += global_dy
    current_angle += angular_vz * dt
    
    # Normalize the angle
    while current_angle > math.pi: current_angle -= 2 * math.pi
    while current_angle < -math.pi: current_angle += 2 * math.pi



# --- ARM & GRIPPER INTEGRATION ---
# This should call the pickup and dropoff sequences

def execute_pickup_sequence():
    """Called when leaving IDLE state to grab an item from the start zone."""
    print("Executing Pickup Sequence...")
    # e.g., mirte.open_gripper()
    # e.g., mirte.move_arm_to(0.0, 1.0, -1.0, 0.5, 2.0)
    # e.g., mirte.close_gripper()
    time.sleep(2) 
    print("Pickup complete!")

def execute_dropoff_sequence():
    """Called when arriving at the target basket to release the item."""
    print("Executing Dropoff Sequence...")
    # e.g., mirte.move_arm_to(0.0, 0.5, -0.5, 0.0, 2.0)
    # e.g., mirte.open_gripper()
    # e.g., mirte.move_arm_to(0.0, 0.0, 0.0, 0.0, 2.0) # Reset arm safely
    time.sleep(2) 
    print("Dropoff complete!")


# --- MAIN LOOP ---

def main():
    global target_x, target_y, target_angle, navigation_state, is_moving_allowed
    print("Mirte Master Navigation Initialized. Waiting for start message...")

    # --- AUTOMATIC HARDWARE START ---
    if TEST_MODE:
        print("\n------TEST MODE ACTIVE-----")
        print("Robot will start moving in 3 seconds...")
        time.sleep(3) # 3seconds delay
        
        is_moving_allowed = True
        mirte.on_receive_start() # Force the library into the STARTED state

        # Inject a fake objective so the Master has somewhere to drive!
        mirte.objective_queue.put({"x": 1.5, "y": 1.5}) 
        print("Test objective position injected.\n")
    
    try:
        while True:
            # Only execute movement logic if the competition is running
            if is_moving_allowed:

                # SAFETY OVERRIDE: Can the camera see the Robot?
                if not is_visible:
                    stop_motors()
                    # Add moving the arm 
                    # We skip the rest of the loop until the camera sees us again
                    time.sleep(0.05)
                    continue

                # STATE 1: IDLE (Waiting at Starting Zone)
                if navigation_state == "IDLE":
                    # Check if Pioneers found something
                    if not mirte.objective_queue.empty():
                        print("Objective in queue! Preparing for departure...")
                        
                        # Grab the item from the start zone
                        execute_pickup_sequence()
                        
                        # Pop the objective and set our new target
                        next_objective = mirte.objective_queue.get() 
                        target_x = next_objective["x"]
                        target_y = next_objective["y"]
                        
                        # Switch state to drive
                        navigation_state = "DELIVERING"
                        print(f"Item secured. Navigating to Objective at X:{target_x:.2f}, Y:{target_y:.2f}")

                # STATE 2: DELIVERING (Driving to Basket)
                elif navigation_state == "DELIVERING":
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    
                    if dist_to_goal < 0.15: # Arrived at the 15cm basket!
                        print("Arrived at target! Halting for dropoff.")
                        stop_motors()
                        
                        # Drop the item in the basket
                        execute_dropoff_sequence()
                        
                        # Set target back to home and switch state
                        target_x = start_x
                        target_y = start_y
                        navigation_state = "RETURNING"
                        print(f"Dropoff complete. Returning to Home Base at X:{target_x:.2f}, Y:{target_y:.2f}")
                        
                    else:
                        # Keep driving via APF
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
                
                # STATE 3: RETURNING (Driving Home)
                elif navigation_state == "RETURNING":
                    target_angle = start_angle # We want to face the line, to pick up the next item
                    dist_to_goal = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                    
                    if dist_to_goal < 0.05: # Tolerance for the starting position
                        print("Arrived back at Home Base. Ready for next item.")
                        stop_motors()
                        
                        # Clear targets and wait for the next queue item
                        target_x = None
                        target_y = None
                        target_angle = None
                        navigation_state = "IDLE"
                        
                    else:
                        # Keep driving via APF
                        local_x, local_y = calculate_local_apf_vector()
                        apply_motor_commands(local_x, local_y)
            
            else:
                # The game is paused or stopped by the referees
                stop_motors()
                
            # Maintain a 20 Hz control loop
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Manual shutdown triggered. Stopping motors.")
        stop_motors()

#if __name__ == "__main__":
#    main()

def start_websocket_listener():
    """Runs the async websocket connection in a separate event loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    # This function comes from the library you provided
    loop.run_until_complete(comm.connect_and_listen_to_websocket())

if __name__ == "__main__":
    # Spin up the server listener in the background
    listener_thread = threading.Thread(target=start_websocket_listener, daemon=True)
    listener_thread.start()

    # Spin the ROS 2 Node so LiDAR and Camera callbacks actually fire
    ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(mirte.node), daemon=True)
    ros_spin_thread.start()
    
    # Give the connection a brief second to establish before driving
    time.sleep(1) 
    
    # 2. Start the main navigation brain
    main()