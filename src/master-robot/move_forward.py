import sys
import os
import math
import time
import rclpy
from threading import Thread

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library.robot_big import Robot
from library.communication import Communication

# Global trackers
current_x, current_y = 0.0, 0.0
is_visible = False

def on_receive_location(x, y, angle, visible, last_seen):
    global current_x, current_y, is_visible
    if visible:
        current_x, current_y = x, y
        is_visible = True
    else:
        is_visible = False

def main():
    global current_x, current_y, is_visible
    rclpy.init()
    mirte = Robot()
    
    # 1. Setup Communication
    TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "uncertain-decoy-dandruff"
    HOST = "172.18.0.2:8000"
    comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)
    comm.register_callback_location(on_receive_location)

    # 2. Start Background Spinning
    # This allows the robot to "listen" to the camera while the loop is running
    spin_thread = Thread(target=lambda: rclpy.spin(mirte.node), daemon=True)
    spin_thread.start()

    print("Checking vision...")
    while not is_visible:
        time.sleep(0.01)

    # 3. Capture Initial Position
    start_x, start_y = current_x, current_y
    print(f"Starting Position: ({start_x:.2f}, {start_y:.2f})")
    
    # 4. Movement Parameters
    target_dist = 0.20  # 20cm
    forward_speed = 0.4 # POSITIVE = FORWARD
    TIMEOUT = 4.0       # Safety cutoff in case it hits a wall
    start_time = time.time()
    
    print(f"Driving FORWARD until camera detects 20cm movement...")

    try:
        while True:
            # Euclidean Distance formula
            dist_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            print(f"Forward Progress: {dist_moved:.3f}m / {target_dist}m", end="\r")

            # Check if we reached the goal
            if dist_moved >= target_dist:
                print(f"\nTarget Reached!")
                break
            
            # Safety Timeout
            if (time.time() - start_time) > TIMEOUT:
                print(f"\n⏱TIMEOUT: Movement stopped for safety.")
                break
            
            if is_visible:
                # mirte.drive(vx, vy, az)
                mirte.drive(forward_speed, 0.0, 0.0)
            else:
                # Stop if we lose vision so we don't overshoot
                mirte.drive(0.0, 0.0, 0.0)
                print("\nVISION LOST - Pausing...")
                
            time.sleep(0.05)

        # Full Stop
        mirte.drive(0.0, 0.0, 0.0)

    except KeyboardInterrupt:
        mirte.drive(0.0, 0.0, 0.0)
        print("\nStopped manually.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()