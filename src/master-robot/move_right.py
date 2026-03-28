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
    spin_thread = Thread(target=lambda: rclpy.spin(mirte.node), daemon=True)
    spin_thread.start()

    print("Waiting for AprilTag vision...")
    while not is_visible:
        time.sleep(0.01)

    # 3. Capture Initial Position
    start_x, start_y = current_x, current_y
    print(f"Start: ({start_x:.2f}, {start_y:.2f})")
    
    # 4. Movement Loop
    target_dist = 0.20 # 20cm
    side_speed = -0.5  # Negative for Right. Increased for high friction.
    
    print(f"Strafing until 20cm movement detected...")

    try:
        while True:
            # Calculate actual distance moved from start
            dist_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            print(f"Progress: {dist_moved:.3f}m / {target_dist}m", end="\r")

            if dist_moved >= target_dist:
                break
            
            if is_visible:
                # Heartbeat command
                mirte.drive(0.0, side_speed, 0.0)
            else:
                mirte.drive(0.0, 0.0, 0.0)
                print("\nVISION LOST - Pausing...")
                
            time.sleep(0.05)

        mirte.drive(0.0, 0.0, 0.0)
        print(f"\nFinished! Moved: {dist_moved:.2f}m")

    except KeyboardInterrupt:
        mirte.drive(0.0, 0.0, 0.0)

    rclpy.shutdown()

if __name__ == "__main__":
    main()