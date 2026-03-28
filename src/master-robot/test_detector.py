import sys
import os
import time
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library.robot_big import Robot
from library.detector import Detector
from library.communication import Communication

# --- CONFIGURATION ---
TEAM_ID, ROBOT_ID, PASSWORD = 5, 0, "uncertain-decoy-dandruff"
HOST = "172.18.0.2:8000"

def main():
    rclpy.init()
    
    # 1. Initialize Objects
    mirte = Robot()
    # Your Communication class only needs these for the URL and Auth
    comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)
    detector = Detector(True) 
    
    # 2. Setup MultiThreadedExecutor to stop the "IndexError"
    executor = MultiThreadedExecutor()
    executor.add_node(mirte.node)
    executor.add_node(detector.node)
    
    # Start the ROS processing in the background
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\n--- 🛰️ AGGRESSIVE SCANNER: Team {TEAM_ID} ---")
    print("Scanning for tags to send to the server...")

    reported_tags = set()

    try:
        while rclpy.ok():
            # 3. Detect AprilTags
            tags = detector.detect_objective_tags()
            
            if tags:
                for tag in tags:
                    # IMPORTANT: Force to standard Python int for struct.pack
                    tag_id = int(tag.tag_id)
                    
                    if tag_id not in reported_tags:
                        # 4. SEND TO SERVER
                        # Based on your file: send_objective_msg(target_team_id, tag_id)
                        # We send it to our own TEAM_ID so our team gets the points!
                        success = comm.send_objective_msg(TEAM_ID, tag_id)
                        
                        if success:
                            reported_tags.add(tag_id)
                            print(f"[{time.strftime('%H:%M:%S')}] 🎯 SUCCESS: Tag {tag_id} reported to Team {TEAM_ID}")
                        else:
                            print(f"⚠️ Failed to send Tag {tag_id}")

            time.sleep(0.2) # Save CPU

    except KeyboardInterrupt:
        print("\nScanner stopped.")
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()