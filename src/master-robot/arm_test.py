import sys
import os
import rclpy
import time
import threading

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir) # Should be 'src'
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library.robot_big import Robot

def main():
    rclpy.init()
    mirte = Robot()

    # Background thread to handle ROS communication
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mirte.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print("Executing Arm Move...")

    # ==========================================
    # ARM COORDINATES:
    # format: (pan, lift, elbow, wrist, duration)
    # ==========================================
    
    mirte.open_gripper()
    time.sleep(1)

    # Example: Move to a test position
    mirte.move_arm_to(0.0, 0.0, 0.0, -1.0, 1)
    time.sleep(3)

    mirte.move_arm_to(0.0, -1.5, 0.0, -1.0, 2)
    time.sleep(3)

    mirte.close_gripper()
    time.sleep(1)

    mirte.move_arm_to(0.0, -0.5, 0.0, -1.0, 2)
    time.sleep(3)

    print("Arm movement completed")

    # Shutdown
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()