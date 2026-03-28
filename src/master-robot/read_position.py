import sys
import os
import time

# --- FOLDER ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from library.communication import Communication

# ==========================================
# 1. SETUP
# ==========================================
TEAM_ID = 5
ROBOT_ID = 0
PASSWORD = "uncertain-decoy-dandruff"
HOST = "172.18.0.2:8000"

# ==========================================
# 2. CALLBACK FUNCTION
# ==========================================
def on_receive_location(x, y, angle, visible, last_seen):
    if visible:
        # Prints neatly on one line, updating constantly
        print(f"Robot POS: X: {x:.2f}  |  Y: {y:.2f}  |  Angle: {angle:.2f}", end="\r")
    else:
        print("AprilTag NOT visible to the overhead camera!        ", end="\r")

# ==========================================
# 3. MAIN LOOP
# ==========================================
def main():
    print(f"Connecting to Competition Server at {HOST}...")
    
    try:
        comm = Communication(host=HOST, team_id=TEAM_ID, robot_id=ROBOT_ID, password=PASSWORD)
        comm.register_callback_location(on_receive_location)
        
        print("Connected! Monitoring position... (Press Ctrl+C to stop)\n")

        # Keep the script alive forever to listen for messages
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nManual shutdown. Exiting monitor.")
    except Exception as e:
        print(f"\nFailed to connect to server: {e}")

if __name__ == "__main__":
    main()





#pickup position: X: 5.29, Y: 1.04, Angle: -1.55