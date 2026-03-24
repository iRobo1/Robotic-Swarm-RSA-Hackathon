from big_mirte_library import Robot
import time
robot = Robot(team_id=None, authorization=None)

driving_time = 1 #value we want 
t_end = time.time() + driving_time


# Go to work dir: cd workdir/
# Run script: python3 workshop.py
# Stop script: ctrl + c



# Driving
while time.time() < t_end:
    # Robot drive uses linear_x, linear_y, and angular_z as input. 
    # Maximum speed = 0.4 m/s
    robot.drive(0.2, 0.0, 0.0) ## initial
    robot.drive(0.0, 0.0, 0.0) ## stop after loop
    time.sleep(0.5) ## recomend before fully stop 



# Receiving Messages: 


# Sending Messages: 

Robot.close_gripper()
Robot.open_gripper()