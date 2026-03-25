from time import sleep
import sys
import os
from mirte_robot import robot

# Test the wheel moving
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'pioneer-robot'))
import hardware as hd

# from "src/pioneer-robot" import hardware

# Runs once at the start
def setup():
    pass

# Main entry point to the program
def main():
    setup()

    # Initializes hardware n shit
    mirte = robot.createRobot()
    hd.set_motor_speed(mirte, 0, 0)
    
    while(1):
        print("Moving the motors")
        hd.set_motor_speed(mirte, 60, 60)
        sleep(1)
        print("Stopping the motors")
        hd.set_motor_speed(mirte, 0, 0)
        sleep(2)

main()