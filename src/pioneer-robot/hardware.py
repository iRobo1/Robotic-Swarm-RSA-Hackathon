# This file contains all low-level calls to control and read data from the robot hardware, 
# including the motors, camera (RGB or RGBD), ultrasonic sensors, and LiDAR.

from cv2 import Mat

# Example function
def set_motor_speed(mirte, left_motor_speed: int, right_motor_speed: int) -> None:
    NotImplemented

    # mirte is an argument because we can use it to call ROS services
    mirte.setMotorSpeed("left", left_motor_speed)
    mirte.setMotorSpeed("right", right_motor_speed)

    # ROS names for motors
    # motor/left 
    # motor/right

def getDistances(mirte):
    return [mirte.getDistance("left"), mirte.getDistance("right")]

    # ROS name for sonar sensors
    # distance/left
    # distance/right

# Take a picture with the robot's camera and return it
def capture_RGB_image(mirte) -> Mat:
    NotImplemented
    
    # ROS topic for camera frame:
    # /video1/image_raw/compressed
