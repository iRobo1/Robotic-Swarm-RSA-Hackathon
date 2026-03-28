# This file contains all low-level calls to control and read data from the robot hardware, 
# including the motors, camera (RGB or RGBD), ultrasonic sensors, and LiDAR.

from cv2 import Mat
import numpy as np

def control_to_motor_commands(self, v, omega, wheel_base=0.15):
    """
    Convert (v, omega) to differential-drive left/right motor commands in [-100, 100].

    Dead-zone: any wheel command with |value| < 10 is zeroed (motor won't turn).

    Args:
        v:          linear velocity  (m/s)
        omega:      angular velocity (rad/s)
        wheel_base: distance between wheels (m), default 0.15 m

    Returns:
        (left, right): int motor commands in [-100, 100]
    """
    # Differential drive inverse kinematics
    v_left  = v - (omega * wheel_base / 2.0)
    v_right = v + (omega * wheel_base / 2.0)

    # Scale from m/s to [-100, 100] using v_max as the reference
    left  = (v_left  / self.v_max) * 100.0
    right = (v_right / self.v_max) * 100.0

    # Clip to valid range
    left  = np.clip(left,  -100, 100)
    right = np.clip(right, -100, 100)

    # Apply dead-zone: wheel must reach |10| to overcome static friction
    DEAD_ZONE = 10
    left  = 0.0 if abs(left)  < DEAD_ZONE else left
    right = 0.0 if abs(right) < DEAD_ZONE else right

    return int(round(left)), int(round(right))

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
