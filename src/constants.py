# A file to hold magic numbers :)

from utils import Position

####################################################
############## SHARED ROBOT CONSTANTS ##############
####################################################

# Maximum robot speed in m/s
MAXIMUM_SPEED_MPS = 0.4


#####################################################
############## GRIPPER ROBOT CONSTANTS ##############
#####################################################

# Position of the sensors relative to the centre of the robot in millimetres
# Position(x, y) = where x is left(-)/right(+) and y is forwards (+)/backwards(-)
GRIPPER_REL_POS_LEFT_ULTRASONIC_MM = Position(-12.3, 12.3)
GRIPPER_REL_POS_RIGHT_ULTRASONIC_MM = Position(12.3, 12.3)
GRIPPER_REL_POS_LIDAR_MM = Position(0, 12.3)
GRIPPER_REL_POS_CAMERA_MM = Position(0, 12.3)

# Angle of the left/right ultrasonic sensor relative to the direction the robot is facing in degrees
# left(-), right(+)
GRIPPER_REL_ANGLE_LEFT_ULTRASONIC_DEG = -12.345
GRIPPER_REL_ANGLE_RIGHT_ULTRASONIC_DEG = 12.345


#####################################################
############## PIONEER ROBOT CONSTANTS ##############
#####################################################

# Position of the sensors relative to the centre of the robot in millimetres
# Position(x, y) = where x is left(-)/right(+) and y is forwards (+)/backwards(-)
PIONEER_REL_POS_LEFT_ULTRASONIC_MM = Position(-12.3, 12.3)
PIONEER_REL_POS_RIGHT_ULTRASONIC_MM = Position(12.3, 12.3)
PIONEER_REL_POS_CAMERA_MM = Position(0, 12.3)

# Angle of the left/right ultrasonic sensor relative to the direction the robot is facing in degrees
# left(-), right(+)
PIONEER_REL_ANGLE_LEFT_ULTRASONIC_DEG = -12.345
PIONEER_REL_ANGLE_RIGHT_ULTRASONIC_DEG = 12.345