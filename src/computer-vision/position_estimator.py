import numpy as np
from typing import Tuple

def estimate_distance_from_img(img_height: int, fov_v: float, y_base: int) -> float:
    """
        Estimate the distance to an object based on its position in the RGB image.

        Parameters:
            - img_height: height of the RGB image in pixels
            - fov_v: vertical field of view of the camera in degrees
            - y_base: the y-coordinate of the base of the object in the image (in pixels)

        Returns:
            - Estimated distance to the object in meters
    """
    dist = 0.0

    theta_depress = fov_v * (y_base - img_height / 2) / img_height
    dist = img_height / np.tan(np.radians(theta_depress))

    return dist

def estimate_position_from_img(robot_pose: Tuple[float, float, float, float], obj_dist: float) -> Tuple[float, float]:
    """
        Estimate the position of an object in the world frame based on the robot's pose and the estimated distance to the object.

        Parameters:
            - robot_pose: a tuple containing the robot's x, y, z coordinates and its orientation (angle in degrees)
            - obj_dist: the estimated distance to the object in meters
        Returns:
            - A tuple containing the estimated x and y coordinates of the object in the world frame
    """
    robot_x, robot_y, robot_z, robot_angle = robot_pose

    # Convert the robot's angle from degrees to radians
    robot_angle_rad = np.radians(robot_angle)

    # Calculate the object's position in the world frame
    obj_x = robot_x + obj_dist * np.cos(robot_angle_rad)
    obj_y = robot_y + obj_dist * np.sin(robot_angle_rad)

    return (obj_x, obj_y)