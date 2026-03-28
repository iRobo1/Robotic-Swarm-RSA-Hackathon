import numpy as np
from typing import Tuple

def estimate_obj_angle_in_img(img_width: int, fov_h: float, x_center: int, robot_heading: float) -> float:
    """
        Estimate angle of the object in the image.

        Parameters:
            - img_width (int): width of the image
            - fov_h (float): horizontal fov of the camera
            - x_center (int): center pixel coordinate of the object in the image.

        Returns:
            - angle (float): angle of the object in the image.
    """
    angle = 0.0

    h_pix_ang_sep = fov_h / img_width
    x_from_right = img_width - x_center
    phi_rel = x_from_right * h_pix_ang_sep

    offset_ang = robot_heading - fov_h / 2
    phi_abs = phi_rel - offset_ang

    return phi_abs

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

def estimate_position_from_img(robot_pos: Tuple[float, float], obj_dist: float, obj_angle: float) -> Tuple[float, float]:
    """
        Estimate the position of an object in the world frame based on the robot's pose and the estimated distance to the object.

        Parameters:
            - robot_pos: a tuple containing the robot's x, y coordinates in world frame
            - obj_dist: the estimated distance to the object in meters
            - obj_angle: angle of the object in world frame
        Returns:
            - A tuple containing the estimated x and y coordinates of the object in the world frame
    """
    robot_x, robot_y = robot_pos

    # Calculate the object's position in the world frame
    obj_x = robot_x + obj_dist * np.cos(obj_angle)
    obj_y = robot_y + obj_dist * np.sin(obj_angle)

    return (obj_x, obj_y)