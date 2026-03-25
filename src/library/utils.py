import re
import socket
import numpy as np
import math

MIN_EDGE_SIZE = 95

def get_team_robot_id():
    """
    Retrieve the team and robot ID.

    Example:
        team_id, robot_id = utils.get_team_robot_id()

    Returns:
        tuple[int, int]: (team_id, robot_id)
    """
    hostname = socket.gethostname()

    match = re.match(r"mirte-(\d+)-(\d+)", hostname)
    if match:
        team = int(match.group(1))
        robot = int(match.group(2))
        return (team, robot)
    else:
        raise ValueError("Hostname does not match expected format")

def get_password():
    """
    Retrieve the team password from the robot (that you need for authenticating requests).

    Example:
        password = utils.get_password()

    Returns:
        str: Password string
    """
    return open("/home/mirte/.wifi_pwd", "r").readline()

def angle_difference(desired, current):
    """
    Compute the difference between two angles in radians, within the range [-pi, pi]

    Example:
        diff = utils.angle_difference(math.pi/2, math.pi/4)
        # desired (float): Target angle in radians
        # current (float): Current angle in radians

    Returns:
        float: Difference between desired and current angle in radians
    """
    difference = desired - current
    difference = (difference + math.pi) % (2 * math.pi) - math.pi
    return difference

def clamp(value, low, high):
    """
    Clamp a value within a specified range.

    Example:
        result = utils.clamp(10, 0, 5)
        # value (float): Input value to clamp
        # low (float): Minimum allowed value
        # high (float): Maximum allowed value

    Returns:
        float: Clamped value within [low, high]
    """
    return max(low, min(high, value))

def is_tag_within_distance(tag):
    """
    Determine if a tag is within 50 cm of the robot (max distance at which you can send an objective message).
    Uses the max edge size, not always accurate but fast to compute

    Example:
        tags = detector.detect_objective_tags()
        for tag in tags:
            print(utils.is_tag_within_distance(tag))

    Returns:
        bool: if the tag is within the max distance
    """
    corners = np.array(tag.corners)
    edges = [
        np.linalg.norm(corners[0] - corners[1]),
        np.linalg.norm(corners[1] - corners[2]),
        np.linalg.norm(corners[2] - corners[3]),
        np.linalg.norm(corners[3] - corners[0])
    ]
    return np.max(edges) > MIN_EDGE_SIZE

def get_team_from_tag_id(tag_id):
    """
    Determine the team ID from an AprilTag ID.

    Example:
        team = get_team_from_tag_id(23)
        # tag_id (int): ID of the AprilTag
        # team (int | None): Team number corresponding to tag_id or None if invalid

    Args:
        tag_id (int): AprilTag ID

    Returns:
        int | None: Team number (tag_id // 10) if valid, otherwise None
    """
    if tag_id < 10 or tag_id > 59:
        return None
    return tag_id // 10