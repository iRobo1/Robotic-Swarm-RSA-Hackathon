# Arena related objects (baskets, obstacles, our robots, other robots, etc.)

from enum import Enum

class Team(Enum):
    PINK = 1
    GREEN = 2
    YELLOW = 3
    BLUE = 4
    RED = 5 # Our team


class Position:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y


class Robot:
    def __init__(self, pos: Position, angle: float):
        self.pos = pos
        self.angle = angle


class PioneerRobot(Robot):
    pass


class MainRobot(Robot):
    pass


class EnemyRobot(Robot):
    pass


class Basket:
    # angle denotes the direction the April Tag is facing
    def __init__(self, pos: Position, angle: float, team: Team):
        self.pos = pos
        self.angle = angle
        self.team = team
        self.completed = False


# A single obstacle (a connected component of points forming one obstacle)
class Obstacle:
    def __init__(self, boundary: list[Position]):
        self.boundary = boundary


class Arena:
    def __init__(self, robots: list[Robot], obstacle_boundaries: list[Position]):
        self.robots = robots
        # obstacle_boundaries refers to points on the boundary of any obstacle (in any order)
        self.obstacle_boundaries = obstacle_boundaries
