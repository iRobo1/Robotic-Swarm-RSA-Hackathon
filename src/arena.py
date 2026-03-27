# Arena related objects (baskets, obstacles, our robots, other robots, etc.)

from src.utils import Position
from src.robot import Robot, Team


class Basket:
    
    def __init__(self, pos: Position, angle: float, team: Team):
        self.pos = pos
        self.angle = angle # angle denotes the direction the April Tag is facing
        self.team = team
        self.discovered = False # whether any team has scanned the basket's April Tag yet
        self.completed = False # whether our Gripper robot has dropped an item in the basket


# A single obstacle (a connected component of points in CW order forming one obstacle)
class Obstacle:
    def __init__(self, boundary: list[Position]):
        self.boundary = boundary


class Arena:
    def __init__(self, robots: list[Robot]):
        self.robots = robots
        # boundary_points refers to points on the boundary of any obstacle, robot, or barrier (in any order)
        self.boundary_points: list[Position]
