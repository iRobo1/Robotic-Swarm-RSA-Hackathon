# Arena related objects (baskets, obstacles, our robots, other robots, etc.)

from src.utils import Position
from src.robot import Robot, Team

class Basket:    
    # Pos is a (x, y) float
    def __init__(self, pos: Position, team: Team, measurement_distance: float):
        self.pos = pos
        self.team = team
        self.scanned = False # whether any team has scanned the basket's April Tag yet
        self.item_delivered = False # whether our Gripper robot has dropped an item in the basket
        self.measurement_distance = measurement_distance # distance from which the basket was identified from (closer = more reliable)


class Arena:
    def __init__(self, robots: list[Robot]):
        self.robots = robots
        # boundary_points refers to points on the boundary of any obstacle, robot, or barrier (in any order)
        self.boundary_points: list[Position]
