# Arena related objects (baskets, obstacles, our robots, other robots, etc.)

from src.utils import Position
from src.robot import Robot, Team

class Basket:    
    # Pos is a (x, y) float
    def __init__(self, pos: Position, measurement_distance: float, clipped_image: bool):
        self.pos = pos
        self.clipped_image = False # whether or not the basket was fully visible in the image that it was identified in
        self.measurement_distance = measurement_distance # distance from which the basket was identified from (closer = more reliable)

        self.team = None # do we know the team yet? (this is only set once the April Tag is scanned)
        self.scanned = False # whether we have scanned the basket's April Tag yet
        self.sent_april_tag = False # has the April tag been sent yet
        self.item_delivered = False # whether our Gripper robot has dropped an item in the basket


class Arena:
    def __init__(self, robots: list[Robot]):
        self.robots = robots
        # boundary_points refers to points on the boundary of any obstacle, robot, or barrier (in any order)
        self.boundary_points: list[Position]
