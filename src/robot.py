# Objects related to Robots

from enum import Enum
from src.utils import Position

class Team(Enum):
    PINK = 1
    GREEN = 2
    YELLOW = 3
    BLUE = 4
    RED = 5 # Our team


class Robot:
    def __init__(self, pos: Position, angle: float, team: Team):
        self.pos = pos
        self.angle = angle
        self.team = team


class PioneerRobot(Robot):
    def __init__(self, pos: Position, angle: float, team: Team):
        super().__init__(self, pos, angle, team)


class GripperRobot(Robot):
    def __init__(self, pos: Position, angle: float, team: Team):
        super().__init__(self, pos, angle, team)
        
        self.is_carrying_item = False



class EnemyRobot(Robot):
    def __init__(self, pos: Position, angle: float, team: Team):
        super().__init__(self, pos, angle, team)