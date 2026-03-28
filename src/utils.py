# Utility classes and functions

class Position:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

class Quadrant:
    def __init__(self, lower_corner: Position, upper_corner: Position):
        self.lower = lower_corner
        self.upper = upper_corner
