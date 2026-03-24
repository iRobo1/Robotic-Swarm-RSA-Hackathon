from arena import *

# Returns a list of tuples containing a basket and the robot's angle to it
# (Sandro will probably change this function's signature. Just an example for now)
def locate_all_visible_baskets() -> list[tuple[Basket, float]]:
    return [(Basket(Position(10, 10), 60, Team.RED), 50)]

