from arena import *
from cv2 import Mat

# Returns a list of tuples containing a basket and the robot's relative angle to it
def find_visible_baskets_in_image(img: Mat) -> list[tuple[Basket, float]]:
    NotImplemented