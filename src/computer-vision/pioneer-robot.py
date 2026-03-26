from src.arena import Basket
from src.utils import Position
from cv2 import Mat

# Returns a list of tuples containing a basket and the robot's relative angle to it
def find_visible_baskets_in_image(img: Mat) -> list[tuple[Basket, float]]:
    NotImplemented




# TESTING BELOW
# You can run this with:
# py -m src.computer-vision.pioneer-robot

def test_vision():
    print("Testing Computer Vision")


test_vision()