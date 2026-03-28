from src.arena import Basket
from src.utils import Position
import cv2 as cv
import os
import re
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import numpy as np
from src.robot import Team


team_mapping = {
    "red": Team.RED,
    "yellow": Team.YELLOW,
    "blue": Team.BLUE,
    "green": Team.GREEN,
    "pink": Team.PINK,
}

color_configs = [
    {
        "name": "yellow",
        "draw_color": (255, 255, 0),
        "ranges": [
            (np.array([26, 128, 244]), np.array([29, 146, 255])),
            (np.array([21, 139, 194]), np.array([25, 158, 215])),
            (np.array([26, 99, 241]), np.array([28, 129, 255])),
            (np.array([23, 135, 216]), np.array([27, 155, 235])),
            (np.array([25, 127, 230]), np.array([28, 150, 246]))
        ]
    },
    {
        "name": "blue",
        "draw_color": (255, 0, 0),
        "ranges": [
            (np.array([108, 53, 128]), np.array([119, 105, 184])),
            (np.array([107, 47, 183]), np.array([115, 112, 243])),
            (np.array([106, 21, 111]), np.array([135, 78, 161])),
            (np.array([0, 15, 144]), np.array([11, 38, 163])),
            (np.array([111, 9, 135]), np.array([178, 54, 190]))
        ]
    },
    {
        "name": "red",
        "draw_color": (0, 0, 255),
        "ranges": [
            (np.array([0, 122, 150]), np.array([14, 167, 189])),
            (np.array([177, 128, 180]), np.array([179, 155, 253])),
            (np.array([0, 84, 216]), np.array([11, 152, 255])),
            (np.array([0, 135, 188]), np.array([11, 165, 225])),
            (np.array([0, 104, 175]), np.array([14, 141, 217]))
        ]
    },
    {
        "name": "green",
        "draw_color": (0, 255, 0),
        "ranges": [
            (np.array([34, 70, 114]), np.array([59, 102, 157])),
            (np.array([40, 46, 93]), np.array([61, 91, 131])),
            (np.array([32, 36, 178]), np.array([55, 90, 209])),
            (np.array([32, 48, 127]), np.array([59, 84, 158])),
            (np.array([38, 35, 153]), np.array([61, 94, 180]))
        ]
    },
    {
        "name": "pink",
        "draw_color": (147, 20, 255),
        "ranges": [
            (np.array([170, 89, 151]), np.array([179, 132, 196])),
            (np.array([0, 74, 144]), np.array([12, 120, 222])),
            (np.array([168, 93, 216]), np.array([179, 142, 246])),
            (np.array([171, 78, 182]), np.array([179, 111, 234])),
            (np.array([169, 111, 189]), np.array([178, 144, 223]))
        ]
    },
    {
        "name": "floor",
        "draw_color": (200, 200, 200),
        "ranges": [
            (np.array([9, 66, 174]), np.array([31, 103, 217])),
            (np.array([11, 16, 219]), np.array([35, 73, 255])),
            (np.array([13, 20, 184]), np.array([32, 71, 223])),
            (np.array([12, 35, 144]), np.array([31, 109, 191])),
            (np.array([7, 60, 211]), np.array([28, 101, 251]))
        ]
    },
    {
        "name": "white", # LLM generated values, need to be calibrated with April Tags
        "draw_color": (255, 255, 255),
        "ranges": [
            # Pure white to slightly off-white (Low Saturation, High Value)
            (np.array([0, 0, 200]), np.array([180, 55, 255])),
            # Slightly cooler/bluer white
            (np.array([90, 0, 220]), np.array([130, 30, 255])),
            # Slightly warmer/yellowish white
            (np.array([20, 0, 220]), np.array([40, 30, 255]))
        ]
    },
    {
        "name": "black", # LLM generated values, need to be calibrated with April Tags
        "draw_color": (0, 0, 0),
        "ranges": [
            # Pure black to dark charcoal (Very low Value)
            (np.array([0, 0, 0]), np.array([180, 255, 50])),
            # Deep shadows with slight blue tint
            (np.array([100, 20, 0]), np.array([140, 255, 40])),
            # Deep shadows with slight red/brown tint
            (np.array([0, 20, 0]), np.array([20, 255, 40]))
        ]
    }
]



# Returns a list of tuples containing a basket and the robot's relative angle to it
def find_visible_baskets_in_image(img: cv.Mat) -> list[tuple[Basket, float]]:
    NotImplemented


SAVE_TEST_IMAGES = False
DRAW_DEBUG = False
# Save image to data/test_outputs/
def save_test_image(img: cv.Mat, filename: str):
    if SAVE_TEST_IMAGES:
        output_dir = 'data/test_outputs/'
        save_path = os.path.join(output_dir, filename)
        cv.imwrite(save_path, img)


# Annotates an image by adding text in the top left on the given row
def annotate_image(img: cv.Mat, text: str, row: int) -> cv.Mat:
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5 
    thickness = 1
    color = (0, 0, 0)

    x = 10
    line_height = 20 
    y = 20 + (row * line_height)

    img_annotated = cv.putText(img, text, (x, y), font, font_scale, color, thickness, cv.LINE_AA)
    
    return img_annotated


# larger number = sharper image. However, boring scenes will get low numbers, even if they are sharp. So use with care
# A number above ~80 is certainly a relatively sharp image. But a low number (e.g., 20) does not necessarily indicate a blurry image.
def calculate_blur_coefficient(img: cv.Mat) -> float:
    # load the image, convert it to grayscale, and compute the
    # focus measure of the image using the Variance of Laplacian method
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    fm = cv.Laplacian(gray, cv.CV_64F).var()
    return fm


def process_bounding_boxes(bounding_boxes):
    # Helper to calculate intersection area
    def get_intersection_area(box1, box2):
        x1, y1, dx1, dy1 = box1[:4]
        x2, y2, dx2, dy2 = box2[:4]
        
        ix = max(x1, x2)
        iy = max(y1, y2)
        iw = min(x1 + dx1, x2 + dx2) - ix
        ih = min(y1 + dy1, y2 + dy2) - iy
        
        if iw > 0 and ih > 0:
            return iw * ih
        return 0

    # --- Step 1: Overlap Resolution ---
    remaining = sorted(bounding_boxes, key=lambda b: b[2] * b[3], reverse=True)
    stage1_results = []

    while remaining:
        # Take the largest box in the current set
        largest = remaining.pop(0)
        lx, ly, ldx, ldy = largest[:4]
        l_area = ldx * ldy
        
        keep_from_remaining = []
        
        for box in remaining:
            bx, by, bdx, bdy = box[:4]
            b_area = bdx * bdy
            inter_area = get_intersection_area(largest, box)
            
            if inter_area == 0:
                keep_from_remaining.append(box)
                continue
                
            overlap_pct = inter_area / b_area
            
            if overlap_pct > 0.20:
                # Remove: do not add to keep_from_remaining
                continue
            else:
                # Decrease the box by the overlap amount
                # The prompt implies shrinking the box's dimensions or area. 
                # Common interpretation: reduce dimensions by the intersecting bounds.
                new_bx = bx if bx >= lx + ldx or bx + bdx <= lx else max(bx, lx + ldx)
                new_by = by if by >= ly + ldy or by + bdy <= ly else max(by, ly + ldy)
                # Note: Adjusting x/y and dx/dy based on specific intersection side
                # For simplicity, we ensure dimensions are recalculated:
                new_bdx = (bx + bdx) - new_bx
                new_bdy = (by + bdy) - new_by
                
                if new_bdy >= 25 and new_bdx >= 10:
                    keep_from_remaining.append((new_bx, new_by, new_bdx, new_bdy, box[4]))
        
        stage1_results.append(largest)
        remaining = sorted(keep_from_remaining, key=lambda b: b[2] * b[3], reverse=True)

    # --- Step 2: Vertical Containment Filtering ---
    # "If [x2, x2+dx2] fully resides inside [x, x+dx] and y2 > y, remove second box"
    final_boxes = []
    to_remove = set()
    
    for i in range(len(stage1_results)):
        for j in range(len(stage1_results)):
            if i == j: continue
            
            x1, y1, dx1, dy1, _ = stage1_results[i]
            x2, y2, dx2, dy2, _ = stage1_results[j]
            
            # Check if box2 is below box1 and horizontally contained
            if y2 > y1 and x2 >= x1 and (x2 + dx2) <= (x1 + dx1):
                to_remove.add(j)
                
    for idx, box in enumerate(stage1_results):
        if idx not in to_remove:
            final_boxes.append(box)
            
    return final_boxes



def process_blobs_refined(img: cv.Mat) -> tuple[cv.Mat, list[tuple[int, int, Team]]]:
    IMG_WIDTH = 640
    IMG_HEIGHT = 480
    TARGET_Y = 200

    #blur = calculate_blur_coefficient(img)

    save_test_image(img, "1original_image.png")

    # 2. Convert to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    color_masks = {}

    # Vertical opening to help with sharp top/bottom edges (except black and white)
    kernel = np.ones((5, 1), np.uint8)

    for config in color_configs:
        name = config["name"]
        
        # Initialize a blank mask for the current color
        # hsv should be your pre-processed HSV image
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        # Combine all defined ranges for this specific color
        for (lower, upper) in config["ranges"]:
            range_mask = cv.inRange(hsv, lower, upper)
            combined_mask = cv.bitwise_or(combined_mask, range_mask)
        
        # Apply morphologyEx ONLY if the color is NOT black or white
        if name.lower() not in ["white", "black"]:
            combined_mask = cv.morphologyEx(combined_mask, cv.MORPH_OPEN, kernel)
    
        # Map the color name to the computed mask in our dictionary
        color_masks[name] = combined_mask

        save_test_image(combined_mask, f"4mask_{name.lower()}.png")

    bounding_boxes = []
    basket_colors = ["red", "blue", "green", "yellow", "pink"]
    for basket_color in basket_colors:
        debug_img = img.copy()
        mask = color_masks[basket_color]
        draw_color = next(c["draw_color"] for c in color_configs if c["name"] == basket_color)

        mask_combined = mask # white and black masks are very wrong!
        #mask_combined = cv.bitwise_or(mask, color_masks["black"]) # April tag counts as matching
        #mask_combined = cv.bitwise_or(mask_combined, color_masks["white"]) # April tag counts as matching
        contours, _ = cv.findContours(mask_combined, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # SAVE: Visualization of contours for this color
        contour_img = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        if DRAW_DEBUG: cv.drawContours(contour_img, contours, -1, draw_color, 2)
        save_test_image(contour_img, f"6contours_{basket_color.lower()}.png")

        for cnt in contours:
            x, y, w, h = cv.boundingRect(cnt)

            if DRAW_DEBUG: cv.rectangle(debug_img, (x, y), (x + w, y + h), draw_color, 1)
            save_test_image(debug_img, f"7before_contraction_{basket_color.lower()}.png")
            
            # Initial filter: must cross Y=200
            if not (y <= TARGET_Y <= (y + h)):
                continue

            # --- Contraction Logic ---
            roi_mask = mask[y:y+h, x:x+w]
            
            # Contract Top
            y_start = 0
            while y_start < h and np.mean(roi_mask[y_start, :]) < 127.5: # 127.5 is 50% of 255
                y_start += 1
            
            # Contract Bottom
            y_end = h - 1
            while y_end > y_start and np.mean(roi_mask[y_end, :]) < 127.5:
                y_end -= 1
                
            # Contract Left
            x_start = 0
            while x_start < w and np.mean(roi_mask[:, x_start]) < 127.5:
                x_start += 1
                
            # Contract Right
            x_end = w - 1
            while x_end > x_start and np.mean(roi_mask[:, x_end]) < 127.5:
                x_end -= 1

            # New Dimensions
            new_x, new_y = x + x_start, y + y_start
            new_w, new_h = (x_end - x_start) + 1, (y_end - y_start) + 1

            # Check size requirements after contraction
            if new_h < 25 or new_w < 10:
                continue

            if DRAW_DEBUG: cv.rectangle(debug_img, (new_x, new_y), (new_x + new_w, new_y + new_h), draw_color, 1)
            save_test_image(debug_img, f"7before_floor_check_{basket_color.lower()}.png")

            # --- Wood (Floor) Color Check ---
            # Instead of cv.inRange, we slice the pre-calculated "floor" mask
            wood_y_start = new_y + new_h + 2 # +2
            wood_y_end = wood_y_start + 20

            #print("running wood check")
            
            if wood_y_end < IMG_HEIGHT:
                floor_mask = color_masks.get("floor")

                # Slice the floor mask at the target ROI
                wood_roi_mask = floor_mask[wood_y_start:wood_y_end, new_x:new_x + new_w]

                #print("checking ROI:", np.mean(wood_roi_mask))
                
                # Check if average pixel satisfies "floor/wood" (more than 50% of ROI)
                if np.mean(wood_roi_mask) > 127.5:
                    #print("drawing bounding box")
                    # Draw Bounding Box
                    #cv.rectangle(img, (new_x, new_y), (new_x + new_w, new_y + new_h), draw_color, 1)
                    bounding_boxes.append((new_x, new_y, new_w, new_h, basket_color))

                    #basket_height = new_h
                    #basket_position_x = new_x + new_w//2

                    #basket = tuple(basket_position_x, basket_height, team=team_mapping[basket_color])
                    #baskets.append(basket)
                    
                    # Draw Black vertical lines inside the boundary
                    #cv.line(img, (new_x + 1, new_y), (new_x + 1, new_y + new_h), (0, 0, 0), 1)
                    #cv.line(img, (new_x + new_w - 1, new_y), (new_x + new_w - 1, new_y + new_h), (0, 0, 0), 1)


    #filtered_bounding_boxes = bounding_boxes
    filtered_bounding_boxes = process_bounding_boxes(bounding_boxes)

    baskets = []

    for bounding_box in filtered_bounding_boxes:
        x, y, dx, dy, basket_color = bounding_box
        draw_color = next(c["draw_color"] for c in color_configs if c["name"] == basket_color)
        cv.rectangle(img, (x, y), (x + dx, y + dy), draw_color, 1)

        basket_y = y
        basket_position_x = x + dx//2

        baskets.append((basket_y, basket_position_x))


    # Final requirement: Add the red horizontal line at height 200
    cv.line(img, (0, TARGET_Y), (IMG_WIDTH, TARGET_Y), (0, 0, 255), 1)

    save_test_image(img, f"8final_image.png")

    return (img, baskets)


def process_image(img: cv.Mat) -> cv.Mat:
    processed_img, _ = process_blobs_refined(img)
    return processed_img


# Testing function. Loads images from data/ and processes them.
# You can run this with:
# py -m src.computer-vision.pioneer-robot
def test_vision_all_images(image_number: int = None):
    print("Testing Computer Vision")

    input_dir = 'data/pioneer_images/raw'
    output_dir = 'data/pioneer_images/annotated/'
    #input_dir = 'data/gripper_images/raw'
    #output_dir = 'data/gripper_images/annotated/'

    # List all files in the raw directory
    files = [f for f in os.listdir(input_dir) if f.lower().endswith('.png')]

    files.sort(key=lambda f: int(re.sub(r'\D', '', f)))
    
    if not files:
        print("No input images found in", input_dir)
        return
    
    if image_number != None:
        files = [files[image_number - 1]]
    
    start_time = time.perf_counter()

    for filename in files:
        img_path = os.path.join(input_dir, filename)
        img = cv.imread(img_path)

        if img is None:
            print(f"Skipping {filename}: could not load image.")
            continue

        processed_image = process_image(img)

        # Save to the annotated folder
        save_path = os.path.join(output_dir, filename)
        cv.imwrite(save_path, processed_image)
        print(f"Processed and saved: {filename}")

    end_time = time.perf_counter()

    execution_time = end_time - start_time
    print(f"Took {1000 * execution_time / len(files):.1f} ms per image, or {len(files) / execution_time:.1f} images/second")


test_vision_all_images()
#test_vision_all_images(image_number=43)
#test_vision_all_images(image_number=138)

