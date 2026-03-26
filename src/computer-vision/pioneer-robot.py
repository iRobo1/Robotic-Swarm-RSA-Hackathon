from src.arena import Basket
from src.utils import Position
import cv2
import os
import re
import time
import numpy as np

# This computer vision code must run relatively quickly, taking less than 100 ms to process.

# Returns a list of tuples containing a basket and the robot's relative angle to it
def find_visible_baskets_in_image(img: cv2.Mat) -> list[tuple[Basket, float]]:
    NotImplemented


import cv2
import numpy as np

# Computer vision tasks
# 1. Improve the identification of baskets to be almost perfect (e.g., sometimes they are slightly larger/smaller than they should be due to e.g. blur or a red chair in the background)
# 2. Calculate the distance and angle of the basket relative to the robot (*by using the dimensions of the basket and specifications of the camera)
# 3. Calculate the global position of the basket based on the current robot's state
# 4. Identify April Tags on the baskets and their global orientation
# 5. Aggregate basket and April Tag locations from all previous images into a single local (*i.e., per robot) map (sharing this with other robots is a different problem that I won't focus on here)
# 8. Identify all the walls in the image, their normal vectors, and positions relative to the robot
# 9. Identify all robots in the image, their distance and angle relative to us, and their global position (similar to the basket thing).
# 10. Identify all robots' directions in the image (i.e., the direction the robots are travelling in)
# 11. Calculate the position and angle of the robot based on visible baskets (this can be used if the top-down camera doesn't seem our robot or fails); it would be pretty cool too to say that our robots are so good they don't even need the external vision system 😅
# 
# Non-computer vision tasks for later
# 1. Aggregate the local maps (assume you are sent maps from other robots, you need to generate a single global map). This code runs only on the Gripper Robot
# 2. Write an object avoidance algorithm using the computer vision data

# Even more tasks:
# - Identify the fire hydrant in the starting zone (position)
# - Identify whether the gripper is holding the fire hydrant
# - (extra) identify the position of the gripper (so that the robot can autonomously drive up to the fire hydrant and pick it up)
# - (extra) identify the state of the gripper (whether it is open/closed and by how much, this can be used to adjust the state of the gripper)
# - 



# Issues with current basket identification (notes for myself):
# - black lines on the sides aren't actually done by checking the height of the matching pixels on the side
# colours should be better defined (best to actually average the colours manually or plot all the colours inside these blobs and
# somehow decide which to use)
# - the boundaries should be contracted by taking into account the actual average colour of the specific blob and contracting until at least 
# 50% of the pixels on the side are very similar to that average colour. The average should be taken from the ~10 pixels on the left-most or
# right-most side of the blob, perhaps ignoring the actual left-most 3 pixels. Actually, the number of pixels to average and to skip
# should be a percentage of the total width of the blob
# - use a slightly larger cut off for the size of the block (e.g., 30 pixels high)
# - if it is helpful, find the actual blob outline? also find outlines inside the blob that do NOT match the blob colour (or match white/black)
# this should identify the April Tags.
# - calculate orientation of April tag based on how close it is to the edge of the blob (or how much of it is not visible)
# - excluding the April Tag, >95% of the pixels inside the blob boundary should be similar in colour (or actually, due to shade changes, 
# there should be very few gradients; there might be like 1 or 2 gradients due to lighting. This is probably a better test.)
# - check if the anti-blur thing makes any sense

# additional constraints (but test also without these because these might remove some incorrectly identified
# targets, but we should want as few constraints as possible to be as powerful as possible):
# - ignore any blob found right next to another larger blob of the same colour

# more problems:
# still doesn't identify walls!
# nor other robots!

def process_blobs_refined(img):
    # Standard dimensions
    IMG_WIDTH = 640
    IMG_HEIGHT = 480
    TARGET_Y = 200

    # 1. Handle horizontal blur
    kernel_sharpen = np.array([[-1, 5, -1]]) / 3.0
    img_filtered = cv2.filter2D(img, -1, kernel_sharpen)
    
    # 2. Convert to HSV
    hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
    
    # 3. Define Color Ranges and their BGR display colors
    # Format: (name, lower_hsv, upper_hsv, bgr_draw_color)
    color_configs = [
        ("Red/Pink", np.array([0, 50, 50]), np.array([10, 255, 255]), (0, 0, 255)),
        ("Red/Pink", np.array([160, 50, 50]), np.array([180, 255, 255]), (0, 0, 255)),
        ("Green", np.array([35, 40, 40]), np.array([85, 255, 255]), (0, 255, 0)),
        ("Blue", np.array([100, 50, 50]), np.array([140, 255, 255]), (255, 0, 0))
    ]

    # Wood color range (converted roughly to HSV for more robust checking)
    # Based on the RGBs provided, wood is roughly Hue 15-30, Sat 20-50%, Val 60-90%
    lower_wood = np.array([10, 30, 130])
    upper_wood = np.array([30, 150, 255])

    for name, lower, upper, draw_color in color_configs:
        mask = cv2.inRange(hsv, lower, upper)
        
        # Vertical opening to help with sharp top/bottom edges
        kernel = np.ones((5, 1), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            
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

            # --- Wood Color Check ---
            # Look at 10 pixels starting 2 pixels below the bottom edge
            wood_y_start = new_y + new_h + 2
            wood_y_end = wood_y_start + 10
            
            # Ensure we stay within image bounds
            if wood_y_end < IMG_HEIGHT:
                wood_roi_hsv = hsv[wood_y_start:wood_y_end, new_x:new_x + new_w]
                wood_mask = cv2.inRange(wood_roi_hsv, lower_wood, upper_wood)
                
                # Check if average pixel satisfies "wood" (more than 50% of ROI)
                if np.mean(wood_mask) > 127.5:
                    # Draw Bounding Box
                    cv2.rectangle(img, (new_x, new_y), (new_x + new_w, new_y + new_h), draw_color, 1)
                    
                    # Draw Black vertical lines inside the boundary
                    # Left vertical line
                    cv2.line(img, (new_x + 1, new_y), (new_x + 1, new_y + new_h), (0, 0, 0), 1)
                    # Right vertical line
                    cv2.line(img, (new_x + new_w - 1, new_y), (new_x + new_w - 1, new_y + new_h), (0, 0, 0), 1)

    # Final requirement: Add the red horizontal line at height 200
    cv2.line(img, (0, TARGET_Y), (IMG_WIDTH, TARGET_Y), (0, 0, 255), 1)

    return img



def process_blobs(img):
    # 1. Handle horizontal blur by slightly sharpening or using a vertical kernel
    # Since the blur is horizontal, a 3x1 or 5x1 kernel can help define edges
    kernel_sharpen = np.array([[-1, 5, -1]]) / 3.0
    img_filtered = cv2.filter2D(img, -1, kernel_sharpen)
    
    # 2. Convert to HSV for better color segmentation
    hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
    
    # 3. Define Color Ranges (Hue ranges: Green ~35-85, Blue ~100-130, Red ~0-10 & 170-180)
    # Note: These ranges cover Red, Pink, Blue, and Green generally.
    lower_colors = [
        np.array([0, 50, 50]),    # Red/Pink Lower
        np.array([170, 50, 50]),  # Red/Pink Upper
        np.array([35, 40, 40]),   # Green
        np.array([100, 50, 50])   # Blue
    ]
    upper_colors = [
        np.array([10, 255, 255]),
        np.array([180, 255, 255]),
        np.array([85, 255, 255]),
        np.array([130, 255, 255])
    ]

    # Combine masks
    full_mask = np.zeros(img.shape[:2], dtype=np.uint8)
    for l, u in zip(lower_colors, upper_colors):
        mask = cv2.inRange(hsv, l, u)
        full_mask = cv2.bitwise_or(full_mask, mask)

    # 4. Clean up the mask
    # We use a vertical opening to favor sharp top/bottom edges and filter noise
    kernel = np.ones((5, 2), np.uint8)
    full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)

    # 5. Find and Filter Contours
    contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Requirement: at least 25px high and 10px wide
        if h >= 25 and w >= 10:
            # Requirement: at least part of the blob present at height 200
            # (Blob must span across the y=200 line)
            if y <= 200 <= (y + h):
                # Draw the 1-pixel rectangle
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)

    # 6. Add the 1-pixel high red horizontal line at height 200
    # BGR for Red is (0, 0, 255)
    cv2.line(img, (0, 200), (640, 200), (0, 0, 255), 1)

    return img



def annotate_image(img: cv2.Mat) -> cv2.Mat:
    # Text parameters
    text = "Hi World, d=11"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5  # Approx 11px height
    thickness = 1
    color = (0, 0, 0)  # White

    # Get text size to center it perfectly
    # (width, height), baseline
    (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)

    # Calculate coordinates for the middle of a 640x480 image
    # Middle is (320, 240). Offset by half the text size.
    x = (img.shape[1] - text_width) // 2
    y = (img.shape[0] + text_height) // 2

    # Put the text on the image
    img_annotated = cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)
    return img_annotated


def process_image(img: cv2.Mat) -> cv2.Mat:
    #img_annotated = annotate_image(img)

    #processed_img = process_blobs(img)
    processed_img = process_blobs_refined(img)
    return processed_img


# objects of similar colour
# at least 25 pixels high
#


# Testing function. Loads images from data/ and processes them.
# You can run this with:
# py -m src.computer-vision.pioneer-robot
def test_vision():
    print("Testing Computer Vision")

    input_dir = 'data/pioneer_images/raw'
    output_dir = 'data/pioneer_images/annotated/'

    # List all files in the raw directory
    files = [f for f in os.listdir(input_dir) if f.lower().endswith('.png')]

    files.sort(key=lambda f: int(re.sub(r'\D', '', f)))
    
    if not files:
        print("No input images found in", input_dir)
        return
    
    start_time = time.perf_counter()

    for filename in files:
        img_path = os.path.join(input_dir, filename)
        img = cv2.imread(img_path)

        if img is None:
            print(f"Skipping {filename}: could not load image.")
            continue

        processed_image = process_image(img)

        # Save to the annotated folder
        save_path = os.path.join(output_dir, filename)
        cv2.imwrite(save_path, processed_image)
        print(f"Processed and saved: {filename}")

    end_time = time.perf_counter()

    execution_time = end_time - start_time
    print(f"Took {1000 * execution_time / len(files):.1f} ms per image, or {len(files) / execution_time:.1f} images/second")


test_vision()