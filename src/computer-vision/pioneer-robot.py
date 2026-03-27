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

# This computer vision code must run relatively quickly, taking less than 100 ms to process.

# Returns a list of tuples containing a basket and the robot's relative angle to it
def find_visible_baskets_in_image(img: cv.Mat) -> list[tuple[Basket, float]]:
    NotImplemented

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

# instead of exclusive masks that don't overlap, let them overlap (e.g., pink and red). Then pick the colour based on the median/average pixel colour.

# using bilateral filter size 5 might work to make the colour inside the blobs uniform-ish, but it might be slow. Perhaps just Gaussian blur
# the image to reduce noise? Perhaps noise doesn't need to be removed. Try ignoring it.
# Find the average HSV of various regions in the image and better pick the HSV ranges.

def save_test_image(img: cv.Mat, filename: str):
    # Save to the test folder
    output_dir = 'data/test_outputs/'
    save_path = os.path.join(output_dir, filename)
    cv.imwrite(save_path, img)


# Annotates an image by adding text in the top left on the given row
def annotate_image(img: cv.Mat, text: str, row: int) -> cv.Mat:
    font = cv.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5 
    thickness = 1
    color = (0, 0, 0)

    # Define the starting position (Top Left)
    x = 10  # Horizontal padding from the left edge
    
    # Calculate vertical position:
    # We start at 20px for row 0, then add 20px for each subsequent row
    line_height = 20 
    y = 20 + (row * line_height)

    # Put the text on the image
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



def plot_hsv_space(hsv_image):
    """
    Takes an HSV image and plots its pixels as 3D coordinates.
    """
    # 1. Downsample for performance (optional but recommended)
    # Reducing the image size makes the 3D plot much more responsive.
    #scale_percent = 0.1  # Use 10% of the original pixels
    #width = int(hsv_image.shape[1] * scale_percent)
    #height = int(hsv_image.shape[0] * scale_percent)
    #small_hsv = cv.resize(hsv_image, (width, height), interpolation=cv.INTER_AREA)
    small_hsv = hsv_image

    # 2. Reshape the image to a list of pixels
    # Each pixel becomes a row with three columns: [H, S, V]
    pixel_colors = small_hsv.reshape((-1, 3))

    # 3. Prepare coordinates
    # In OpenCV HSV: H is 0-179, S is 0-255, V is 0-255
    h = pixel_colors[:, 0]
    s = pixel_colors[:, 1]
    v = pixel_colors[:, 2]

    # 4. Prepare actual RGB colors for the plot points
    # We want the color of the dot to match the actual color of the pixel
    rgb_image = cv.cvtColor(small_hsv, cv.COLOR_HSV2RGB)
    rgb_colors = rgb_image.reshape((-1, 3)) / 255.0  # Normalize to [0, 1] for Matplotlib

    # 5. Create the 3D Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(h, s, v, c=rgb_colors, marker='o', s=1, alpha=0.5)

    # Labeling the axes
    ax.set_xlabel('Hue (0-179)')
    ax.set_ylabel('Saturation (0-255)')
    ax.set_zlabel('Value (0-255)')
    ax.set_title('Image Pixels in HSV Space')

    plt.show()




def process_blobs_refined(img):
    # Standard dimensions
    IMG_WIDTH = 640
    IMG_HEIGHT = 480
    TARGET_Y = 200

    save_test_image(img, "1original_image.png")

    blur = calculate_blur_coefficient(img)

    img = annotate_image(img, "hi", 0)
    img = annotate_image(img, "world", 1)
    img = annotate_image(img, f"blur: {blur}", 2)

    save_test_image(img, "1annotated_image.png")

    print("blur value:", blur)


    # 1. Handle horizontal blur (does like nothing)
    #kernel_sharpen = np.array([[-1, 5, -1]]) / 3.0
    #img_filtered = cv.filter2D(img, -1, kernel_sharpen)
    # SAVE: Filtered Image
    #save_test_image(img_filtered, "2filtered_image.png")
    
    # 2. Convert to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # SAVE: HSV Image (Note: looks weird in BGR space, but useful for debugging)
    save_test_image(hsv, "3hsv_image.png")
    
    # 3. Define Color Ranges and their BGR display colors
    # Format: (name, lower_hsv, upper_hsv, bgr_draw_color)
    # 3. Define Color Configurations
    # Metadata is stored once, with one or more HSV ranges per color
    color_configs = [
        {
            "name": "Pink",
            "draw_color": (147, 20, 255),
            "ranges": [
                (np.array([160, 20, 50]), np.array([170, 170, 255])),
                (np.array([160, 20, 50]), np.array([180, 90, 255]))
            ]
        },
        {
            "name": "Red",
            "draw_color": (0, 0, 255),
            "ranges": [
                (np.array([0, 50, 50]), np.array([10, 255, 255])),
                (np.array([170, 90, 50]), np.array([180, 255, 255]))
            ]
        },
        {
            "name": "Green",
            "draw_color": (0, 255, 0),
            "ranges": [(np.array([35, 40, 40]), np.array([85, 255, 255]))]
        },
        {
            "name": "Blue",
            "draw_color": (255, 0, 0),
            "ranges": [(np.array([100, 50, 50]), np.array([140, 255, 255]))]
        },
        {
            "name": "Yellow",
            "draw_color": (255, 255, 0),
            "ranges": [(np.array([20, 100, 100]), np.array([30, 255, 255]))] # Fixed overlapping Blue values
        }
    ]

    # Wood color range (converted roughly to HSV for more robust checking)
    # Based on the RGBs provided, wood is roughly Hue 15-30, Sat 20-50%, Val 60-90%
    lower_wood = np.array([10, 30, 130])
    upper_wood = np.array([30, 150, 255])

    for config in color_configs:
        name = config["name"]
        draw_color = config["draw_color"]
        
        # Initialize an empty mask for the current color
        # hsv.shape[:2] gives us (height, width)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        for (lower, upper) in config["ranges"]:
            range_mask = cv.inRange(hsv, lower, upper)
            # Combine ranges using bitwise OR
            mask = cv.bitwise_or(mask, range_mask)
        
        # SAVE: The combined mask for this specific color
        save_test_image(mask, f"4mask_{name.lower()}.png")

        # Vertical opening to help with sharp top/bottom edges
        kernel = np.ones((5, 1), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        # SAVE: The combined mask for this specific color
        save_test_image(mask, f"5mask_with_morph_{name.lower()}.png")

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # SAVE: Visualization of contours for this color
        contour_img = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        cv.drawContours(contour_img, contours, -1, draw_color, 2)
        save_test_image(contour_img, f"6contours_{name.lower()}.png")

        for cnt in contours:
            x, y, w, h = cv.boundingRect(cnt)

            cv.rectangle(img, (x, y), (x + w, y + h), draw_color, 1)
            
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
                wood_mask = cv.inRange(wood_roi_hsv, lower_wood, upper_wood)
                save_test_image(wood_mask, f"7mask_wood_{name.lower()}.png")
                
                # Check if average pixel satisfies "wood" (more than 50% of ROI)
                if np.mean(wood_mask) > 127.5:
                    # Draw Bounding Box
                    cv.rectangle(img, (new_x, new_y), (new_x + new_w, new_y + new_h), draw_color, 1)
                    
                    # Draw Black vertical lines inside the boundary
                    # Left vertical line
                    cv.line(img, (new_x + 1, new_y), (new_x + 1, new_y + new_h), (0, 0, 0), 1)
                    # Right vertical line
                    cv.line(img, (new_x + new_w - 1, new_y), (new_x + new_w - 1, new_y + new_h), (0, 0, 0), 1)

    # Final requirement: Add the red horizontal line at height 200
    cv.line(img, (0, TARGET_Y), (IMG_WIDTH, TARGET_Y), (0, 0, 255), 1)

    save_test_image(img, f"8final_image_{name.lower()}.png")

    return img



def process_blobs(img):
    # 1. Handle horizontal blur by slightly sharpening or using a vertical kernel
    # Since the blur is horizontal, a 3x1 or 5x1 kernel can help define edges
    kernel_sharpen = np.array([[-1, 5, -1]]) / 3.0
    img_filtered = cv.filter2D(img, -1, kernel_sharpen)
    
    # 2. Convert to HSV for better color segmentation
    hsv = cv.cvtColor(img_filtered, cv.COLOR_BGR2HSV)
    
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
        mask = cv.inRange(hsv, l, u)
        full_mask = cv.bitwise_or(full_mask, mask)

    # 4. Clean up the mask
    # We use a vertical opening to favor sharp top/bottom edges and filter noise
    kernel = np.ones((5, 2), np.uint8)
    full_mask = cv.morphologyEx(full_mask, cv.MORPH_OPEN, kernel)

    # 5. Find and Filter Contours
    contours, _ = cv.findContours(full_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        
        # Requirement: at least 25px high and 10px wide
        if h >= 25 and w >= 10:
            # Requirement: at least part of the blob present at height 200
            # (Blob must span across the y=200 line)
            if y <= 200 <= (y + h):
                # Draw the 1-pixel rectangle
                cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)

    # 6. Add the 1-pixel high red horizontal line at height 200
    # BGR for Red is (0, 0, 255)
    cv.line(img, (0, 200), (640, 200), (0, 0, 255), 1)

    return img




def sample_hsv_values(image_number: int = None):
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

    all_sampled_hsv = []

    color_configs = [
        {
            "name": "Pink", # done
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
            "name": "Red", # done
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
            "name": "Green", # done
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
            "name": "Blue", # done
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
            "name": "Yellow", # done (but with only one photo!)
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
            "name": "floor", # done
            "draw_color": (255, 255, 0),
            "ranges": [
                (np.array([9, 66, 174]), np.array([31, 103, 217])),
                (np.array([11, 16, 219]), np.array([35, 73, 255])),
                (np.array([13, 20, 184]), np.array([32, 71, 223])),
                (np.array([12, 35, 144]), np.array([31, 109, 191])),
                (np.array([7, 60, 211]), np.array([28, 101, 251]))
            ]
        }
    ]
    
    # Identify which config is the 'floor' for special logic
    floor_cfg = next(c for c in color_configs if c['name'] == 'floor')
    other_cfgs = [c for c in color_configs if c['name'] != 'floor']

    for filename in files:
        img_path = os.path.join(input_dir, filename)
        img = cv.imread(img_path)
        if img is None: continue
        
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]

        # 1. Create Masks
        def get_mask(config):
            mask = np.zeros((height, width), dtype=np.uint8)
            for (lower, upper) in config["ranges"]:
                range_mask = cv.inRange(hsv, lower, upper)
                mask = cv.bitwise_or(mask, range_mask)
            return mask

        floor_mask = get_mask(floor_cfg) > 0
        
        # Track which color mask matches for condition 4
        color_mask_ids = np.zeros((height, width), dtype=int) # 0 means none
        any_color_mask = np.zeros((height, width), dtype=bool)
        
        for i, cfg in enumerate(other_cfgs, 1):
            m = get_mask(cfg) > 0
            color_mask_ids[m] = i
            any_color_mask = np.logical_or(any_color_mask, m)

        # 2. Pre-calculate "7 out of 10 floor pixels" using convolution
        # We look at y+1 to y+10. 
        kernel = np.ones((10, 1), dtype=np.float32)
        # We shift the floor mask so the "sum" at (x,y) represents pixels below it
        floor_float = floor_mask.astype(np.float32)
        floor_counts = cv.filter2D(floor_float, -1, kernel, anchor=(0, 0))
        seven_of_ten = floor_counts >= 7

        # 3. Dynamic Programming / Bottom-up constraint check
        # matches[y, x] stores if pixel satisfies constraint
        matches = np.zeros((height, width), dtype=bool)
        
        # Base Case: y >= 240
        matches[240:, :] = True

        cv.imshow('Floor Mask', floor_mask.astype(np.uint8)*255)
        
        # Recursive Case: y < 240 (Iterate 239 down to 0)
        for y in range(239, -1, -1):
            # Condition 1: (x, y+1) must match
            c1 = matches[y+1, :]
            
            # Condition 3: matches floor
            c3 = floor_mask[y, :]
            
            # Condition 4: matches a color mask
            # Same color mask at y+1 OR 7/10 floor pixels
            same_color = (color_mask_ids[y, :] == color_mask_ids[y+1, :]) & (color_mask_ids[y, :] > 0)
            c4 = any_color_mask[y, :] & (same_color | seven_of_ten[y+1, :])
            
            # Condition 2: matches nothing (neither floor nor color)
            # Must have 7/10 floor pixels
            c2 = (~floor_mask[y, :]) & (~any_color_mask[y, :]) & seven_of_ten[y+1, :]
            
            matches[y, :] = c1 & (c2 | c3 | c4)

        # 4. Find the top matching pixel for each column
        # np.argmax returns the first index where the value is True (lowest y)
        # We also check if any match exists in that column at all
        has_match = np.any(matches, axis=0)
        top_indices = np.argmax(matches, axis=0)

        # Create a copy of the original image to annotate
        annotated_img = img.copy()

        # Color the top pixels red [BGR: (0, 0, 255)]
        for x in range(width):
            if has_match[x]:
                y = top_indices[x]
                annotated_img[y, x] = [0, 0, 255]

        # Save to the output folder
        save_path = os.path.join(output_dir, filename)
        cv.imwrite(save_path, annotated_img)

        # 5. Sample 100 pixels
        y_coords, x_coords = np.where(matches)
        if len(y_coords) > 0:
            indices = np.random.choice(len(y_coords), min(100, len(y_coords)), replace=False)
            for idx in indices:
                all_sampled_hsv.append(hsv[y_coords[idx], x_coords[idx]])

    # 6. Plotting
    if all_sampled_hsv:
        all_sampled_hsv = np.array(all_sampled_hsv)
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        h = all_sampled_hsv[:, 0]
        s = all_sampled_hsv[:, 1]
        v = all_sampled_hsv[:, 2]
        
        img = ax.scatter(h, s, v, c=v, cmap='inferno', marker='o', alpha=0.5)
        ax.set_xlabel('Hue (H)')
        ax.set_ylabel('Saturation (S)')
        ax.set_zlabel('Value (V)')
        plt.title('3D HSV Pixel Distribution')
        plt.show()

    return all_sampled_hsv












def process_image(img: cv.Mat) -> cv.Mat:
    #img_annotated = annotate_image(img)

    #processed_img = process_blobs(img)
    #processed_img = process_blobs_refined(img)
    #blur = calculate_blur_coefficient(img)
    #processed_img = annotate_image(img, f"blur {blur}", 0)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    plot_hsv_space(hsv)
    return img


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







#test_vision_all_images()
#test_vision_all_images(image_number=138)
sample_hsv_values(138)
