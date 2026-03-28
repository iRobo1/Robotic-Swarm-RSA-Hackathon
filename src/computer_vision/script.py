import numpy as np
import json

# The raw data provided
raw_data = """
62 16 94 | 37 20 99 | 36 17 98 | 46 19 96 | 37 20 99 | 35 22 94 | 35 15 97 | 45 19 93
63 18 94 | 48 21 94 | 51 21 96 | 70 4 92 | 65 9 93 | 51 20 96 | 49 21 92 | 51 14 94
40 18 97 | 40 21 93 | 45 29 87 | 31 33 76 | 32 30 76 | 40 22 88 | 45 23 87 | 50 26 77
46 21 87 | 33 30 71 | 32 28 73 | 36 16 97 | 40 23 91 | 43 24 87 | 48 25 87 | 34 29 76
37 24 89 | 32 28 74 | 46 23 81 | 40 29 85 | 36 32 77 | 48 24 85 | 52 21 82 | 51 22 83
49 18 88 | 43 18 96 | 43 22 93 | 42 21 78 | 28 27 86 | 49 26 74 | 38 16 95 | 36 36 87
58 22 66 | 51 27 76 | 49 24 74 | 49 23 73 | 43 27 77 | 34 31 81 | 37 33 81 | 33 27 78
31 33 87 | 42 29 87 | 33 33 85 | 41 33 86 | 36 20 96 | 42 35 75 | 53 33 69 | 54 35 71
45 28 74 | 37 23 90 | 38 25 89 | 42 31 85 | 29 38 74 | 40 34 90 | 41 34 88 | 35 21 98
34 42 74 | 38 28 87 | 33 23 92 | 37 25 89 | 32 39 69 | 33 40 74 | 27 37 67 | 40 38 66
32 37 65 | 27 36 70 | 35 24 92 | 34 20 93 | 31 39 74 | 36 26 94 | 37 33 81
"""

# Clean and parse the points
points_list = []
for line in raw_data.replace('|', '\n').split('\n'):
    parts = line.split()
    for i in range(0, len(parts), 3):
        points_list.append([float(parts[i]), float(parts[i+1]), float(parts[i+2])])

points = np.array(points_list)

# --- CONVERSION TO OPENCV RANGE ---
# Hue: 0-360 -> 0-180
# Saturation: 0-100 -> 0-255
# Value: 0-100 -> 0-255
points[:, 0] = points[:, 0] / 2.0
points[:, 1] = points[:, 1] * 255.0 / 100.0
points[:, 2] = points[:, 2] * 255.0 / 100.0

# Round to integers for OpenCV compatibility
points = np.round(points).astype(int)
# ----------------------------------

def get_ranges(pts, tolerance=5):
    ranges = []
    # Sort points to find neighbors
    sorted_indices = np.lexsort((pts[:, 2], pts[:, 1], pts[:, 0]))
    pts = pts[sorted_indices]
    
    used = np.zeros(len(pts), dtype=bool)
    
    for i in range(len(pts)):
        if used[i]: continue
        
        current_min = pts[i] - tolerance
        current_max = pts[i] + tolerance
        
        # Ensure values stay within OpenCV legal bounds
        current_min = np.clip(current_min, [0, 0, 0], [180, 255, 255])
        current_max = np.clip(current_max, [0, 0, 0], [180, 255, 255])
        
        used[i] = True
        
        for j in range(i + 1, len(pts)):
            if used[j]: continue
            p = pts[j]
            if np.all(p - tolerance <= current_max + 1) and np.all(p + tolerance >= current_min - 1):
                if np.all(np.abs(p - pts[i]) <= tolerance * 2):
                    current_min = np.minimum(current_min, p - tolerance)
                    current_max = np.maximum(current_max, p + tolerance)
                    used[j] = True
                    
        ranges.append((np.clip(current_min, 0, 255), np.clip(current_max, 0, 255)))
    return ranges

hsv_ranges = get_ranges(points)

# Formatting the output string
ranges_str = ",\n".join([
    f"                (np.array([{low[0]}, {low[1]}, {low[2]}]), np.array([{high[0]}, {high[1]}, {high[2]}]))"
    for low, high in hsv_ranges
])

config = f"""{{
    "name": "floor",
    "draw_color": (255, 255, 0),
    "ranges": [
{ranges_str}
    ]
}}"""

print(config)