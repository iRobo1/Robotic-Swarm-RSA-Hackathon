import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from mpl_toolkits.mplot3d import Axes3D

def process_hsv_clusters(image, n_min_occurrences, m_clusters):
    # 1. Convert to HSV
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 2. Reshape and count unique HSV values
    pixels = hsv_img.reshape(-1, 3)
    unique_colors, counts = np.unique(pixels, axis=0, return_counts=True)
    
    # 3. Filter values occurring at least N times
    filtered_hsv = unique_colors[counts >= n_min_occurrences]
    
    if len(filtered_hsv) < m_clusters:
        return filtered_hsv, [], "Not enough unique values to cluster."

    # 4. Cluster the points into M HSV ranges
    kmeans = KMeans(n_clusters=m_clusters, n_init=10, random_state=42)
    labels = kmeans.fit_predict(filtered_hsv)
    
    # Generate ranges (Min/Max per cluster)
    hsv_ranges = []
    for i in range(m_clusters):
        cluster_points = filtered_hsv[labels == i]
        lower_bound = np.min(cluster_points, axis=0)
        upper_bound = np.max(cluster_points, axis=0)
        hsv_ranges.append((lower_bound, upper_bound))
        
    return filtered_hsv, hsv_ranges, labels


def draw_hsv_box(ax, low, high, color='black'):
    """Draws a 3D wireframe bounding box."""
    # Define the 8 vertices of the box
    h = [low[0], high[0]]
    s = [low[1], high[1]]
    v = [low[2], high[2]]
    
    # Iterate through the edges
    for s_i in s:
        for v_i in v:
            ax.plot(h, [s_i, s_i], [v_i, v_i], color=color, linewidth=1, alpha=0.7)
    for h_i in h:
        for v_i in v:
            ax.plot([h_i, h_i], s, [v_i, v_i], color=color, linewidth=1, alpha=0.7)
    for h_i in h:
        for s_i in s:
            ax.plot([h_i, h_i], [s_i, s_i], v, color=color, linewidth=1, alpha=0.7)


def visualize_hsv(filtered_hsv, labels, hsv_ranges):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Normalize for visualization: H is [0, 179], S/V are [0, 255]
    # To see actual colors, we convert HSV back to RGB for the scatter plot
    hsv_normalized = filtered_hsv.copy().astype(np.float32)
    rgb_colors = []
    for val in filtered_hsv:
        color = cv2.cvtColor(np.uint8([[val]]), cv2.COLOR_HSV2RGB)[0][0]
        rgb_colors.append(color / 255.0)

    # Plot the unique HSV points
    ax.scatter(filtered_hsv[:, 0], filtered_hsv[:, 1], filtered_hsv[:, 2], 
               c=rgb_colors, marker='o', s=10, alpha=0.5)

    # Draw Bounding Boxes for each cluster
    for i, (low, high) in enumerate(hsv_ranges):
        # We use a distinct color for the box lines or keep it neutral
        draw_hsv_box(ax, low, high, color='red')
        
        # Label the clusters at their minimum coordinate point
        ax.text(low[0], low[1], low[2], f"Cluster {i+1}", 
                color='red', fontsize=10, fontweight='bold')

    ax.set_xlabel('Hue (0-179)')
    ax.set_ylabel('Saturation (0-255)')
    ax.set_zlabel('Value (0-255)')
    ax.set_title(f'Top HSV Values and their Clusters')
    plt.tight_layout()
    plt.show()

# --- Execution ---
# Load an image
img = cv2.imread('data/pioneer_images/raw/138.png') # Replace with your file
if img is not None:
    N = 50  # Minimum occurrences
    M = 5   # Number of clusters
    
    points, ranges, cluster_labels = process_hsv_clusters(img, N, M)
    
    print(f"Found {len(points)} unique HSV values occurring >= {N} times.")
    for idx, (low, high) in enumerate(ranges):
        print(f"Cluster {idx+1} Range: Lower{low} - Upper{high}")
        
    visualize_hsv(points, cluster_labels, ranges)