# System Engineering 1 - Operations

## A. Pioneering

### A1 Spread open

The pioneer moves to a arbitrary position in its respective quadrant while avoiding obstacle and other robots. At any time, if it detects an objective, it switches to **A2**. Once it reaches to the target position, it generates a new arbitrary position within the quadrant, repeating **A1** scenario. 

### A2 Inspect objective

The pioneer closes its distance to the bin up to a defined threshold. Once the distance is met, the pioneer finds the AprilTag on the objective by constantly changing its position and reorientating towards the objective's centroid. Once the AprilTag is found, it scans the tag, and switches to **A1**.

## B. Pick-and-Place

T