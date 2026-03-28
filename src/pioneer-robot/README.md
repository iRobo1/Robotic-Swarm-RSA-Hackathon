# High-Level Strategy

For now the Mirte Master robot operates using a reactive, mapless navigation architecture designed to quickly navigate the chaotic, multi-robot arena without the heavy computational overhead of global pathfinding (like A* or SLAM). For future approaches, we could try implementing mapping as well. 

## 1. Target Selection & State Management
The robot's high-level behavior is controlled by a Finite State Machine (FSM) with three primary states:

* **`IDLE`:** The robot waits at its hardcoded Starting Zone. It monitors a shared `objective_queue` populated by the Pioneer robots via the server. When an objective appears, it triggers the pickup sequence and transitions to delivering.
* **`DELIVERING`:** The robot navigates to the target's X, Y coordinates, executes the drop-off sequence upon arrival, and targets the Starting Zone.
* **`RETURNING`:** The robot navigates back to Starting Zone to await the next objective.

## 2. Pathfinding & Navigation
Instead of pre-calculating a complex route, the robot uses a straight-line "Go-To-Goal" approach combined with real-time obstacle avoidance. Because the Mirte Master is holonomic, it can strafe sideways to dodge obstacles without needing to rotate its chassis. 

## 3. Core Algorithm: Artificial Potential Fields (APF)
All movement is dictated by a local Artificial Potential Field algorithm.

* **Attractive Force:** A vector continuously pulls the robot toward its current target coordinates.
* **Repulsive Force:** The 360-degree 2D LiDAR array acts as a protective bubble. Any obstacle (static walls or moving opponent robots) detected within a specific threshold (0.5 meters) generates a repulsive force pushing the robot away.
* **Kinematic Mapping:** The attractive and repulsive forces are summed together in the robot's local coordinate frame. This final vector is safely scaled down to abide by the strict 0.4 m/s speed limit and fed directly into the Mecanum wheels as local X (forward/back) and Y (strafe) velocities.