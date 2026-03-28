import numpy as np
import math

def wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

class MotionPlanning:
    def __init__(self,
                 k_rho=0.4, k_alpha=1.2, k_beta=0.0, k_theta=2.0,
                 v_max=0.4, w_max=0.8, dt=0.05,
                 dist_thresh=0.1, theta_thresh=0.05,
                 k_rep=0.3, rep_radius=0.5):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.k_theta = k_theta
        self.v_max = v_max
        self.w_max = w_max
        self.dt = dt
        self.dist_thresh = dist_thresh
        self.theta_thresh = theta_thresh
        self.k_rep = k_rep           # repulsion gain
        self.rep_radius = rep_radius # influence radius (m)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _repulsion_velocity(self, robot_state, obstacles):
        """
        Compute a repulsive (vx, vy) world-frame velocity from all obstacles
        within rep_radius, then project onto the robot's heading to get (v, w).

        Each obstacle inside rep_radius contributes a force proportional to
        1/d - 1/rep_radius, pointing away from the obstacle.

        Args:
            robot_state: np.array([x, y, theta])
            obstacles:   list/array of np.array([x, y]) or None

        Returns:
            (v_rep, w_rep): scalar repulsive corrections
        """
        if not obstacles:
            return 0.0, 0.0

        fx, fy = 0.0, 0.0
        for obs in obstacles:
            diff = robot_state[:2] - obs[:2]
            d = np.linalg.norm(diff)
            if 0 < d < self.rep_radius:
                magnitude = self.k_rep * (1.0/d - 1.0/self.rep_radius) / (d**2)
                fx += magnitude * diff[0]
                fy += magnitude * diff[1]

        # Project world-frame repulsion onto robot's local heading
        theta = robot_state[2]
        v_rep =  fx * np.cos(theta) + fy * np.sin(theta)
        w_rep = (-fx * np.sin(theta) + fy * np.cos(theta)) / max(self.rep_radius, 0.01)
        return v_rep, w_rep

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_control(self, robot_state, waypoint, obstacles=None):
        """
        Phase 1: Drive toward a waypoint with optional obstacle repulsion.

        Args:
            robot_state: np.array([x, y, theta])
            waypoint:    np.array([x, y])
            obstacles:   list of np.array([x, y]) for other robots / obstacles

        Returns:
            (v, omega): linear and angular velocity
        """
        # Attractive control
        rho = np.linalg.norm(waypoint - robot_state[:2])
        angle_to_goal = np.arctan2(waypoint[1] - robot_state[1],
                                   waypoint[0] - robot_state[0])
        alpha = wrap_angle(angle_to_goal - robot_state[2])
        beta  = wrap_angle(-robot_state[2] - alpha)

        v_att = self.k_rho * rho
        w_att = self.k_alpha * alpha + self.k_beta * beta

        # Repulsive correction
        v_rep, w_rep = self._repulsion_velocity(robot_state, obstacles)

        v = np.clip(v_att + v_rep, -self.v_max, self.v_max)
        w = np.clip(w_att + w_rep, -self.w_max, self.w_max)
        return v, w

    def compute_orient_control(self, robot_state, goal_position, obstacles=None):
        """
        Phase 2: Rotate to face goal with optional obstacle repulsion.

        Args:
            robot_state:   np.array([x, y, theta])
            goal_position: np.array([x, y])
            obstacles:     list of np.array([x, y]) or None

        Returns:
            (v, omega): linear velocity (may be nonzero if pushed) + angular velocity
        """
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        theta = wrap_angle(theta_goal - robot_state[2])
        w_att = self.k_theta * theta

        v_rep, w_rep = self._repulsion_velocity(robot_state, obstacles)

        v = np.clip(v_rep, -self.v_max, self.v_max)
        w = np.clip(w_att + w_rep, -self.w_max, self.w_max)
        return v, w

    def reached_waypoint(self, robot_state, waypoint):
        return np.linalg.norm(waypoint - robot_state[:2]) <= self.dist_thresh

    def reached_orientation(self, robot_state, goal_position):
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        return abs(wrap_angle(theta_goal - robot_state[2])) <= self.theta_thresh

    def step(self, robot_state, v, w):
        """Integrate one timestep; returns updated robot_state (does not mutate input)."""
        state = robot_state.copy()
        state[0] += self.dt * v * np.cos(state[2])
        state[1] += self.dt * v * np.sin(state[2])
        state[2]  = wrap_angle(state[2] + self.dt * w)
        return state
    

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # ── Scene setup ───────────────────────────────────────────────────────────
    start         = np.array([0.0, 0.0, 0.0])
    waypoints     = [np.array([2.0, 1.5]), np.array([4.0, 0.5])]
    goal_position = np.array([5.0, 2.0])
    obstacles     = [np.array([1.0, 0.8]),
                     np.array([2.8, 0.9]),
                     np.array([3.5, 1.6])]

    planner     = MotionPlanning(k_rep=0.6, rep_radius=0.7)
    robot_state = start.copy()

    all_paths   = []   # one array per waypoint-to-waypoint leg
    orient_path = []   # phase-2 rotation at the final waypoint

    # ── Phase 1: drive through every waypoint ─────────────────────────────────
    for waypoint in waypoints:
        leg = [robot_state.copy()]
        while not planner.reached_waypoint(robot_state, waypoint):
            v, w = planner.compute_control(robot_state, waypoint, obstacles)
            robot_state = planner.step(robot_state, v, w)
            leg.append(robot_state.copy())
        all_paths.append(np.array(leg))

    # ── Phase 2: orient toward goal ───────────────────────────────────────────
    orient_path = [robot_state.copy()]
    while not planner.reached_orientation(robot_state, goal_position):
        v, w = planner.compute_orient_control(robot_state, goal_position, obstacles)
        robot_state = planner.step(robot_state, v, w)
        orient_path.append(robot_state.copy())
    orient_path = np.array(orient_path)

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 6))
    leg_colors = plt.cm.Blues(np.linspace(0.45, 0.85, len(all_paths)))

    for idx, (leg, color) in enumerate(zip(all_paths, leg_colors)):
        label = 'Phase 1 – driving' if idx == 0 else None
        ax.plot(leg[:, 0], leg[:, 1], color=color,
                linewidth=2, alpha=0.8, zorder=2, label=label)

        # Heading arrows along the leg
        step = max(1, len(leg) // 15)
        for i in range(0, len(leg), step):
            x, y, theta = leg[i]
            ax.annotate('',
                xy=(x + np.cos(theta) * 0.12, y + np.sin(theta) * 0.12),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.4),
                zorder=3)

    # Phase 2 arc
    if len(orient_path) > 1:
        cx, cy       = orient_path[0, 0], orient_path[0, 1]
        theta_start  = orient_path[0,  2]
        theta_end    = orient_path[-1, 2]
        r            = 0.25
        arc_thetas   = np.linspace(theta_start, theta_end, 60)
        ax.plot(cx + r * np.cos(arc_thetas),
                cy + r * np.sin(arc_thetas),
                color='orange', lw=2.5, zorder=3, label='Phase 2 – orienting')
        ax.annotate('',
            xy=(cx + np.cos(theta_start) * 0.38, cy + np.sin(theta_start) * 0.38),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle='->', color='grey', lw=1.8))
        ax.annotate('',
            xy=(cx + np.cos(theta_end) * 0.38, cy + np.sin(theta_end) * 0.38),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle='-|>', color='red', lw=2.5))

    # Obstacles with influence radius
    for i, obs in enumerate(obstacles):
        circle = plt.Circle(obs, planner.rep_radius,
                            color='salmon', alpha=0.20, zorder=1)
        ax.add_patch(circle)
        ax.plot(*obs, 'rs', markersize=8, zorder=4,
                label='Obstacle' if i == 0 else None)

    # Waypoints
    for i, wp in enumerate(waypoints):
        ax.plot(*wp, 'c^', markersize=9, zorder=4,
                label='Waypoint' if i == 0 else None)
        ax.annotate(f'WP{i+1}', xy=wp, xytext=(wp[0]+0.08, wp[1]+0.1),
                    fontsize=8, color='teal')

    # Start / goal
    ax.plot(*start[:2],    'go', markersize=10, zorder=5, label='Start')
    ax.plot(*goal_position,'k*', markersize=14, zorder=5, label='Goal')

    ax.set_title('MotionPlanning – APF with obstacle repulsion', fontsize=13)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper left', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    