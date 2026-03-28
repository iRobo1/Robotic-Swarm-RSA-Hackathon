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
                 k_rep=0.3, rep_radius=0.5,
                 W=None, H=None, k_bound=0.3, bound_radius=0.1):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.k_theta = k_theta
        self.v_max = v_max
        self.w_max = w_max
        self.dt = dt
        self.dist_thresh = dist_thresh
        self.theta_thresh = theta_thresh
        self.k_rep = k_rep
        self.rep_radius = rep_radius
        self.W = W                   # arena width  (x: 0 … W), None = disabled
        self.H = H                   # arena height (y: 0 … H), None = disabled
        self.k_bound = k_bound       # boundary repulsion gain
        self.bound_radius = bound_radius  # influence distance from each wall (m)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _repulsion_velocity(self, robot_state, obstacles):
        """Repulsion from other robots / point obstacles."""
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

        theta = robot_state[2]
        v_rep =  fx * np.cos(theta) + fy * np.sin(theta)
        w_rep = (-fx * np.sin(theta) + fy * np.cos(theta)) / max(self.bound_radius, 0.01)
        return v_rep, w_rep

    def _boundary_repulsion(self, robot_state):
        """
        Repulsion from the four walls of the [0,W] x [0,H] arena.

        Each wall is treated as a planar obstacle. A force is applied
        normal to the wall (pointing inward) whenever the robot is within
        bound_radius of that wall, using the same APF formula as point
        obstacles:  magnitude = k_bound * (1/d - 1/r) / d²

        Args:
            robot_state: np.array([x, y, theta])

        Returns:
            (v_rep, w_rep): projected onto the robot body frame
        """
        if self.W is None and self.H is None:
            return 0.0, 0.0

        x, y, theta = robot_state
        fx, fy = 0.0, 0.0
        r = self.bound_radius

        # Each entry: (distance to wall, outward-pointing unit normal)
        walls = []
        if self.W is not None:
            walls += [
                (x,        np.array([ 1.0,  0.0])),   # left wall   x = 0
                (self.W-x, np.array([-1.0,  0.0])),   # right wall  x = W
            ]
        if self.H is not None:
            walls += [
                (y,        np.array([ 0.0,  1.0])),   # bottom wall y = 0
                (self.H-y, np.array([ 0.0, -1.0])),   # top wall    y = H
            ]

        for d, normal in walls:
            if 0 < d < r:
                magnitude = self.k_bound * (1.0/d - 1.0/r) / (d**2)
                fx += magnitude * normal[0]
                fy += magnitude * normal[1]

        v_rep =  fx * np.cos(theta) + fy * np.sin(theta)
        w_rep = (-fx * np.sin(theta) + fy * np.cos(theta)) / max(r, 0.01)
        return v_rep, w_rep

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_control(self, robot_state, waypoint, obstacles=None):
        """Phase 1: drive toward waypoint with obstacle + boundary repulsion."""
        rho = np.linalg.norm(waypoint - robot_state[:2])
        angle_to_goal = np.arctan2(waypoint[1] - robot_state[1],
                                   waypoint[0] - robot_state[0])
        alpha = wrap_angle(angle_to_goal - robot_state[2])
        beta  = wrap_angle(-robot_state[2] - alpha)

        v_att = self.k_rho * rho
        w_att = self.k_alpha * alpha + self.k_beta * beta

        v_obs, w_obs   = self._repulsion_velocity(robot_state, obstacles)
        v_bound, w_bound = self._boundary_repulsion(robot_state)

        v = np.clip(v_att + v_obs + v_bound, -self.v_max, self.v_max)
        w = np.clip(w_att + w_obs + w_bound, -self.w_max, self.w_max)
        return v, w

    def compute_orient_control(self, robot_state, goal_position, obstacles=None):
        """Phase 2: rotate toward goal with obstacle + boundary repulsion."""
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        theta = wrap_angle(theta_goal - robot_state[2])
        w_att = self.k_theta * theta

        v_obs, w_obs     = self._repulsion_velocity(robot_state, obstacles)
        v_bound, w_bound = self._boundary_repulsion(robot_state)

        v = np.clip(v_obs + v_bound, -self.v_max, self.v_max)
        w = np.clip(w_att + w_obs + w_bound, -self.w_max, self.w_max)
        return v, w

    def reached_waypoint(self, robot_state, waypoint):
        return np.linalg.norm(waypoint - robot_state[:2]) <= self.dist_thresh

    def reached_orientation(self, robot_state, goal_position):
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        return abs(wrap_angle(theta_goal - robot_state[2])) <= self.theta_thresh

    def step(self, robot_state, v, w):
        state = robot_state.copy()
        state[0] += self.dt * v * np.cos(state[2])
        state[1] += self.dt * v * np.sin(state[2])
        state[2]  = wrap_angle(state[2] + self.dt * w)
        return state

    def control_to_motor_commands(self, v, omega, wheel_base=0.15):
        v_left  = v - (omega * wheel_base / 2.0)
        v_right = v + (omega * wheel_base / 2.0)
        left  = np.clip((v_left  / self.v_max) * 100.0, -100, 100)
        right = np.clip((v_right / self.v_max) * 100.0, -100, 100)
        DEAD_ZONE = 10
        left  = 0.0 if abs(left)  < DEAD_ZONE else left
        right = 0.0 if abs(right) < DEAD_ZONE else right
        return int(round(left)), int(round(right))
    

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # ── Scene setup ───────────────────────────────────────────────────────────
    W, H          = 6.0, 4.0
    start         = np.array([0.3, 0.3, 0.0])
    waypoints     = [np.array([2.0, 1.5]),
                     np.array([4.0, 0.8]),
                     np.array([5.0, 3.0])]
    goal_position = np.array([5.5, 3.5])
    obstacles     = [np.array([1.2, 0.6]),
                     np.array([3.0, 2.2]),
                     np.array([4.2, 1.8])]

    planner     = MotionPlanning(k_rep=0.6,   rep_radius=0.7,
                                 W=W, H=H,
                                 k_bound=0.5, bound_radius=0.6)
    robot_state = start.copy()

    # ── Phase 1: drive through every waypoint ─────────────────────────────────
    all_paths = []
    for wp_idx, waypoint in enumerate(waypoints):
        leg = [robot_state.copy()]
        while not planner.reached_waypoint(robot_state, waypoint):
            v, w     = planner.compute_control(robot_state, waypoint, obstacles)
            left, right = planner.control_to_motor_commands(v, w)
            print(f"WP{wp_idx+1} | v={v:+.3f}  ω={w:+.3f}  →  L={left:+4d}  R={right:+4d}")
            robot_state = planner.step(robot_state, v, w)
            leg.append(robot_state.copy())
        all_paths.append(np.array(leg))
        print(f"── reached waypoint {wp_idx+1} ──")

    # ── Phase 2: orient toward goal ───────────────────────────────────────────
    orient_path = [robot_state.copy()]
    while not planner.reached_orientation(robot_state, goal_position):
        v, w        = planner.compute_orient_control(robot_state, goal_position, obstacles)
        left, right = planner.control_to_motor_commands(v, w)
        print(f"ORIENT | v={v:+.3f}  ω={w:+.3f}  →  L={left:+4d}  R={right:+4d}")
        robot_state = planner.step(robot_state, v, w)
        orient_path.append(robot_state.copy())
    orient_path = np.array(orient_path)
    print("── orientation reached ──")

    # ── Figure / axes ─────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(15, 6),
                             gridspec_kw={'width_ratios': [2, 1]})
    ax   = axes[0]   # main trajectory view
    ax_m = axes[1]   # motor-command timeline

    # ── Arena boundary ────────────────────────────────────────────────────────
    arena = plt.Polygon([[0,0],[W,0],[W,H],[0,H]],
                        closed=True, fill=False,
                        edgecolor='black', linewidth=2,
                        linestyle='--', zorder=0)
    ax.add_patch(arena)

    # Boundary influence bands (shaded)
    band = planner.bound_radius
    for wall_x, sign in [(0, +1), (W, -1)]:
        ax.add_patch(plt.Rectangle((wall_x if sign > 0 else wall_x - band, 0),
                                   band, H,
                                   color='purple', alpha=0.07, zorder=0))
    for wall_y, sign in [(0, +1), (H, -1)]:
        ax.add_patch(plt.Rectangle((0, wall_y if sign > 0 else wall_y - band),
                                   W, band,
                                   color='purple', alpha=0.07, zorder=0))

    # ── Obstacles ─────────────────────────────────────────────────────────────
    for i, obs in enumerate(obstacles):
        ax.add_patch(plt.Circle(obs, planner.rep_radius,
                                color='salmon', alpha=0.20, zorder=1))
        ax.plot(*obs, 'rs', markersize=8, zorder=4,
                label='Obstacle' if i == 0 else None)

    # ── Phase 1 legs ─────────────────────────────────────────────────────────
    leg_colors = plt.cm.Blues(np.linspace(0.45, 0.85, len(all_paths)))
    for idx, (leg, color) in enumerate(zip(all_paths, leg_colors)):
        ax.plot(leg[:, 0], leg[:, 1], color=color,
                linewidth=2, alpha=0.85, zorder=2,
                label='Phase 1 – driving' if idx == 0 else None)
        arrow_step = max(1, len(leg) // 15)
        for i in range(0, len(leg), arrow_step):
            x, y, theta = leg[i]
            ax.annotate('',
                xy=(x + np.cos(theta) * 0.12, y + np.sin(theta) * 0.12),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.4),
                zorder=3)

    # ── Phase 2 arc ───────────────────────────────────────────────────────────
    if len(orient_path) > 1:
        cx, cy      = orient_path[0, 0], orient_path[0, 1]
        theta_start = orient_path[0,  2]
        theta_end   = orient_path[-1, 2]
        r_arc       = 0.25
        arc_thetas  = np.linspace(theta_start, theta_end, 60)
        ax.plot(cx + r_arc * np.cos(arc_thetas),
                cy + r_arc * np.sin(arc_thetas),
                color='orange', lw=2.5, zorder=3,
                label='Phase 2 – orienting')
        ax.annotate('',
            xy=(cx + np.cos(theta_start) * 0.38,
                cy + np.sin(theta_start) * 0.38),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle='->', color='grey', lw=1.8))
        ax.annotate('',
            xy=(cx + np.cos(theta_end) * 0.38,
                cy + np.sin(theta_end) * 0.38),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle='-|>', color='red', lw=2.5))

    # ── Waypoints ─────────────────────────────────────────────────────────────
    for i, wp in enumerate(waypoints):
        ax.plot(*wp, 'c^', markersize=9, zorder=4,
                label='Waypoint' if i == 0 else None)
        ax.annotate(f'WP{i+1}', xy=wp,
                    xytext=(wp[0] + 0.08, wp[1] + 0.12),
                    fontsize=8, color='teal')

    # ── Start / goal markers ──────────────────────────────────────────────────
    ax.plot(*start[:2],    'go', markersize=10, zorder=5, label='Start')
    ax.plot(*goal_position,'k*', markersize=14, zorder=5, label='Goal')

    ax.set_title('MotionPlanning – APF with obstacle + boundary repulsion',
                 fontsize=12)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(-0.3, W + 0.3)
    ax.set_ylim(-0.3, H + 0.3)
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Motor command timeline ────────────────────────────────────────────────
    # Re-run simulation to collect motor commands per timestep
    motor_left, motor_right, phases = [], [], []
    robot_state = start.copy()

    for wp_idx, waypoint in enumerate(waypoints):
        while not planner.reached_waypoint(robot_state, waypoint):
            v, w = planner.compute_control(robot_state, waypoint, obstacles)
            l, r = planner.control_to_motor_commands(v, w)
            motor_left.append(l);  motor_right.append(r)
            phases.append(wp_idx + 1)
            robot_state = planner.step(robot_state, v, w)

    while not planner.reached_orientation(robot_state, goal_position):
        v, w = planner.compute_orient_control(robot_state, goal_position, obstacles)
        l, r = planner.control_to_motor_commands(v, w)
        motor_left.append(l);  motor_right.append(r)
        phases.append(len(waypoints) + 1)
        robot_state = planner.step(robot_state, v, w)

    t = np.arange(len(motor_left)) * planner.dt
    ax_m.plot(t, motor_left,  label='Left',  color='steelblue', lw=1.5)
    ax_m.plot(t, motor_right, label='Right', color='tomato',    lw=1.5)
    ax_m.axhline( 10, color='grey', lw=0.8, linestyle=':')
    ax_m.axhline(-10, color='grey', lw=0.8, linestyle=':')
    ax_m.fill_between(t, -10, 10, alpha=0.07, color='grey', label='Dead zone')

    # Shade phase transitions
    phase_colors = plt.cm.Pastel1(np.linspace(0, 1, len(waypoints) + 1))
    prev_t, prev_p = 0.0, phases[0]
    for i, (ti, p) in enumerate(zip(t, phases)):
        if p != prev_p or i == len(t) - 1:
            ax_m.axvspan(prev_t, ti,
                         color=phase_colors[prev_p - 1], alpha=0.25,
                         label=f'WP{prev_p}' if prev_p <= len(waypoints) else 'Orient')
            prev_t, prev_p = ti, p

    ax_m.set_title('Motor commands over time', fontsize=12)
    ax_m.set_xlabel('Time (s)')
    ax_m.set_ylabel('Motor command')
    ax_m.set_ylim(-110, 110)
    ax_m.legend(fontsize=8, loc='upper right')
    ax_m.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()
