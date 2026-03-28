import numpy as np

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class MotionPlanning:
    # def __init__(self,
    #              k_rho=0.4, k_alpha=1.2, k_beta=0.0, k_theta=2.0,
    #              v_max=0.4, w_max=0.8, dt=0.05,
    #              dist_thresh=0.1, theta_thresh=0.05,
    #              k_rep=0.3, rep_radius=0.5,
    #              W=None, H=None, k_bound=0.3, bound_radius=0.4,
    #              sensor_angles=None, sensor_rep_radius=0.4, k_sensor=0.5):
    def __init__(self,
                 k_rho=0.2, k_alpha=1.8, k_beta=0.2, k_theta=2.0,
                 v_max=0.4, w_max=0.8, dt=0.01,
                 dist_thresh=0.1, theta_thresh=0.05,
                 k_rep=0.3, rep_radius=0.5,
                 W=None, H=None, k_bound=0.3, bound_radius=0.4,
                 sensor_angles=None, sensor_rep_radius=0.5, k_sensor=0.8):
        # Attractive controller gains
        self.k_rho       = k_rho
        self.k_alpha     = k_alpha
        self.k_beta      = k_beta
        self.k_theta     = k_theta
        # Velocity limits
        self.v_max       = v_max
        self.w_max       = w_max
        self.dt          = dt
        # Convergence thresholds
        self.dist_thresh  = dist_thresh
        self.theta_thresh = theta_thresh
        # Point-obstacle repulsion
        self.k_rep       = k_rep
        self.rep_radius  = rep_radius
        # Boundary repulsion
        self.W           = W
        self.H           = H
        self.k_bound     = k_bound
        self.bound_radius = bound_radius
        # Ultrasonic sensor config
        self.sensor_angles = (sensor_angles if sensor_angles is not None
                            else np.array([-np.pi/6, np.pi/6]))  # ±30° (2 sensors)
        self.sensor_rep_radius = sensor_rep_radius
        self.k_sensor          = k_sensor

    # ------------------------------------------------------------------
    # Simulation / hardware interface
    # ------------------------------------------------------------------

    def read_ultrasonic_sensors(self, robot_state, obstacles):
        """
        Simulate ultrasonic sensor readings via ray-casting.

        Each sensor fires a ray at (theta + angle_offset). Returns the
        distance to the nearest obstacle along that ray, capped at
        sensor_rep_radius if nothing is detected.

        Args:
            robot_state: np.array([x, y, theta])
            obstacles:   list of np.array([x, y])

        Returns:
            readings: np.array, one distance per sensor angle
        """
        readings = np.full(len(self.sensor_angles), self.sensor_rep_radius)
        if not obstacles:
            return readings

        x, y, theta = robot_state
        HIT_RADIUS  = 0.08   # obstacle detection cross-section (m)

        for i, angle_offset in enumerate(self.sensor_angles):
            ray_angle = theta + angle_offset
            ray_dir   = np.array([np.cos(ray_angle), np.sin(ray_angle)])

            for obs in obstacles:
                to_obs = obs[:2] - np.array([x, y])
                proj   = np.dot(to_obs, ray_dir)
                if proj <= 0:
                    continue
                perp = np.linalg.norm(to_obs - proj * ray_dir)
                if perp < HIT_RADIUS and proj < readings[i]:
                    readings[i] = proj

        return readings

    def control_to_motor_commands(self, v, omega, wheel_base=0.15):
        """
        Convert (v, omega) to differential-drive motor commands in [-100, 100].
        Commands with |value| < 10 are zeroed (dead-zone).

        Args:
            v:          linear velocity (m/s)
            omega:      angular velocity (rad/s)
            wheel_base: distance between wheels (m)

        Returns:
            (left, right): int motor commands in [-100, 100]
        """
        v_left  = v - (omega * wheel_base / 2.0)
        v_right = v + (omega * wheel_base / 2.0)

        left  = np.clip((v_left  / self.v_max) * 100.0, -100, 100)
        right = np.clip((v_right / self.v_max) * 100.0, -100, 100)

        DEAD_ZONE = 10
        left  = 0.0 if abs(left)  < DEAD_ZONE else left
        right = 0.0 if abs(right) < DEAD_ZONE else right

        return int(round(left)), int(round(right))

    # ------------------------------------------------------------------
    # Internal repulsion helpers
    # ------------------------------------------------------------------

    def _repulsion_velocity(self, robot_state, obstacles):
        """APF repulsion from point obstacles, projected to body frame."""
        if not obstacles:
            return 0.0, 0.0

        fx, fy = 0.0, 0.0
        for obs in obstacles:
            diff = robot_state[:2] - obs[:2]
            d    = np.linalg.norm(diff)
            if 0 < d < self.rep_radius:
                mag = self.k_rep * (1.0/d - 1.0/self.rep_radius) / (d**2)
                fx += mag * diff[0]
                fy += mag * diff[1]

        theta  = robot_state[2]
        v_rep  =  fx * np.cos(theta) + fy * np.sin(theta)
        w_rep  = (-fx * np.sin(theta) + fy * np.cos(theta)) / max(self.rep_radius, 0.01)
        return v_rep, w_rep

    def _boundary_repulsion(self, robot_state):
        """APF repulsion from the four walls of the [0,W]x[0,H] arena."""
        if self.W is None and self.H is None:
            return 0.0, 0.0

        x, y, theta = robot_state
        fx, fy = 0.0, 0.0
        r = self.bound_radius

        walls = []
        if self.W is not None:
            walls += [
                (x,        np.array([ 1.0,  0.0])),  # left   wall x=0
                (self.W-x, np.array([-1.0,  0.0])),  # right  wall x=W
            ]
        if self.H is not None:
            walls += [
                (y,        np.array([ 0.0,  1.0])),  # bottom wall y=0
                (self.H-y, np.array([ 0.0, -1.0])),  # top    wall y=H
            ]

        for d, normal in walls:
            if 0 < d < r:
                mag = self.k_bound * (1.0/d - 1.0/r) / (d**2)
                fx += mag * normal[0]
                fy += mag * normal[1]

        v_rep  =  fx * np.cos(theta) + fy * np.sin(theta)
        w_rep  = (-fx * np.sin(theta) + fy * np.cos(theta)) / max(r, 0.01)
        return v_rep, w_rep

    def _sensor_repulsion(self, robot_state, sensor_readings):
        """
        Convert ultrasonic readings to body-frame repulsion (v, w).
        Force points opposite to each sensor's mounting angle.
        """
        fx_body, fy_body = 0.0, 0.0
        r = self.sensor_rep_radius

        for d, angle_offset in zip(sensor_readings, self.sensor_angles):
            if 0 < d < r:
                mag = self.k_sensor * (1.0/d - 1.0/r) / (d**2)
                fx_body -= mag * np.cos(angle_offset)
                fy_body -= mag * np.sin(angle_offset)

        v_rep = fx_body
        w_rep = fy_body / max(r, 0.01)
        return v_rep, w_rep

    # ------------------------------------------------------------------
    # Public control API
    # ------------------------------------------------------------------

    def compute_control(self, robot_state, waypoint,
                        obstacles=None, sensor_readings=None):
        """
        Phase 1: drive toward waypoint.
        Combines attractive control with obstacle, boundary, and sensor repulsion.

        Args:
            robot_state:     np.array([x, y, theta])
            waypoint:        np.array([x, y])
            obstacles:       list of np.array([x, y]) or None
            sensor_readings: output of read_ultrasonic_sensors(), or None
                             (auto-simulated from obstacles if not provided)

        Returns:
            (v, omega)
        """
        # Attractive
        rho           = np.linalg.norm(waypoint - robot_state[:2])
        angle_to_goal = np.arctan2(waypoint[1] - robot_state[1],
                                   waypoint[0] - robot_state[0])
        alpha = wrap_angle(angle_to_goal - robot_state[2])
        beta  = wrap_angle(-robot_state[2] - alpha)
        v_att = self.k_rho * rho
        w_att = self.k_alpha * alpha + self.k_beta * beta

        # Repulsive
        v_obs,   w_obs   = self._repulsion_velocity(robot_state, obstacles)
        v_bound, w_bound = self._boundary_repulsion(robot_state)

        if sensor_readings is None and obstacles:
            sensor_readings = self.read_ultrasonic_sensors(robot_state, obstacles)
        v_sens, w_sens = (self._sensor_repulsion(robot_state, sensor_readings)
                         if sensor_readings is not None else (0.0, 0.0))

        v = np.clip(v_att + v_obs + v_bound + v_sens, -self.v_max, self.v_max)
        w = np.clip(w_att + w_obs + w_bound + w_sens, -self.w_max, self.w_max)
        return v, w

    def compute_orient_control(self, robot_state, goal_position,
                               obstacles=None, sensor_readings=None):
        """
        Phase 2: rotate toward goal.
        Combines orientation control with obstacle, boundary, and sensor repulsion.

        Args:
            robot_state:     np.array([x, y, theta])
            goal_position:   np.array([x, y])
            obstacles:       list of np.array([x, y]) or None
            sensor_readings: output of read_ultrasonic_sensors(), or None

        Returns:
            (v, omega)
        """
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        theta = wrap_angle(theta_goal - robot_state[2])
        w_att = self.k_theta * theta

        v_obs,   w_obs   = self._repulsion_velocity(robot_state, obstacles)
        v_bound, w_bound = self._boundary_repulsion(robot_state)

        if sensor_readings is None and obstacles:
            sensor_readings = self.read_ultrasonic_sensors(robot_state, obstacles)
        v_sens, w_sens = (self._sensor_repulsion(robot_state, sensor_readings)
                         if sensor_readings is not None else (0.0, 0.0))

        v = np.clip(v_obs + v_bound + v_sens, -self.v_max, self.v_max)
        w = np.clip(w_att + w_obs + w_bound + w_sens, -self.w_max, self.w_max)
        return v, w

    # ------------------------------------------------------------------
    # Termination checks
    # ------------------------------------------------------------------

    def reached_waypoint(self, robot_state, waypoint):
        return np.linalg.norm(waypoint - robot_state[:2]) <= self.dist_thresh

    def reached_orientation(self, robot_state, goal_position):
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        return abs(wrap_angle(theta_goal - robot_state[2])) <= self.theta_thresh

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def step(self, robot_state, v, w):
        """Euler-integrate one timestep. Returns new state without mutating input."""
        state    = robot_state.copy()
        state[0] += self.dt * v * np.cos(state[2])
        state[1] += self.dt * v * np.sin(state[2])
        state[2]  = wrap_angle(state[2] + self.dt * w)
        return state


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':

    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

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

    planner = MotionPlanning(
        k_rep=0.6,   rep_radius=0.7,
        W=W, H=H,    k_bound=0.5,  bound_radius=0.6,
        k_sensor=0.8, sensor_rep_radius=0.5,
        # sensor_angles=np.linspace(-np.pi/2, np.pi/2, 7),
    )

    # ── Simulation ────────────────────────────────────────────────────────────
    robot_state = start.copy()
    all_paths   = []
    ray_log     = []   # (state, readings) snapshots for visualisation
    RAY_LOG_STEP = 15

    motor_left, motor_right, phases = [], [], []
    step_count = 0

    # Phase 1 — drive through waypoints
    for wp_idx, waypoint in enumerate(waypoints):
        leg = [robot_state.copy()]
        while not planner.reached_waypoint(robot_state, waypoint):
            readings = planner.read_ultrasonic_sensors(robot_state, obstacles)
            if step_count % RAY_LOG_STEP == 0:
                ray_log.append((robot_state.copy(), readings.copy()))

            v, w   = planner.compute_control(robot_state, waypoint,
                                             obstacles, readings)
            l, r   = planner.control_to_motor_commands(v, w)
            motor_left.append(l);  motor_right.append(r)
            phases.append(wp_idx + 1)

            print(f"WP{wp_idx+1} | "
                  f"v={v:+.3f}  ω={w:+.3f}  →  L={l:+4d}  R={r:+4d}  "
                  f"sensors=[{', '.join(f'{d:.2f}' for d in readings)}]")

            robot_state = planner.step(robot_state, v, w)
            leg.append(robot_state.copy())
            step_count += 1

        all_paths.append(np.array(leg))
        print(f"── reached waypoint {wp_idx + 1} ──")

    # Phase 2 — orient toward goal
    orient_path = [robot_state.copy()]
    while not planner.reached_orientation(robot_state, goal_position):
        readings = planner.read_ultrasonic_sensors(robot_state, obstacles)
        v, w     = planner.compute_orient_control(robot_state, goal_position,
                                                  obstacles, readings)
        l, r     = planner.control_to_motor_commands(v, w)
        motor_left.append(l);  motor_right.append(r)
        phases.append(len(waypoints) + 1)

        print(f"ORIENT | v={v:+.3f}  ω={w:+.3f}  →  L={l:+4d}  R={r:+4d}")

        robot_state = planner.step(robot_state, v, w)
        orient_path.append(robot_state.copy())

    orient_path = np.array(orient_path)
    print("── orientation reached ──")

    # ── Figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(17, 9))
    gs  = fig.add_gridspec(2, 2, width_ratios=[2, 1], hspace=0.38, wspace=0.32)
    ax      = fig.add_subplot(gs[:, 0])   # trajectory  (full height, left)
    ax_m    = fig.add_subplot(gs[0, 1])   # motor commands (top right)
    ax_sens = fig.add_subplot(gs[1, 1])   # sensor readings (bottom right)

    # ── Arena ─────────────────────────────────────────────────────────────────
    ax.add_patch(plt.Polygon([[0,0],[W,0],[W,H],[0,H]],
                             closed=True, fill=False,
                             edgecolor='black', linewidth=2,
                             linestyle='--', zorder=0))
    band = planner.bound_radius
    for wall_x, sign in [(0, +1), (W, -1)]:
        ax.add_patch(plt.Rectangle(
            (wall_x if sign > 0 else wall_x - band, 0), band, H,
            color='purple', alpha=0.07, zorder=0))
    for wall_y, sign in [(0, +1), (H, -1)]:
        ax.add_patch(plt.Rectangle(
            (0, wall_y if sign > 0 else wall_y - band), W, band,
            color='purple', alpha=0.07, zorder=0))

    # ── Obstacles ─────────────────────────────────────────────────────────────
    for i, obs in enumerate(obstacles):
        ax.add_patch(plt.Circle(obs, planner.rep_radius,
                                color='salmon', alpha=0.20, zorder=1))
        ax.plot(*obs, 'rs', markersize=8, zorder=4,
                label='Obstacle' if i == 0 else None)

    # ── Sensor rays ───────────────────────────────────────────────────────────
    for state, readings in ray_log:
        x, y, theta = state
        for d, angle_offset in zip(readings, planner.sensor_angles):
            ray_angle = theta + angle_offset
            x_end = x + d * np.cos(ray_angle)
            y_end = y + d * np.sin(ray_angle)
            hit   = d < planner.sensor_rep_radius * 0.95
            ax.plot([x, x_end], [y, y_end],
                    color='red' if hit else 'limegreen',
                    lw=0.8, alpha=0.45, zorder=2)
            if hit:
                ax.plot(x_end, y_end, 'r.', markersize=4, alpha=0.6, zorder=3)

    # ── Phase 1 trajectory ────────────────────────────────────────────────────
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
        arc_thetas  = np.linspace(theta_start, theta_end, 60)
        ax.plot(cx + 0.25 * np.cos(arc_thetas),
                cy + 0.25 * np.sin(arc_thetas),
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

    # ── Waypoints / start / goal ──────────────────────────────────────────────
    for i, wp in enumerate(waypoints):
        ax.plot(*wp, 'c^', markersize=9, zorder=4,
                label='Waypoint' if i == 0 else None)
        ax.annotate(f'WP{i+1}', xy=wp,
                    xytext=(wp[0] + 0.08, wp[1] + 0.12),
                    fontsize=8, color='teal')

    # Sensor ray legend proxies
    ax.plot([], [], color='limegreen', lw=1.2, alpha=0.7, label='Sensor – free')
    ax.plot([], [], color='red',       lw=1.2, alpha=0.7, label='Sensor – hit')

    ax.plot(*start[:2],    'go', markersize=10, zorder=5, label='Start')
    ax.plot(*goal_position,'k*', markersize=14, zorder=5, label='Goal')

    ax.set_title('MotionPlanning – APF + ultrasonic sensors', fontsize=12)
    ax.set_xlabel('x (m)');  ax.set_ylabel('y (m)')
    ax.set_xlim(-0.3, W + 0.3);  ax.set_ylim(-0.3, H + 0.3)
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=7.5)
    ax.grid(True, alpha=0.3)

    # ── Motor command timeline ────────────────────────────────────────────────
    t = np.arange(len(motor_left)) * planner.dt
    ax_m.plot(t, motor_left,  label='Left',  color='steelblue', lw=1.5)
    ax_m.plot(t, motor_right, label='Right', color='tomato',    lw=1.5)
    ax_m.axhline( 10, color='grey', lw=0.8, linestyle=':')
    ax_m.axhline(-10, color='grey', lw=0.8, linestyle=':')
    ax_m.fill_between(t, -10, 10, alpha=0.08, color='grey', label='Dead zone')

    phase_colors = plt.cm.Pastel1(np.linspace(0, 1, len(waypoints) + 1))
    prev_t, prev_p = 0.0, phases[0]
    for i, (ti, p) in enumerate(zip(t, phases)):
        if p != prev_p or i == len(t) - 1:
            label = f'WP{prev_p}' if prev_p <= len(waypoints) else 'Orient'
            ax_m.axvspan(prev_t, ti,
                         color=phase_colors[prev_p - 1], alpha=0.25, label=label)
            prev_t, prev_p = ti, p

    ax_m.set_title('Motor commands', fontsize=11)
    ax_m.set_xlabel('Time (s)');  ax_m.set_ylabel('Command')
    ax_m.set_ylim(-110, 110)
    ax_m.legend(fontsize=7, loc='upper right')
    ax_m.grid(True, alpha=0.3)

    # ── Sensor reading timeline ───────────────────────────────────────────────
    # Collect per-timestep min sensor reading across all sensors
    robot_state  = start.copy()
    sensor_history = []   # (timestep, per-sensor distances)

    for waypoint in waypoints:
        while not planner.reached_waypoint(robot_state, waypoint):
            readings = planner.read_ultrasonic_sensors(robot_state, obstacles)
            sensor_history.append(readings.copy())
            v, w        = planner.compute_control(robot_state, waypoint,
                                                  obstacles, readings)
            robot_state = planner.step(robot_state, v, w)

    while not planner.reached_orientation(robot_state, goal_position):
        readings = planner.read_ultrasonic_sensors(robot_state, obstacles)
        sensor_history.append(readings.copy())
        v, w        = planner.compute_orient_control(robot_state, goal_position,
                                                     obstacles, readings)
        robot_state = planner.step(robot_state, v, w)

    sensor_history = np.array(sensor_history)   # (T, n_sensors)
    t_s = np.arange(len(sensor_history)) * planner.dt
    sensor_cmap = plt.cm.viridis(np.linspace(0.1, 0.9, sensor_history.shape[1]))

    for s_idx in range(sensor_history.shape[1]):
        angle_deg = np.degrees(planner.sensor_angles[s_idx])
        ax_sens.plot(t_s, sensor_history[:, s_idx],
                     color=sensor_cmap[s_idx], lw=1.2,
                     label=f'{angle_deg:+.0f}°')

    ax_sens.axhline(planner.sensor_rep_radius, color='red', lw=1,
                    linestyle='--', label='Influence radius')
    ax_sens.fill_between(t_s,
                         0, planner.sensor_rep_radius,
                         alpha=0.06, color='red')
    ax_sens.set_title('Ultrasonic sensor readings', fontsize=11)
    ax_sens.set_xlabel('Time (s)');  ax_sens.set_ylabel('Distance (m)')
    ax_sens.legend(fontsize=7, loc='upper right', ncol=2)
    ax_sens.grid(True, alpha=0.3)

    plt.suptitle('MotionPlanning – full simulation', fontsize=13, y=1.01)
    plt.tight_layout()
    plt.show()
