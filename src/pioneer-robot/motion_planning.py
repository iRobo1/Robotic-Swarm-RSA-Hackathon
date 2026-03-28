import numpy as np
import math

class MotionPlanning:
    def __init__(self,
                 k_rho=0.4, k_alpha=1.2, k_beta=0.0, k_theta=2.0,
                 v_max=0.4, w_max=0.8, dt=0.05,
                 dist_thresh=0.1, theta_thresh=0.05):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.k_theta = k_theta
        self.v_max = v_max
        self.w_max = w_max
        self.dt = dt
        self.dist_thresh = dist_thresh
        self.theta_thresh = theta_thresh

    def compute_control(self, robot_state, waypoint):
        """
        Phase 1: Drive toward a waypoint.
        
        Args:
            robot_state: np.array([x, y, theta])
            waypoint:    np.array([x, y])
        
        Returns:
            (v, omega): linear and angular velocity
        """
        rho = np.linalg.norm(waypoint - robot_state[:2])
        angle_to_goal = np.arctan2(waypoint[1] - robot_state[1],
                                   waypoint[0] - robot_state[0])
        alpha = self.wrap_angle(angle_to_goal - robot_state[2])
        beta  = self.wrap_angle(-robot_state[2] - alpha)

        v = np.clip(self.k_rho * rho, -self.v_max, self.v_max)
        w = np.clip(self.k_alpha * alpha + self.k_beta * beta, -self.w_max, self.w_max)
        return v, w

    def compute_orient_control(self, robot_state, goal_position):
        """
        Phase 2: Rotate to face a goal position (zero linear velocity).
        
        Args:
            robot_state:    np.array([x, y, theta])
            goal_position:  np.array([x, y])
        
        Returns:
            (v, omega): (0.0, angular velocity)
        """
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        theta = self.wrap_angle(theta_goal - robot_state[2])
        w = np.clip(self.k_theta * theta, -self.w_max, self.w_max)
        return 0.0, w

    def reached_waypoint(self, robot_state, waypoint):
        return np.linalg.norm(waypoint - robot_state[:2]) <= self.dist_thresh

    def reached_orientation(self, robot_state, goal_position):
        theta_goal = np.arctan2(goal_position[1] - robot_state[1],
                                goal_position[0] - robot_state[0])
        return abs(self.wrap_angle(theta_goal - robot_state[2])) <= self.theta_thresh

    def step(self, robot_state, v, w):
        """Integrate one timestep; returns updated robot_state (does not mutate input)."""
        state = robot_state.copy()
        state[0] += self.dt * v * np.cos(state[2])
        state[1] += self.dt * v * np.sin(state[2])
        state[2]  = self.wrap_angle(state[2] + self.dt * w)
        return state
    
    def wrap_angle(self, a: float) -> float:
        """Wrap angle to [-π, π]."""
        return (a + math.pi) % (2 * math.pi) - math.pi
