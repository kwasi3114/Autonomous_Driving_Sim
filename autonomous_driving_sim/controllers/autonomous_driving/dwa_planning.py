from dataclasses import dataclass
import math, numpy as np

@dataclass
class DWAConfig:
    # Speed limits
    max_speed: float = 10.0        # m/s (match your vehicle)
    min_speed: float = 0.0         # m/s
    max_steer: float = 0.5         # rad (max steering angle)
    
    # Dynamic constraints
    max_accel: float = 2.0         # m/s^2
    max_steer_rate: float = 0.4    # rad/s
    
    # Simulation
    dt: float = 0.1                # time step (s)
    predict_time: float = 2.5      # how far ahead to simulate (s)
    
    # Sampling resolution
    v_samples: int = 7
    w_samples: int = 15
    
    # Scoring weights (tune these)
    w_heading: float = 1.5         # align with goal
    w_clearance: float = 1.0       # stay away from obstacles
    w_velocity: float = 0.5        # prefer higher speeds
    
    # Safety
    robot_radius: float = 1.2      # vehicle half-width + margin (m)
    collision_radius: float = 1.8  # hard stop distance (m)
    
 
def calc_dynamic_window(v_current, steer_current, config):
    """
    Intersect velocity limits with what's reachable given current
    speed and acceleration constraints.
    """
    # Hard velocity limits
    v_min = config.min_speed
    v_max = config.max_speed
    steer_min = -config.max_steer
    steer_max =  config.max_steer

    # Reachable window this step
    dw_v_min = v_current - config.max_accel * config.dt
    dw_v_max = v_current + config.max_accel * config.dt
    dw_s_min = steer_current - config.max_steer_rate * config.dt
    dw_s_max = steer_current + config.max_steer_rate * config.dt

    return (
        max(v_min, dw_v_min),
        min(v_max, dw_v_max),
        max(steer_min, dw_s_min),
        min(steer_max, dw_s_max)
    )
    

def simulate_trajectory(v, steer, config, wheelbase=2.5):
    """
    Simulate vehicle motion using Ackermann steering model.
    Vehicle starts at local origin (0,0) heading forward (+x).
    Returns list of (x, y, heading) poses.
    """
    trajectory = []
    x, y, theta = 0.0, 0.0, 0.0
    
    steps = int(config.predict_time / config.dt)
    for _ in range(steps):
        # Ackermann: turning radius = wheelbase / tan(steer)
        if abs(steer) > 1e-4:
            turn_radius = wheelbase / math.tan(steer)
            dtheta = v * config.dt / turn_radius
        else:
            dtheta = 0.0
        
        x += v * config.dt * math.cos(theta + dtheta / 2)
        y += v * config.dt * math.sin(theta + dtheta / 2)
        theta += dtheta
        trajectory.append((x, y, theta))
    
    return trajectory
    
    
def dwa_plan(v_current, steer_current, obstacles, goal_local, config):
    """
    Main DWA step. Returns (best_v, best_steer, best_trajectory).
    
    Args:
        v_current:     current vehicle speed (m/s)
        steer_current: current steering angle (rad)
        obstacles:     [(cx, cy, radius), ...] in vehicle-local frame
        goal_local:    (gx, gy) lookahead point in vehicle-local frame
        config:        DWAConfig
    
    Returns:
        (v, steer, trajectory) — best command and its predicted path
    """
    v_min, v_max, s_min, s_max = calc_dynamic_window(
        v_current, steer_current, config
    )
    
    best_score = -float('inf')
    best_v = v_current
    best_steer = 0.0
    best_traj = []
    
    v_range = np.linspace(v_min, v_max, config.v_samples)
    s_range = np.linspace(s_min, s_max, config.w_samples)
    
    for v in v_range:
        for steer in s_range:
            traj = simulate_trajectory(v, steer, config)
            
            # Skip zero-motion trajectories
            if not traj:
                continue
            
            c_clear = score_clearance(traj, obstacles, config)
            if c_clear == 0.0:
                continue  # Collision — skip entirely
            
            c_head = score_heading(traj, goal_local)
            c_vel  = score_velocity(v, config)
            
            score = (config.w_heading   * c_head +
                     config.w_clearance * c_clear +
                     config.w_velocity  * c_vel)
            
            if score > best_score:
                best_score = score
                best_v     = v
                best_steer = steer
                best_traj  = traj
    
    # Fallback: if all trajectories collide, brake
    if not best_traj:
        best_v     = max(0.0, v_current - config.max_accel * config.dt)
        best_steer = 0.0
    
    return best_v, best_steer, best_traj
    

def get_local_goal(global_path, state, lookahead_dist=8.0):
    """
    Find pure pursuit lookahead point and return it in vehicle-local frame.
    """
    vx, vy, heading = state
    
    # Find closest point on path, then step lookahead_dist ahead
    best_idx = min(
        range(len(global_path)),
        key=lambda i: math.hypot(global_path[i][0] - vx, global_path[i][1] - vy)
    )
    
    for i in range(best_idx, len(global_path)):
        wx, wy = global_path[i]
        dist = math.hypot(wx - vx, wy - vy)
        if dist >= lookahead_dist:
            # Transform to local frame
            dx, dy = wx - vx, wy - vy
            lx =  dx * math.cos(heading) + dy * math.sin(heading)
            ly = -dx * math.sin(heading) + dy * math.cos(heading)
            return (lx, ly)
    
    # End of path
    wx, wy = global_path[-1]
    dx, dy = wx - vx, wy - vy
    lx =  dx * math.cos(heading) + dy * math.sin(heading)
    ly = -dx * math.sin(heading) + dy * math.cos(heading)
    return (lx, ly)
    
    
