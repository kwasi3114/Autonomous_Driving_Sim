import numpy as np
import math

class EKFLocalizer:
    def __init__(self, dt, initial_state=None):
        """
        State vector: [x, y, theta, v]
          x, y   — position (m) in world frame
          theta  — heading (rad), 0 = East, pi/2 = North
          v      — forward speed (m/s)
        
        Control input u: [a, omega]
          a      — linear acceleration from IMU (m/s^2)
          omega  — yaw rate from IMU (rad/s)
        
        Measurement z: [gps_x, gps_y, compass_theta]
        """
        self.dt = dt
        
        # State estimate
        self.x = np.zeros(4) if initial_state is None else np.array(initial_state, dtype=float)
        
        # --- Covariance matrices (tune these for your setup) ---
        
        # P: Initial state uncertainty
        # Large = "I don't know where I am yet"
        self.P = np.diag([5.0, 5.0, 0.5, 2.0])
        
        # Q: Process noise — how much we distrust the motion model
        # Larger = prediction less trusted, filter reacts faster to measurements
        self.Q = np.diag([
            0.05,   # x position drift
            0.05,   # y position drift
            0.01,   # heading drift
            0.5     # velocity drift (IMU accel is noisy)
        ])
        
        # R: Measurement noise — how much we distrust sensors
        # Match your actual sensor accuracy
        self.R = np.diag([
            1.0,    # GPS x (meters^2) — ~1m GPS accuracy
            1.0,    # GPS y
            0.05    # compass heading (rad^2) — ~13 deg accuracy
        ])

    # ------------------------------------------------------------------
    def predict(self, u):
        """
        Predict step: propagate state forward using motion model.
        u = [a, omega]  (acceleration m/s^2, yaw rate rad/s)
        """
        x, y, theta, v = self.x
        a, omega = u
        dt = self.dt

        # --- Nonlinear state transition f(x, u) ---
        # Use midpoint theta for more accurate integration
        theta_mid = theta + 0.5 * omega * dt

        x_new     = x + v * np.cos(theta_mid) * dt
        y_new     = y + v * np.sin(theta_mid) * dt
        theta_new = theta + omega * dt
        v_new     = v + a * dt

        # Clamp velocity — vehicle can't go backwards here (adjust if needed)
        v_new = max(0.0, v_new)

        # Wrap heading to [-pi, pi]
        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi

        self.x = np.array([x_new, y_new, theta_new, v_new])

        # --- Jacobian of f w.r.t state (linearization at current state) ---
        # Partial derivatives of each state equation
        F = np.array([
            [1, 0, -v * np.sin(theta_mid) * dt,  np.cos(theta_mid) * dt],
            [0, 1,  v * np.cos(theta_mid) * dt,  np.sin(theta_mid) * dt],
            [0, 0,  1,                            0                     ],
            [0, 0,  0,                            1                     ]
        ])

        # Propagate covariance — uncertainty grows during prediction
        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    def update(self, z):
        """
        Update step: correct prediction using sensor measurement.
        z = [gps_x, gps_y, compass_theta]
        """
        # H: Measurement matrix — maps state space to measurement space
        # We directly observe x, y, theta (not v)
        H = np.array([
            [1, 0, 0, 0],   # observe x
            [0, 1, 0, 0],   # observe y
            [0, 0, 1, 0],   # observe theta
        ])

        # --- Innovation (residual): difference between measurement and prediction ---
        y_raw = z - H @ self.x

        # CRITICAL: wrap the heading residual to [-pi, pi]
        # Without this, a jump from -3.1 to +3.1 rad causes a huge false correction
        y_raw[2] = (y_raw[2] + np.pi) % (2 * np.pi) - np.pi

        # --- Innovation covariance ---
        S = H @ self.P @ H.T + self.R

        # --- Kalman gain: how much to trust measurement vs prediction ---
        # K large = trust measurement more; K small = trust prediction more
        # Use solve instead of inv for numerical stability
        K = self.P @ H.T @ np.linalg.solve(S.T, np.eye(S.shape[0])).T

        # --- State update ---
        self.x = self.x + K @ y_raw

        # Wrap heading again after update
        self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi

        # --- Covariance update (Joseph form — numerically stable) ---
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    # ------------------------------------------------------------------
    def get_state(self):
        """Return current state estimate as (x, y, theta, v)"""
        return tuple(self.x)