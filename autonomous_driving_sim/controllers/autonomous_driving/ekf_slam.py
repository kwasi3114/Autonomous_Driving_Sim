import numpy as np
import math

class EKF_SLAM:
    def __init__(self, map_size=100, resolution=0.1, lidar=None):
        self.dt = 0.1  # Time step
        self.x = np.zeros(4)  # [x, y, theta, v]
        self.P = np.eye(4) * 0.1  # Covariance matrix
        self.Q = np.eye(4) * 0.01  # Process noise
        self.R = np.diag([0.5, 0.5, 0.1])  # Measurement noise (GPS, IMU)
        self.map_size = map_size
        self.resolution = resolution
        self.grid_map = np.zeros((map_size, map_size))  # Occupancy grid
        #self.lidar = lidar  # Lidar sensor

    def predict(self, u):
        x, y, theta, v = self.x
        a, omega = u  # Control inputs from IMU

        # Nonlinear state update
        self.x[0] += v * np.cos(theta) * self.dt
        self.x[1] += v * np.sin(theta) * self.dt
        self.x[2] += omega * self.dt
        self.x[3] += a * self.dt

        # Normalize theta to keep it within [-pi, pi]
        self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi

        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt, np.cos(theta) * self.dt],
            [0, 1, v * np.cos(theta) * self.dt, np.sin(theta) * self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # Measurement model
        H = np.array([
            [1, 0, 0, 0],  # GPS x
            [0, 1, 0, 0],  # GPS y
            [0, 0, 1, 0]   # IMU theta
        ])

        # Innovation
        y = z - H @ self.x

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        self.update_map()

    def get_lidar_data(self, ranges):
        #if self.lidar is None:
        #    return []

        #ranges = self.lidar.getRangeImage()
        angles = [i * (2 * math.pi / len(ranges)) - math.pi for i in range(len(ranges))]
        points = []

        pose = self.x[:3]  # Extract current pose (x, y, theta)
        for i, r in enumerate(ranges):
            if r < self.lidar.getMaxRange():
                x = pose[0] + r * math.cos(pose[2] + angles[i])
                y = pose[1] + r * math.sin(pose[2] + angles[i])
                points.append((x, y))

        return points

    def world_to_map(self, x, y):
        mx = int((x + self.map_size * self.resolution / 2) / self.resolution)
        my = int((y + self.map_size * self.resolution / 2) / self.resolution)
        return mx, my

    def update_map(self):
        points = self.get_lidar_data()
        for x, y in points:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                self.grid_map[mx, my] = 1  # Mark as occupied

    def save_map_image(self, filename="slam_map.png"):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid_map, cmap="gray_r", origin="lower")
        plt.colorbar(label="Occupancy")
        plt.xlabel("X (Grid)")
        plt.ylabel("Y (Grid)")
        plt.title("SLAM Occupancy Grid Map")
        plt.savefig(filename)
        plt.close()

    def SLAM(self, u, z):
        self.predict(u)
        self.update(z)
        self.save_map_image()
