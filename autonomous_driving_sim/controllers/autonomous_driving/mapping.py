import numpy as np
import math
import matplotlib.pyplot as plt

class Mapping:
    def __init__(self, map_size=300, resolution=0.1, maxRange=50.0):
        self.map_size = map_size
        self.resolution = resolution
        self.grid_map = np.zeros((int(map_size/resolution), int(map_size/resolution)))  # Occupancy grid
        self.maxRange = maxRange
        self.state = np.zeros(3)
        self.counter = 0
        #self.lidar = lidar  # Lidar sensor

    def get_lidar_data(self, ranges):
        #if self.lidar is None:
        #    return []

        #ranges = self.lidar.getRangeImage()
        angles = [i * (2 * math.pi / len(ranges)) - math.pi for i in range(len(ranges))]
        points = []

        pose = self.state  # Extract current pose (x, y, theta)
        for i, r in enumerate(ranges):
            if r < self.maxRange:
                x = pose[0] + r * math.cos(pose[2] + angles[i])
                y = pose[1] + r * math.sin(pose[2] + angles[i])
                points.append((x, y))

        return points

    def world_to_map(self, x, y):
        mx = int((x + self.map_size * self.resolution / 2) / self.resolution)
        my = int((y + self.map_size * self.resolution / 2) / self.resolution)
        return mx, my

    def process_pointcloud_data(self, point_cloud):
        lidar_data = []
        for point in point_cloud:
            lidar_data.append((point.x, point.y))
    
        processed_data = []
        for x, y in lidar_data:
            if x != np.inf and y != np.inf:
                processed_data.append((round(x + self.state[0] + 150, 1), round(y + self.state[1] + 150, 1)))
        
        return processed_data
        # Print extracted data
        #for x, y, z, timestamp in lidar_data:
        #    print(f"X: {x}, Y: {y}, Z: {z}, Time: {timestamp}")
    
    def update_map(self, cloud):
        #points = self.get_lidar_data(ranges)
        cloud = self.process_pointcloud_data(cloud)
        for x, y in cloud:
            x_coord = int(x*10)
            y_coord = int(y*10)
            self.grid_map[x_coord, y_coord] = 1
            #mx, my = self.world_to_map(x, y)
            #print("MX: " + str(mx))
            #print("MY: " + str(my))
            #if 0 <= mx < self.map_size and 0 <= my < self.map_size:
            #    self.grid_map[mx, my] = 1  # Mark as occupied
            #    print(f"Mapping point at ({mx}, {my})")

    def get_map(self):
        return self.grid_map
                
    def save_map_image(self, filename="data/slam_map.png"):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid_map, cmap="gray_r", origin="lower")
        plt.colorbar(label="Occupancy")
        plt.xlabel("X (Grid)")
        plt.ylabel("Y (Grid)")
        plt.title("SLAM Occupancy Grid Map")
        plt.savefig(filename)
        plt.close()

    def mapping(self, cloud, state):
        self.state = state
        self.update_map(cloud)
        self.counter += 1
        if self.counter == 500:
            self.save_map_image()
            self.counter = 0
        #self.save_map_image()
