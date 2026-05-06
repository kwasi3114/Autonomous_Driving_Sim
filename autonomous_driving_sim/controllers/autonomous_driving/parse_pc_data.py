import math
import numpy as np

# process point cloud input to obtain filtered obstacles

def world_to_local(points, state):
    """
    Transform world-frame (x, y) points into vehicle-local frame.
    state = (x, y, heading_rad)
    Returns list of (local_x, local_y) where +x is forward, +y is left
    """
    vx, vy, heading = state
    local_points = []
    
    for (wx, wy) in points:
        dx = wx - vx
        dy = wy - vy
        # Rotate into vehicle frame
        lx =  dx * math.cos(heading) + dy * math.sin(heading)
        ly = -dx * math.sin(heading) + dy * math.cos(heading)
        local_points.append((lx, ly))
    
    return local_points

def filter_local_obstacles(local_points, 
                            max_forward=20.0,   # meters ahead
                            max_behind=2.0,     # small buffer behind
                            max_lateral=4.0,    # ~one lane each side
                            min_dist=0.5):      # ignore points on the car itself
    """
    Keep only points in a forward-facing corridor around the vehicle.
    local frame: +x = forward, +y = left
    """
    filtered = []
    for (lx, ly) in local_points:
        dist = math.hypot(lx, ly)
        if (lx > -max_behind and          # not too far behind
            lx < max_forward and           # not too far ahead
            abs(ly) < max_lateral and      # within road corridor
            dist > min_dist):              # not the car itself
            filtered.append((lx, ly))
    return filtered
    

def cluster_points(local_points, cluster_radius=1.5):
    """
    Simple greedy clustering. Returns list of (centroid_x, centroid_y, radius).
    """
    if not local_points:
        return []
    
    points = list(local_points)
    clusters = []
    used = set()
    
    for i, p in enumerate(points):
        if i in used:
            continue
        cluster = [p]
        used.add(i)
        for j, q in enumerate(points):
            if j not in used and math.dist(p, q) < cluster_radius:
                cluster.append(q)
                used.add(j)
        
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        # Estimate obstacle radius from cluster spread
        radius = max(math.dist((cx, cy), c) for c in cluster)
        radius = max(radius, 0.5)  # minimum 0.5m radius
        clusters.append((cx, cy, radius))
    
    return clusters

def parse_pc_data(point_cloud, state):
    """
    Full pipeline: raw world-frame point cloud → DWA-ready obstacle list
    """
    # 1. Transform to local frame
    local_pts = world_to_local(point_cloud, state)
    
    # 2. Filter to road corridor
    local_pts = filter_local_obstacles(local_pts,
                                        max_forward=20.0,
                                        max_lateral=4.0)
    
    # 3. Cluster into obstacles
    obstacles = cluster_points(local_pts, cluster_radius=1.5)
    
    return obstacles  # [(cx, cy, radius), ...]
    
 