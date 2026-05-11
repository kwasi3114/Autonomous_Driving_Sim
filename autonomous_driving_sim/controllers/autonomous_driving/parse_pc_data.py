import math
import numpy as np

# process point cloud input to obtain filtered obstacles

def world_to_local(points, state, convention='standard'):
    """
    convention options:
      'standard'  — original (your current version)
      'flip_dy'   — negate dy before rotation
      'flip_heading' — negate heading
      'swap_axes' — swap x/y world axes (for Webots XZ ground plane)
    """
    vx, vy, heading = state
    local_points = []

    for (wx, wy) in points:
        dx = wx - vx
        dy = wy - vy

        if convention == 'standard':
            lx =  dx * math.cos(heading) + dy * math.sin(heading)
            ly = -dx * math.sin(heading) + dy * math.cos(heading)

        elif convention == 'flip_dy':
            lx =  dx * math.cos(heading) - dy * math.sin(heading)
            ly =  dx * math.sin(heading) + dy * math.cos(heading)

        elif convention == 'flip_heading':
            h = -heading
            lx =  dx * math.cos(h) + dy * math.sin(h)
            ly = -dx * math.sin(h) + dy * math.cos(h)

        elif convention == 'swap_axes':
            # treat world (x,y) as (z,x) — Webots ground plane fix
            dx, dy = dy, dx
            lx =  dx * math.cos(heading) + dy * math.sin(heading)
            ly = -dx * math.sin(heading) + dy * math.cos(heading)

        local_points.append((lx, ly))

    return local_points

def filter_local_obstacles(local_points, 
                            max_forward=100.0,   # meters ahead
                            max_behind=0.0,     # small buffer behind
                            max_lateral=100.0,    # ~one lane each side
                            min_dist=0.0):      # ignore points on the car itself
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

def parse_raw_lidar(lidar, state):
    """
    Read LiDAR directly — bypass gmapping.process_pointcloud_data entirely.
    For a 180-degree front-arc LiDAR in Webots.
    """
    vx, vy, heading = state
    
    range_data  = lidar.getRangeImage()   # flat list of distances
    fov         = lidar.getFov()          # total FOV in radians (should be ~pi)
    num_rays    = lidar.getHorizontalResolution()
    
    obstacles_local = []
    
    for i, dist in enumerate(range_data):
        # Skip invalid returns
        if dist >= lidar.getMaxRange() or dist <= 0.1 or math.isinf(dist):
            continue
        
        # Angle of this ray in sensor frame
        # For 180-deg arc: rays go from -fov/2 (right) to +fov/2 (left)
        angle = -fov / 2.0 + i * (fov / (num_rays - 1))
        
        # Hit position in vehicle-local frame (+x forward, +y left)
        lx = dist * math.cos(angle)
        ly = dist * math.sin(angle)
        
        obstacles_local.append((lx, ly))
    
    return obstacles_local


def parse_pc(lidar, state):
    """
    Full pipeline using raw LiDAR ranges.
    """
    # 1. Get points already in local frame from raw LiDAR
    local_pts = parse_raw_lidar(lidar, state)
    
    #print(f"[DEBUG] Raw local points (first 5): {local_pts[:5]}")
    #print(f"[DEBUG] Local X range: {min(p[0] for p in local_pts):.1f} "
          #f"to {max(p[0] for p in local_pts):.1f}")
    
    # 2. Filter — no world_to_local needed, already in local frame
    filtered = filter_local_obstacles(local_pts,
                                       max_forward=20.0,
                                       max_behind=1.0,
                                       max_lateral=4.0,
                                       min_dist=0.5)
    
    #print(f"[DEBUG] Points after filter: {len(filtered)} / {len(local_pts)}")
    
    # 3. Cluster
    obstacles = cluster_points(filtered, cluster_radius=1.0)
    
    #print(f"[DEBUG] Final obstacles: {obstacles}")
    return obstacles
    
 