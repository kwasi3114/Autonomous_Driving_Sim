import math
import numpy as np

def world_to_local(dx, dy, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    local_x = c*dx + s*dy
    local_y = -s*dx + c*dy
    return local_x, local_y

def pure_pursuit_steer(current_pos, yaw, path, lookahead_dist, wheelbase):
    # Find lookahead point
    for px, py in path:
        dx = px - current_pos[0]
        dy = py - current_pos[1]
        if dx*dx + dy*dy >= lookahead_dist**2:
            lx, ly = world_to_local(dx, dy, yaw)
            break
    else:
        return 0.0  # reached end of path

    # curvature = 2*y / L^2
    k = 2 * ly / (lookahead_dist**2)

    # steering = atan(k * wheelbase)
    steer = math.atan(k * wheelbase)
    #print(steer)

    return steer
