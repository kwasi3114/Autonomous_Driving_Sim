import numpy as np
import warnings

warnings.filterwarnings("ignore")

def transform_to_local(dx, dy, theta):
    # Rotate global error into vehicle coordinate frame
    local_x = dx * np.cos(theta) + dy * np.sin(theta)
    local_y = - dx * np.sin(theta) + dy * np.cos(theta)
    return local_x, local_y


def compute(error, Kp, Ki, Kd, prev_error, sum_error, dt):
    integral = sum_error
    derivative = (error - prev_error) / dt
    output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    #self.prev_error = error
    #output = Kp * error
    return output

def pathFollower(current_pos, heading, target_pos, prev_error, sum_error, dt):
    # 1. Compute global position error
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]

    # 2. Convert error into car's local frame
    local_x, local_y = transform_to_local(dx, dy, heading)
    #print("Local Y: " + str(local_y))
    #print("DY: " + str(dy))

    # local_y = lateral offset → controlled by steering
    #print(dy)
    lateral_error = local_y
    #lateral_error = (dx+dy)/2
    #print("Error: " + str(lateral_error))

    # 3. Compute steering using PID
    steering_angle = compute(lateral_error, 2, 1.2, 0.2, prev_error, sum_error, dt)

    # clamp output if necessary
    #steering_angle = np.clip(steering_angle, -1, 1)
    
    if steering_angle < -0.8:
        return -0.8, lateral_error, lateral_error*dt
    elif steering_angle > 0.8:
        return 0.8, lateral_error, lateral_error*dt
    else:
        return steering_angle, lateral_error, lateral_error*dt

    return steering_angle, lateral_error, lateral_error*dt

