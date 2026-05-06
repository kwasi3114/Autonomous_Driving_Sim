def pathFollowerA(desired, current, dt):
    #print("Desired: " + str(desired) + ", Current: " + str(current))
    error = desired - current
    #error = current - desired
    #print("Error: " + str(error))
    Kp = 0.55
    return Kp*error