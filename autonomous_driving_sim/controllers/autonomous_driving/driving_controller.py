import numpy as np

def getLaneCenter(hough, carCenter):
    laneMarkers = []
    for line in hough:
        x1, y1, x2, y2 = line[0]
        if y1 < y2:
            laneMarkers.append(x2)
        else:
            laneMarkers.append(x1)
    
    if len(hough) > 1:
        return (laneMarkers[0] + laneMarkers[1])/2
    else:
        if laneMarkers[0] < carCenter:
            return laneMarkers[0] + 25
        else:
            return laneMarkers[0] - 25

def laneController(width, hough, dt):
    if hough is None:
        return 0.0
    
    carCenter = width/2
    laneCenter = getLaneCenter(hough, carCenter)
    error = laneCenter - carCenter
    Kp = 0.05
    return Kp*error
    