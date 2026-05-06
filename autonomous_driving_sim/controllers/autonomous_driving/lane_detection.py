import numpy as np
import pandas as pd
import cv2 


def hough_transform(image):
    #best performance so far: 10 threshold, 10 minlinegap
    rho = 1
    theta = np.pi/180 
    threshold = 10
    minLineLength = 10
    maxLineLength = 5
    maxLineGap = 500
    return cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)

def region_selection(image):
    mask = np.zeros_like(image)
    
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    rows, cols = image.shape[:2]
    bottom_left = [cols * 0.25, rows * 0.95]
    top_left = [cols * 0.25, rows * 0.8]
    bottom_right = [cols * 0.95, rows * 0.95]
    top_right = [cols * 0.95, rows * 0.8]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def average_slope_intercept(lines):
    """
    Find the slope and intercept of the left and right lanes of each image.
    Parameters:
        lines: output from Hough Transform
    """
    left_lines    = [] #(slope, intercept)
    left_weights  = [] #(length,)
    right_lines   = [] #(slope, intercept)
    right_weights = [] #(length,)
     
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            # calculating slope of a line
            slope = (y2 - y1) / (x2 - x1)
            # calculating intercept of a line
            intercept = y1 - (slope * x1)
            # calculating length of a line
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            # slope of left lane is negative and for right lane slope is positive
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    # 
    left_lane  = np.dot(left_weights,  left_lines) / np.sum(left_weights)  if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane
   
def pixel_points(y1, y2, line):
    """
    Converts the slope and intercept of each line into pixel points.
        Parameters:
            y1: y-value of the line's starting point.
            y2: y-value of the line's end point.
            line: The slope and intercept of the line.
    """
    if line is None:
        return None
    slope, intercept = line
    if slope == 0:  # Avoid division by zero
       slope = 0.001
    #print("Slope: " + str(slope))
    #print("intercept: " + str(intercept))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))
   
def lane_lines(image, lines):
    """
    Create full lenght lines from pixel points.
        Parameters:
            image: The input test image.
            lines: The output lines from Hough Transform.
    """
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = y1 * 0.6
    left_line  = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)
    return left_line, right_line
 
     
def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
    """
    Draw lines onto the input image.
        Parameters:
            image: The input test image (video frame in our case).
            lines: The output lines from Hough Transform.
            color (Default = red): Line color.
            thickness (Default = 12): Line thickness. 
    """
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)


def fit_lane_polynomial(lines, image_shape):
    if lines is None:
        return None

    all_x, all_y = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            all_x.extend([x1, x2])
            all_y.extend([y1, y2])
    
    if len(all_x) < 2:
        return None

    # Fit a 2nd-degree polynomial: y = a*x^2 + b*x + c
    fit = np.polyfit(all_x, all_y, 2)
    return fit


def draw_polyline(image, fit, color=(0, 0, 255), thickness=8):
    if fit is None:
        return image
    
    plot_y = np.linspace(int(image.shape[0]*0.6), image.shape[0]-1, num=100)
    fit_x = fit[0]*plot_y**2 + fit[1]*plot_y + fit[2]
    
    points = np.array([np.transpose(np.vstack([fit_x, plot_y]))], dtype=np.int32)
    cv2.polylines(image, np.int32([points]), isClosed=False, color=color, thickness=thickness)
    return image


def lane_detector(image):
    #convert image to grayscale
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # apply Gaussian Blur to deal with image noise
    blur = cv2.GaussianBlur(grayscale, (5,5), 0)
    
    # canny edge detection
    edges = cv2.Canny(blur, 50, 150)
    
    region = region_selection(edges)
    hough = hough_transform(region)
    if hough is None:
        return image, hough
    
    result = draw_lane_lines(image, lane_lines(image,hough))
    #fit = fit_lane_polynomial(hough, image)
    #result = draw_polyline(image, fit)
    return result, hough