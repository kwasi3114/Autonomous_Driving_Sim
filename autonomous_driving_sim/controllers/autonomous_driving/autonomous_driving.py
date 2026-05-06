"""autonomous_driving controller."""

#[-112.45720986532739, -45.15309724498789, 0.31736890310890103]
#[51.50372894386022, -62.294313241898045, 0.314506557899614]

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from controller import Display
from vehicle import Car
from vehicle import Driver


#import matlab.engine
import cv2
import time
import numpy as np
import math
from lane_detection import lane_detector
from driving_controller import laneController
from mapping import Mapping
from path_generation import pathGeneration
from path_follower import pathFollower
from pure_pursuit import pure_pursuit_steer
from path_follower_alpha import pathFollowerA

# create the Robot instance.
#robot = Robot()
#car = Car()
#import sys
#print(sys.version)

from controller import Keyboard
#keyboard = Keyboard()
#keyboard.enable(10)   # check for keypress every 50 ms

#keyboard = Robot.getKeyboard()
#keyboard.enable(10)


driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(20)

#robot = Robot()
#keyboard = robot.getKeyboard()
#keyboard = Keyboard()
#keyboard.enable(10)

camera = driver.getDevice("camera")
camera.enable(10)

gps = driver.getDevice("gps")
gps.enable(10)

imu = driver.getDevice("inertial unit")
imu.enable(10)

lidar = driver.getDevice("Sick LMS 291")
lidar.enable(10)
lidar.enablePointCloud()

gyro = driver.getDevice("gyro")
gyro.enable(10)

gmapping = Mapping()

image_path = r"C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving\data\image.jpg"
test_path = r"C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving\data\test.jpg"


#display = Display(driver.getDevice("Display"))
display = driver.getDevice("display")
display.attachCamera(camera)

x_path, y_path = pathGeneration()
path = []

for i in range(len(x_path)):
    #if i != 0:
    path.append([x_path[i], y_path[i]])

path = np.array(path)
#print(path)
target_pos = np.array([x_path[1], y_path[1]])
path_index = 1

prev_error = 0
error_dt = 0

angle_des = 0
steer_angle = 0

theta = 0.0

def getSign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi
    #return (a + 2*np.pi) % (4*np.pi) - 2*np.pi
    #return a+np.pi
    #a = np.asarray(a)
    #a = np.where(a >  2*np.pi, a - 2*np.pi, a)
    #a = np.where(a < -2*np.pi, a + 2*np.pi, a)
    return a

def update_theta(theta, omega_z, dt):
    return theta + omega_z * dt    
#camera = Camera('camera')
#camera.enable(

# get the time step of the current world.
#timestep = int(driver.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while driver.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    image = camera.getImage()
    #print(image)
    
    #if image:
    #camera.saveImage(image_path, 50)
    width = camera.getWidth()
    height = camera.getHeight()
        
    imageArray = np.frombuffer(image, dtype=np.uint8)
    imageArray = imageArray.reshape((height, width, 4))
    #imageBGR = cv2.cvtColor(imageArray, cv2.COLOR_RGBA2BGR)
    lanes, hough = lane_detector(imageArray)
    
    #window_name = 'camera_feed'
    #ret, frame = cap.read()
    #cv2.imshow('video', lanes)
    #print(lanes.dtype)
    #print(lanes.shape)
    #cv2.imwrite(test_path, lanes.astype(np.uint8))
    lanes = lanes.astype(np.uint8).tobytes()
    #cv2.imwrite("test.jpg", lanes)
    #print(lanes)
    
    #im = display.imageNew(image, 6, width, height)
    #display.imagePaste(im,0,0,False)
    #display.setColor(0x0000FF)
    #display.fillOval(1,1,200,200)
    
    #print(gps.getValues())
    pos = gps.getValues()
    rot = imu.getRollPitchYaw()
    state = np.array([pos[0], pos[1], rot[2]])
    
    current_pos = np.array([pos[0], pos[1]])
    heading = rot[2]
    #print("Heading: " + str(heading))
    
    angvel = gyro.getValues()
    yaw_roc = -angvel[2]
    theta = update_theta(theta, yaw_roc, 0.01)
    
    #if heading > 0:
    #    vehi
    #vehicle_yaw = -wrap(heading + np.pi)
    #vehicle_yaw = -wrap(heading - np.pi)
    #vehicle_yaw = wrap(heading)
    
    #if getSign(angle_des) == -1 and heading > 0 and angle_des < -np.pi:
        #print("Vehicle Yaw: " + str(vehicle_yaw))
        #print("Calc: " + str(abs(vehicle_yaw - np.pi)))
    #    if getSign(vehicle_yaw) == 1:
    #        vehicle_yaw = -vehicle_yaw
            
    #    vehicle_yaw = -vehicle_yaw - 2*(abs(vehicle_yaw - np.pi))
        #print("New Yaw: " + str(vehicle_yaw))
    
    #if getSign(angle_des) == -1 and getSign(vehicle_yaw) != -1:
    #    if heading > 0:
    #        vehicle_yaw = -vehicle_yaw - abs(vehicle_yaw - np.pi)
        #vehicle_yaw = (rot[2] + 2*np.pi) % (4*np.pi) - 2*np.pi
    
    #if vehicle_yaw < -3.14:
    #    vehicle_yaw = -vehicle_yaw
    #vehicle_yaw = -(heading + np.pi)
    #print(vehicle_yaw)
    
    #print("Current Position: " + str(current_pos))
    #print("Target Position: " + str(target_pos))
    
    #steeringAngle = pure_pursuit_steer(current_pos, vehicle_yaw, path, 5, 2.93)
    
    #steeringAngle = pathFollowerA(angle_des, vehicle_yaw, 10)
    steeringAngle = pathFollowerA(angle_des, theta, 10)
    
    #if steeringAngle < -0.785:
    #    steeringAngle += 0.785
    #print("Steering Angle: " + str(steeringAngle))
    
    #steeringAngle = laneController(width, hough, 10)
    #steeringAngle, prev_error, sum_error = pathFollower(current_pos, vehicle_yaw, target_pos, prev_error, error_dt, 10)
    #error_dt += sum_error
    
    #key = keyboard.getKey()
    """
    if key == ord('A'):
        steer_angle -= 0.01
    if key == ord('D'):
        steer_angle += 0.01 
    
    driver.setSteeringAngle(steer_angle)
    print("Steer Angle: " + str(steer_angle))
    """
    driver.setSteeringAngle(steeringAngle)
    #driver.setSteeringAngle(0.15*angle_des)
    #print(steeringAngle)
    #if steeringAngle < -1:
    #    driver.setSteeringAngle(-1.0)
    #elif steeringAngle > 1:
    #    driver.setSteeringAngle(1.0)
    #else:
    #    driver.setSteeringAngle(steeringAngle)
    
    
    #im = display.imageNew(lanes, Display.RGBA, width, height)
    #display.imagePaste(im,0,0,False)
    #display.drawLine(x_path[1], y_path[1], x_path[2], y_path[2])
    #display.drawRectangle(path[2], path[3]
    
    dist_to_target = np.linalg.norm(target_pos - current_pos)
    #dy = target_pos[1] - current_pos[1]
    #dx = target_pos[0] - current_pos[0]
    #print("Angle: " + str(math.atan(dy/dx)))
    #print("Distance to Target: " + str(dist_to_target))
    
    if dist_to_target < 5.0:
        print("Waypoint Reached")
        
        if path_index == 26:
            print("Final Destination Reached")
            driver.setCruisingSpeed(0)
            break
        else:
            path_index += 1
            print("PATH INDEX: " + str(path_index))
            target_pos = np.array([x_path[path_index], y_path[path_index]])
            print("New Target Set, Waypoint " + str(path_index))
            dy = target_pos[1] - y_path[path_index-1]
            dx = target_pos[0] - x_path[path_index-1]
            #angle_des = -math.atan(dy/dx)
            angle_des = -(math.atan2(dy,dx) + np.pi)
            #angle_des = wrap(angle_des)
            #if angle_des < -3.14:
                #angle_des += -(math.atan2(dy,dx))
            #    angle_des = 0.00
            #print("Desired Angle: " + str(angle_des))
            
            #if path_index == 25:
            #    print("Final Destination Reached")
            #    driver.setCruisingSpeed(0)
            #    break
            

            
        #driver.setCruisingSpeed(0)
        #break
    
    #print(gps.getValues())
    #pos = gps.getValues()
    #rot = imu.getRollPitchYaw()
    #state = np.array([pos[0], pos[1], rot[2]])
    
    #current_pos = np.array([pos[0], pos[1]])
    #distance = 
    
    #if 
    #print(state)
    
    #ranges = lidar.getRangeImage()
    point_cloud = lidar.getPointCloud()
    #print(point_cloud)
    gmapping.mapping(point_cloud, state)
    points = gmapping.process_pointcloud_data(point_cloud)
    print(points)

    # Extract useful data
    #lidar_data = []
    #for point in point_cloud:
        #print(point.x)
    #    lidar_data.append((point.x, point.y))
    
    # Print extracted data
    #nparr = np.array(lidar_data)
    #print(nparr.shape)
    #print(lidar_data[0])
    #for x, y, z, timestamp in lidar_data:
        #print(f"X: {x}, Y: {y}, Z: {z}, Time: {timestamp}")
    #cloud = lidar.getPointCloud()
    #print(cloud)
    #print(ranges)
    #maxRange = lidar.getMaxRange()
    #gmapping.mapping(ranges, state)
    #is_all_zeros = np.all(gmapping.get_map() == 0)
    #print(is_all_zeros)
    #if is_all_zeros == False:
    #    print(is_all_zeros)
    #print(gmapping.get_map())
        
        
        #window_name = 'camera_feed'
        #cv_img = cv2.imread(image_path)
        #time.sleep(0.01)
        #cv2.imshow(window_name, imageBGR)
        #cv_img = cv2.imread(image_path)
        #cv2.imshow(window_name, imageBGR)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
