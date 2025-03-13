"""autonomous_driving controller."""

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
from lane_detection import lane_detector
from driving_controller import laneController

# create the Robot instance.
#robot = Robot()
#car = Car()
#import sys
#print(sys.version)

driver = Driver()
driver.setSteeringAngle(0)
driver.setCruisingSpeed(20)

camera = driver.getDevice("camera")
camera.enable(10)

#gps = driver.getDevice("gps")

lidar = driver.getDevice("Sick LMS 291")
lidar.enable(10)

image_path = r"C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving\data\image.jpg"
test_path = r"C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving\data\test.jpg"


#display = Display(driver.getDevice("Display"))
display = driver.getDevice("display")
display.attachCamera(camera)

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
    
    steeringAngle = laneController(width, hough, 10)
    #print(steeringAngle)
    driver.setSteeringAngle(steeringAngle)
    
    im = display.imageNew(lanes, Display.RGBA, width, height)
    display.imagePaste(im,0,0,False)
    
        
        
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
