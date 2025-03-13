"""autonomous_driving controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera
from controller import Display
from vehicle import Car
from vehicle import Driver


import matlab.engine
import cv2
import time
import numpy as np

# create the Robot instance.
#robot = Robot()
#car = Car()
#import sys
#print(sys.version)

driver = Driver()
driver.setSteeringAngle(-0.05)
driver.setCruisingSpeed(20)

#camera = driver.getDevice("camera")
#camera.enable(10)

#image_path = r"C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving\data\image.jpg"

#display = Display(driver.getDevice("Display"))
#display = driver.getDevice("display")
#display.attachCamera(camera)

eng = matlab.engine.connect_matlab()
#model_path = r'C:\Users\kwasi\Documents\autonomous_driving_sim\controllers\autonomous_driving_matlab\vehicle_controller.slx'
eng.open_system('vehicle_controller', nargout=0)
eng.set_param('vehicle_controller', 'SimulationCommand', 'start', nargout=0)


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
    #image = camera.getImage()
    #if image:
    #camera.saveImage(image_path, 50)
    #width = camera.getWidth()
    #height = camera.getHeight()
        
    #imageArray = np.frombuffer(image, dtype=np.uint8)
    #imageArray = imageArray.reshape((height, width, 4))
    #imageBGR = cv2.cvtColor(imageArray, cv2.COLOR_RGBA2BGR)
    
    #im = display.imageNew(image, 6, width, height)
    #display.imagePaste(im,0,0,False)
    
    eng.set_param('vehicle_controller', 'SimulationCommand', 'step', nargout=0)
    
        
        
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
