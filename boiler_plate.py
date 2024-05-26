#Start with imports, ie: import the wrapper
#import other libraries as needed
import time
import TMMC_Wrapper
import rclpy
import numpy as np
import math

#Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = True
if not TMMC_Wrapper.is_SIM:
    #Specify hardware api
    TMMC_Wrapper.use_hardware()

if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#Debug messaging 
print("running main")

#start processes
MIN_DIST = 0.5
#add starter functions here
def stopSignDetection(robot):
    try:
        # Check the image using the robot's functionality
        picture = robot.checkImage()

        # Convert the image to OpenCV format
        imageToCV = robot.roslmg_to_cv2()

        # Load the YOLO model for object detection
        model = YOLO('yolov8n.pt')

        # Perform prediction of stop signs using the ML_predict_stop_sign function
        stop_sign_detected, x1, y1, x2, y2 = ML_predict_stop_sign(model, imageToCV)

        # Further process the detection results as needed

        # Return the detection results or perform additional actions

        return stop_sign_detected, x1, y1, x2, y2

    except Exception as e:
        print(f"Error in stopSignDetection: {e}")
        return None


def detectCollision(): 
    msg = robot.checkScan()
    min_dist, min_dist_angle = robot.detect_obstacle(msg)
    print("Test1")
    if min_dist <= MIN_DIST:
        print("Collision detected")
        return True
    return False

def reverseTurn():
    robot.send_cmd_vel(0, 0)
    msg = robot.checkScan()
    min_dist, min_dist_angle = robot.detect_obstacle(msg)
    
    while (min_dist <= MIN_DIST * 3):
        robot.send_cmd_vel(-1, 0)
        rclpy.spin_once(robot, timeout_sec=0.1)
        msg = robot.checkScan()
        min_dist, min_dist_angle = robot.detect_obstacle(msg)
    robot.rotate(30, 1)

def automatedMove():
    robot.send_cmd_vel(1, 0)
    print("Moving")
    
#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:

        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)

        #Add looping functionality here
        automatedMove()

        if detectCollision():
            print("Working correctly")
            reverseTurn()

        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()