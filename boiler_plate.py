#Start with imports, ie: import the wrapper
#import other libraries as needed
import TMMC_Wrapper
import rclpy
import numpy as np
import math

#Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

#Specify hardware api
TMMC_Wrapper.use_hardware()
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#Debug messaging 
print("running main")

#start processes
MIN_DIST = 0.3
#add starter functions here
def stopSignDetection(robot):
    """
    Detect stop signs in an image using YOLO object detection.

    Parameters:
    - robot: An object that provides access to image processing and other functionalities.

    Returns:
    - results: Detection results containing information about stop sign locations.
    """
    try:
        # Check the image using the robot's functionality
        picture = robot.checkImage()

        # Convert the image to OpenCV format
        imageToCV = robot.roslmg_to_cv2()

        # Load the YOLO model for object detection
        model = YOLO('yolov8n.pt')

        # Perform object detection using the image
        results = model.predict(imageToCV)

        # Further process the results as needed

        # Return the detection results or perform additional actions
        return results

    except Exception as e:
        print(f"Error in stopSignDetection: {e}")
        return None


def detectCollision(): 
    msg = robot.checkScan()
    min_dist, min_dist_angle = robot.detect_obstacle(self, msg)
    if min_dist <= MIN_DIST:
        return True
    return False

def reverseTurn():
    robot.send_cmd_vel(self, 0, 0)
    msg = robot.checkScan()
    min_dist, min_dist_angle = robot.detect_obstacle(self, msg)
    
    while (min_dist <= MIN_DIST * 3):
        robot.send_cmd_vel(self, -10, 0)
        rclpy.spin_once(robot, timeout_sec=0.05)
        msg = robot.checkScan()
        min_dist, min_dist_angle = robot.detect_obstacle(self, msg)
    robot.rotate(self, 30, 1)

def automatedMove():
    robot.send_cmd_vel(self, 3, 0)
    
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
            reverseTurn()
        
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()
