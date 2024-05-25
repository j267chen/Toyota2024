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
def detectCollision(): 
    msg = robot.checkScan()
    min_dist, min_dist_angle = robot.detect_obstacle(self, msg)
    if min_dist <= MIN_DIST:
        return True
    return False

def stopAndReverse():
    robot.send_cmd_vel(self, 0, 0)
    while (//condition is certain distance from wall):
        robot.send_cmd_vel(self, -1, 0)


#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:

        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)

        #Add looping functionality here
        
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()
