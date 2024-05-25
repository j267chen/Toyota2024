#-----------------
#---Features we want---
#---    - subscribe and publish to cmd vel  (done)
#---    - setting max velocity (done)
#---    - stopping at stop sign (camera integration from allyn)
#---    - WASD control (done)
#---    - dock and undock  (done)
#---    - simulator (in progress)
#---    - sensor values (done)
#-----------------

def use_hardware():
    global is_SIM



#-----imports-----
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.action import Dock,Undock
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.srv import ResetPose
import time
import rclpy
import tf2_ros
import tf2_ros.buffer
import tf2_ros.transform_listener
import eigenpy
import numpy as np
import os
import subprocess
from pynput.keyboard import Listener
import math

#---imports for vision---
import cv2
import numpy as numpy
import matplotlib.pyplot as plt
# import apriltag

#---constants---
CONST_speed_control = 0.5 #set this to 1 for full speed, 0.5 for half speed
DEBUG = False #set to false to disable terminal printing of some functions

#not sure if we need , modify later, seems like an init thing
def use_hardware():
    # import ROS settings for working locally or with the robot (equivalent of ros_local/ros_robot in the shell)
    env_file = ".env_ros_robot"
    os.environ.update(dict([l.strip().split("=") for l in filter(lambda x: len(x.strip())>0,open(env_file).readlines())]))
    try:
        output = subprocess.check_output("ip addr show",shell=True)
        import re
        robot_id = int(re.search(r"tap[0-9]\.([0-9]+)@tap",output.decode()).groups()[0])
    except Exception as ex:
        raise Exception("VPN does not seem to be running, did you start it?:",ex)
    print("You are connected to uwbot-{:02d}".format(robot_id))
    try:
        subprocess.check_call("ping -c 1 -w 10 192.168.186.3",shell=True,stdout=subprocess.DEVNULL)
    except Exception as ex:
        raise Exception("Could not ping robot (192.168.186.3)")
    print("Robot is reachable")
    try:
        subprocess.check_call("ros2 topic echo --once /ip",shell=True,stdout=subprocess.DEVNULL)
    except Exception as ex:
        print("ros2 topic echo --once /ip failed. Proceed with caution.")
    print("ros2 topic subscription working. Everything is working as expected.")
    

class Robot(Node):
    def __init__(self):
        super().__init__('notebook_wrapper')
        # Create custom qos profile to make subscribers time out faster once notebook
        import rclpy.qos
        import rclpy.time
        from copy import copy
        qos_profile_sensor_data = copy(rclpy.qos.qos_profile_sensor_data)
        qos_policy = copy(rclpy.qos.qos_profile_sensor_data)
        #qos_policy.liveliness = rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC
        #qos_policy.liveliness_lease_duration = rclpy.time.Duration(seconds=10)

        self.last_scan_msg = None
        self.last_imu_msg = None

        self.scan_future = rclpy.Future()
        self.scan_subscription = self.create_subscription(LaserScan,'/scan',self.scan_listener_callback,qos_policy)
        self.scan_subscription  # prevent unused variable warning
        
        self.imu_future = rclpy.Future()
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_listener_callback,qos_profile_sensor_data)
        self.imu_subscription  # prevent unused variable warning
        
        self.image_future = rclpy.Future()
        self.image_subscription = self.create_subscription(Image,'/oakd/rgb/preview/image_raw',self.image_listener_callback,qos_profile_sensor_data)
        self.image_subscription  # prevent unused variable warning
        
        self.camera_info_future = rclpy.Future()
        self.camera_info_subscription = self.create_subscription(CameraInfo,'/oakd/rgb/preview/camera_info',self.camera_info_listener_callback,qos_profile_sensor_data)
        self.camera_info_subscription  # prevent unused variable warning
        
        self.battery_state_future = rclpy.Future()
        self.battery_state_subscription = self.create_subscription(BatteryState,'/battery_state',self.battery_state_listener_callback,qos_profile_sensor_data)
        self.battery_state_subscription  # prevent unused variable warning
                
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.dock_client.wait_for_server()
        self.undock_client.wait_for_server()
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.logging_topics = ["/tf","/tf_static","/scan","/odom"]
        
        self._reset_pose_client = self.create_client(ResetPose, '/reset_pose')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keyboard_listener = None #temp placeholder for the keyboard listener
    
    def get_tf_transform(self,parent_frame,child_frame,wait=True,time_in=rclpy.time.Time()):
        if wait:
            myfuture = self.tf_buffer.wait_for_transform_async(parent_frame,child_frame,time_in)
            self.spin_until_future_completed(myfuture)
        t = self.tf_buffer.lookup_transform(parent_frame,child_frame,time_in).transform
        q = eigenpy.Quaternion(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z)
        R = q.toRotationMatrix()
        v = numpy.array([[t.translation.x,t.translation.y,t.translation.z,1]]).T
        T = numpy.hstack([numpy.vstack([R,numpy.zeros((1,3))]),v])
        return T
    
    def reduce_transform_to_2D(self,transform_3D):
        return transform_3D[(0,1,3),:][:,(0,1,3)]
    
    def rotation_from_transform(self, transform_2D):
        import numpy
        fake_R3D = numpy.eye(3)
        fake_R3D[0:2,0:2] = transform_2D[0:2,0:2]
        import eigenpy
        aa = eigenpy.AngleAxis(fake_R3D)
        return aa.angle
   
    def configure_logging(self,topics):
        self.logging_topics = topics
            
    def start_logging(self):
        if hasattr(self,'logging_instance'):
            raise Exception("logging already active")
        self.logging_dir = bag_dir = '/tmp/notebook_bag_'+str(int(time.time()))
        self.logging_instance = subprocess.Popen("ros2 bag record -s mcap --output "+self.logging_dir+" "+' '.join(self.logging_topics)+" > /tmp/ros2_bag.log 2>&1",shell=True,stdout=subprocess.PIPE,preexec_fn=os.setsid)
        # Wait until topics are subscribed
        # TODO: check log for this
        time.sleep(5)
        
    def stop_logging(self):
        import signal
        os.killpg(os.getpgid(self.logging_instance.pid), signal.SIGINT)
        self.logging_instance.wait()
        del self.logging_instance
        return self.logging_dir
            
    def get_logging_data(self, logging_dir):
        # get log data
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=logging_dir,storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('','')
        reader.open(storage_options,converter_options)
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        log_content = dict()
        while reader.has_next():
            (topic,data,t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            if topic not in log_content.keys():
                log_content[topic] = []
            log_content[topic].append((t,msg))
        return log_content
    
    def delete_logging_data(self, logging_dir):
        import shutil
        shutil.rmtree(logging_dir)

#-----scan listeners and grabbers-----    
    def scan_listener_callback(self, msg):
        self.last_scan_msg = msg
        if DEBUG == True:
            print(f"Laserscan data recieved: Range - {msg.ranges[:5]}")
        self.scan_future.set_result(msg)
        self.scan_future.done()

    def checkScan(self):
        self.scan_future = rclpy.Future()
        self.spin_until_future_completed(self.scan_future)
        return self.last_scan_msg
        
#-----imu listeners and grabbers-----
    def imu_listener_callback(self, msg):
        self.last_imu_msg = msg
        if DEBUG == True:
            print(f"IMU Data recieved: orientation - {msg.orientation}")
        self.imu_future.set_result(msg)
        self.imu_future.done()
        
    def checkImu(self):
        self.imu_future = rclpy.Future()
        self.spin_until_future_completed(self.imu_future)
        return self.last_imu_msg
    
#-----image listeners and grabbers-----     
    def image_listener_callback(self, msg):
        self.last_image_msg = msg
        self.image_future.set_result(msg)
        self.image_future.done()
    
    def checkImage(self):
        self.image_future = rclpy.Future()
        self.spin_until_future_completed(self.image_future)
        return self.last_image_msg

    def checkImageRelease(self): #this one returns an actual image instead of all the data
        image = self.checkImage()
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        cv2.imshow("image", img_3D)
        cv2.waitKey(0)

        
#-----camera listeners and grabbers-----  
    def camera_info_listener_callback(self, msg):
        self.last_camera_info_msg = msg
        self.camera_info_future.set_result(msg)
        self.camera_info_future.done()

    def checkCamera(self):
        self.camera_info_future = rclpy.Future()
        self.spin_until_future_completed(self.camera_info_future)
        return self.last_camera_info_msg 
        # ^ this might have more data, test this

#-----battery listeners and grabbers-----   
    def battery_state_listener_callback(self, msg):
        self.last_battery_state_msg = msg
        self.battery_state_future.set_result(msg)
        self.battery_state_future.done()
    
    def checkBattery(self):
        self.battery_state_future = rclpy.Future()
        self.spin_until_future_completed(self.battery_state_future)
        return self.last_battery_state_msg.percentage
        
    def cmd_vel_timer_callback(self):
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate and self.cmd_vel_stop:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.cmd_vel_publisher.publish(msg)
    
            
    def set_cmd_vel(self, velocity_x, velocity_phi, duration, stop=True):
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        self.cmd_vel_stop = stop
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self,self.cmd_vel_future)
        
        
    def spin_until_future_completed(self,future):
        rclpy.spin_until_future_complete(self,future)
        return future.result()
    
    def undock(self):
        # does not wait until finished
        action_completed_future = rclpy.Future()
        def result_cb(future):
            result = future.result().result
            action_completed_future.set_result(result)
            action_completed_future.done()
        goal_received_future = self.undock_client.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self,goal_received_future)
        goal_handle = goal_received_future.result()
        if not goal_handle.accepted:
            raise Exception('Goal rejected')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(result_cb)
        rclpy.spin_until_future_complete(self,action_completed_future)
        return action_completed_future.result()
        
    def dock(self):
        action_completed_future = rclpy.Future()
        def result_cb(future):
            result = future.result().result
            action_completed_future.set_result(result)
            action_completed_future.done()
        goal_received_future = self.dock_client.send_goal_async(Dock.Goal())
        rclpy.spin_until_future_complete(self,goal_received_future)
        goal_handle = goal_received_future.result()
        if not goal_handle.accepted:
            raise Exception('Goal rejected')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(result_cb)
        rclpy.spin_until_future_complete(self,action_completed_future)
        return action_completed_future.result()

#----some functions for telop control----

    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
    
    def start_keyboard_control(self):
        if self.keyboard_listener is None:
            def on_press(key):
                try:
                    print(f"Key {key.char} pressed")
                    key_char = key.char
                except AttributeError:
                    print(f"Special key {key} pressed")
                    key_char = str(key) #---the below cluster of if statements can be removed to make level one more challenging---
                if key_char == 'w':
                    self.move_forward()
                if key_char == 's':
                    self.move_backward()
                if key_char == 'a':
                    self.turn_left()
                if key_char == 'd':
                    self.turn_right() 
            def on_release(key):
                print("Key released")
            self.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.keyboard_listener.start()
        else:
            print("Keyboard listener already running")

    def stop_keyboard_control(self):
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()
            self.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

    def on_press(self, key):
        try:
            if hasstr(key, 'char') and key.char in self.action_mape:
                self.action_map[key.char]()
        except:
            pass

    def move_forward(self):
        self.send_cmd_vel(1.0*CONST_speed_control, 0.0)
    
    def move_backward(self):
        self.send_cmd_vel(-1.0*CONST_speed_control, 0.0)
    
    def turn_left(self):
        self.send_cmd_vel(0.0, 1.0*CONST_speed_control)

    def turn_right(self):
        self.send_cmd_vel(0.0, -1.0*CONST_speed_control)

    def lidar_data_too_close(self, scan, th1, th2, min_dist):
        #returns points between angles th1, th2 that are closer tha min_dist
        if th2 < th1:
            temp = th1
            th1 = th2 
            th2 = temp 
        th1 = max(th1, scan.angle_min)
        th2 = min(th2, scan.angle_max)
        ind_start = int((th1-scan.angle_min)/scan.angle_increment)
        ind_end = int((th2-scan.angle_min)/scan.angle_increment)
        meas = scan.ranges[ind_start:ind_end]
        total = len(meas)
        meas = [m for m in meas if np.isfinite(m)]
        print(meas)
        print(len(meas))
        if len(meas) == 0:
            return 0.0
        num_too_close = 0.0
        for m in meas: 
            if m < min_dist: 
                print(f"m < min dist addition is {m}")
                num_too_close = num_too_close + 1
        print(float(num_too_close))
        #print(float(num_too_close) / total)

            
        return float(num_too_close) / total


    def get_lidar_obstacle_avoidance_feeder(self):
        #print("starting integrated lidar and keyboard control")
        def on_press(key):
            try:
                key_char = key.char 
            except AttributeError:
                key_char = str(key) #handling special keys
            if key_char == 'w':
                move_forward()
            elif key_char == 's':
                move_backward()
            elif key_char == 'a':
                turn_left()
            elif key_char == 'd':
                turn_right()

        def update_speed_based_on_lidar(base_speed):
            lidar = self.checkScan()
            front_data = self.lidar_data_too_close(lidar, np.radians(45), np.radians(135), 1.00)
            print(f"calculated front data is:{front_data}")
            speed_scale = min(0.0, 1-front_data) #if front_data > 0 else 1.0
            print(f"calculated speed scaler is: {base_speed*speed_scale}")
            return base_speed * speed_scale

        def move_forward():
            ranges = self.last_scan_msg.ranges
            num_ranges = len(ranges)
            print(f"num ranges is found to be: {num_ranges}")
            quarter_segment = num_ranges // 4
            degrees_per_range = 360 / num_ranges
            start_index = int(60 / degrees_per_range)
            end_index = int(120 / degrees_per_range)
            
            def analyze_segment(segment):
                sorted_segment = sorted(segment)
                unique_distances = []
                last_added = None
                total = 0

                for distance in sorted_segment: 
                    if (last_added is None or abs(distance - last_added) > 0.15) and distance > 0.35:
                        unique_distances.append(distance)
                        last_added = distance 
                        if len(unique_distances) >=5:
                            break
                    total += distance
                average_distance = total / len(segment)
                return unique_distances

            front_distances = analyze_segment(ranges[start_index:end_index])
            
            def back_away():
                reverse_speed = -0.5*CONST_speed_control
                reverse_time = 1
                self.send_cmd_vel(reverse_speed, 0.0)
                time.sleep(reverse_time)
                self.send_cmd_vel(0.0,0.0)

            if front_distances:
                closest_object = min(front_distances)
                print(f"The closest object rn is: {closest_object}")
                if closest_object < 0.4:
                    print("Object within 40cm, will not move now")
                    self.send_cmd_vel(0.0,0.0)
                    time.sleep(0.3) #300 ms 
                    back_away()
                    return 
                elif closest_object < 1.0:
                    speed_scale = (closest_object - 0.40) / (1.0-0.40)
                    speed = max(0.1, 1.0*CONST_speed_control * speed_scale)
                    self.send_cmd_vel(speed, 0.0)
                    return 
                
            self.send_cmd_vel(0.0*CONST_speed_control, 0.0)

        def move_backward():
            self.send_cmd_vel(-1.0*CONST_speed_control, 0.0)
        def turn_left():
            self.send_cmd_vel(0.0, 1.0*CONST_speed_control)
        def turn_right():
            self.send_cmd_vel(0.0, -1.0*CONST_speed_control)
        
        listener = Listener(on_press=on_press)
        listener.start()


    def test_lidar_orientation(self):
       #---this was used to find the front heading of the robot, should not be used in solutions
        ranges = self.last_scan_msg.ranges
        num_ranges = len(ranges)
        quarter_segment = num_ranges // 4
        degrees_per_range = 360 / num_ranges

        def index_range_for_segment(start_angle, end_angle):
            start_index = int(start_angle / degrees_per_range)
            end_index = int(end_angle / degrees_per_range)
            return ranges[start_index:end_index]
        
                    #calculate the min distance in each quadrant 
        def find_smallest_unique(segment):
            sorted_segment = sorted(segment)
            unique_distances = []
            last_added = None
            for distance in sorted_segment: 
                if (last_added is None and abs(distance - last_added) > 0.15) and distance > 0.23:
                    unique_distances.append(distance)
                    last_added = distance 
                    if len(unique_distances) >=5:
                        break
            return unique_distances
        
        def analyze_segment(segment):
            sorted_segment = sorted(segment)
            unique_distances = []
            last_added = None
            total = 0

            for distance in sorted_segment: 
                if (last_added is None or abs(distance - last_added) > 0.15) and distance > 0.23:
                    unique_distances.append(distance)
                    last_added = distance 
                    if len(unique_distances) >=5:
                        break
                total += distance
            average_distance = total / len(segment)
            return unique_distances, average_distance
        front_segment = index_range_for_segment(45, 90+45)
        right_segment = index_range_for_segment(90, 180)
        back_segment = index_range_for_segment(180, 270)
        left_segment = index_range_for_segment(270, 359)
        front = analyze_segment(front_segment)
        right = analyze_segment(right_segment)
        back = analyze_segment(back_segment)
        left = analyze_segment(left_segment)


        print(f"Front (Lidar left): {front} meters") #front is 45-135
       # print(f"Right (Lidar Front): {right} meters")
        # print(f"Back (Lidar Right): {back} meters")
        #print(f"Left (Lidar Back): {left} meters")

    
#--functions for camera detections

    def detect_april_tag_from_img(self, img):
        """
            returns the april tag id, translation vector and rotation matrix from
            :param img: image from camera stream, np array
            :return: dict: {int tag_id: tuple (float distance, float angle)}
            """
        # convert image to grayscale
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Apriltag detection
        detector = apriltag.Detector()
        detections = detector.detect(img_gray)
        # dictionary for return
        dict = {}
        # process each apriltag found in image to get the id and spacial information
        for detection in detections:
            tag_id = detection.tag_id
            translation_vector, rotation_matrix = self.homography_to_pose(detection.homography)
            dict[int(tag_id)] = (self.translation_vector_to_distance(translation_vector), self.rotation_matrix_to_angles(rotation_matrix))
        return dict

    def check_stop_sign_from_img(self, img, area_threshold):
        """
        check if stop sign appears in the image
        :param img: list RGB image array
        :param area_threshold: int threshold of how much percentage the stop sign should take up in the view to trigger
        :return: true if stop sign appear and appear large enough in the view, otherwise false
        """
        STOP_SIGN_THRESHOLD = 500
        red_only_img = self.red_filter(img)
        contour_img, stop_sign_area = self.add_contour(red_only_img)
        return True if stop_sign_area > STOP_SIGN_THRESHOLD else False, contour_img

#--Helper Functions for Computer Vision - not supposed to be called outside of the class
# AprilTag
    @staticmethod
    def homography_to_pose(H):
        """
        Convert a homography matrix to rotation matrix and translation vector.
        :param H: list homography matrix
        :return: tuple (list translation_vector, list rotational_matrix)
        """
        # Perform decomposition of the homography matrix
        R, Q, P = np.linalg.svd(H)

        # Ensure rotation matrix has determinant +1
        if np.linalg.det(R) < 0:
            R = -R

        # Extract translation vector
        t = H[:, 2] / np.linalg.norm(H[:, :2], axis=1)

        return t, R

    @staticmethod
    def rotation_matrix_to_angles(R):
        """
        Convert a 3x3 rotation matrix to Euler angles (in degrees).
        Assumes the rotation matrix represents a rotation in the XYZ convention.
        :param R, rotation_matrix: list
        :return: list [float angle_x, float angle_y, float angle_z]
        """
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])

    @staticmethod
    def translation_vector_to_distance(translation_vector):
        """
        convert 3D translation vector to distance
        :param translation_vector: list
        :return: float
        """
        # Calculate the distance from the translation vector
        distance = np.linalg.norm(translation_vector)
        return distance

# stop sign
    @staticmethod
    def red_filter(img):
        """
        mask image for red only area, note that the red HSV bound values are tunable and should be adjusted base on evironment
        :param img: list RGB image array
        :return: list RGB image array of masked red only image
        """
        # Colour Segmentation
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([-3, 100, 0])  # Lower bound for red hue
        upper_red = np.array([3, 255, 255])  # Upper bound for red hue
        red_pixels_mask = cv2.inRange(hsv_img, lower_red, upper_red)
        red_only_img = cv2.bitwise_and(img, img, mask=red_pixels_mask)
        return red_only_img

    @staticmethod
    def add_contour(img):
        """
        apply contour detection to the red only masked image
        :param img: list image array
        :return: tuple (list image with contour, int largest red-occupied area)
        """
        AREA_THRESHOLD = 50     # threshold of what percentage of area the stop sign occupies in the cameras view
        max_area = 0    # stores the largest red area detected
        edges = cv2.Canny(img, 100, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # approximate contour into a simpler shape
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx_polygon = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(approx_polygon)
            if area >= AREA_THRESHOLD:
                max_area = max(max_area, area)
                cv2.drawContours(img, [approx_polygon], -1, (0, 255, 0), 2)
        return img, max_area
