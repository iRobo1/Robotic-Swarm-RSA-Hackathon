import time
import rclpy
from rclpy.node import Node 
# import requests
import math
from sensor_msgs.msg import Image
from enum import Enum
# import websockets
import threading
import argparse
import struct
import asyncio
# import apriltag
# from cv_bridge import CvBridge
# import cv2
import math
# import numpy as np
from queue import Queue
from std_msgs.msg import Int32

class Communication:
    def __init__(self):
        self.location_callback = None
        self.start_callback = None
        self.stop_callback = None
        self.objective_callback = None
    
    def register_callback_location(self, func):
        self.location_callback = func
    
    def register_callback_start(self, func):
        self.start_callback = func
        
    def register_callback_stop(self, func):
        self.stop_callback = func
    
    def register_callback_objective(self, func):
        self.objective_callback = func
    
    async def connect_and_listen_to_websocket(self):
        
        url = f"ws://{END_POINT}/ws/connect/{TEAM_ID}/{ROBOT_ID}"
        while True:
            try:
                async with websockets.connect(url) as websocket:
                    print("Connected!")
                    async for packet in websocket:
                        
                        message = struct.unpack_from("<B", packet)[0]
                        if message == START_MSG:
                            if self.start_callback:
                                self.start_callback()
                        elif message == STOP_MSG:
                            if self.stop_callback:
                                self.stop_callback()
                        elif message == LOCATION_MSG:
                            _, x, y, angle, visible, last_seen = struct.unpack("<Bfff?I", packet)
                            if self.location_callback:
                                self.location_callback(x, y, angle, visible, last_seen)
                        elif message == OBJECTIVE_FOUND_MSG:
                            _, team_id, robot_id, tag_id, x, y, angle, visible, last_seen = struct.unpack("<BBBBfff?I", packet)
                            
                            if self.objective_callback:
                                self.objective_callback(team_id, robot_id, tag_id, x, y, angle, visible, last_seen)
            except Exception as e:
                print(f"Error/Disconnected: {e}, retrying in 5 seconds...")
                await asyncio.sleep(5)

TEAM_CREDENTIALS = ("3", "test3")
END_POINT = "192.168.1.20:8000"
TEAM_ID = 3
ROBOT_ID = 0

LOCATION_MSG = 1
START_MSG = 2
STOP_MSG = 3
OBJECTIVE_FOUND_MSG = 4

class RobotState(Enum):
    STOPPED = "stopped"
    STARTED = "started"

class SafePublisher:
    def __init__(self, publisher, robot, stop_msg):
        self.publisher = publisher
        self.robot = robot
        self.stop_msg = stop_msg
    
    def publish(self, msg):
        if self.robot.state == RobotState.STOPPED:
            if self.stop_msg is not None:
                self.publisher.publish(self.stop_msg)
            return
        return self.publisher.publish(msg)

def angle_difference(desired, current):
    difference = desired - current
    difference = (difference + math.pi) % (2 * math.pi) - math.pi
    return difference

def clamp(value, low, high):
    return max(low, min(high, value))

class Robot:
    def __init__(self, team_id, authorization, dev_mode=False):
        if dev_mode:
            self.state = RobotState.STARTED
        else:
            self.state = RobotState.STOPPED

        self.robot_position = None
        
        self.latest_image = None

        rclpy.init()
        
        # for now objectives in a queue but you can choose your own heuristic to 
        # optimize in which order to store the objectives
        # this is an objective queue before handling the stuff from the queue first use
        # if not self.objective_queue.empty(): in the place where you handle the objectives
        # the queue can also be done with a priority queue
        self.objective_queue = Queue()
        self.sent_tags = set()

        self.robot_id = ROBOT_ID
        self.authorization = authorization
        self.bridge = CvBridge()
        # self.april_tag_option = apriltag.DetectorOptions(families="tag36h11")
        # self.april_tag_detector = apriltag.Detector(self.april_tag_option)

       
        
        # control parameters
        self.kp_linear = 0.6
        self.kp_angular = 2.5

        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.8

        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

        self.stop_driving_forward_angle = math.pi / 2
        self.slower_driving_forward_angle = 0.4

        self.poll_freq = 0.01

        
        self.node = Node(f"robot_{self.robot_id}")
        
        self.img_subscriber = self.node.create_subscription(Image, "/video1/image_raw", self.on_receive_image, 10)
        self.pub_motor_left = SafePublisher(self.node.create_publisher(Int32, '/io/motor/left/speed', 10), self, Int32())
        self.pub_motor_right = SafePublisher(self.node.create_publisher(Int32, '/io/motor/right/speed', 10), self, Int32())

    def clamp_linear_speed(self, value, low, high):
        if value > 0.0 and value < 0.05:
            return 0.1
        else:
            return clamp(value, low, high)
    
    def wait_if_stopped(self):
        while self.state == RobotState.STOPPED:
            time.sleep(0.1)
    
    def on_receive_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        
    def process_image(self, target_team_id=2):
        if self.state != RobotState.STARTED:
            return
        if self.latest_image is None:
            return 
        self.detect_objective_tag(self.latest_image, target_team_id)
    
    # april tag from the objective is close enough (only allowed to send a message at 50 cms)
    # def detect_objective_tag(self, image, target_team_id):
    #     # for now always take the first tag
    #     print(image.shape)
    #     gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #     tags_detected = self.april_tag_detector.detect(gray_img)
    #     print("tags: ", tags_detected)
    #     if tags_detected:
    #         first_detected_tag = tags_detected[0]
    #         corners = np.array(first_detected_tag.corners)
    #         edge_size =  math.sqrt(((corners[1][0] - corners[0][0]) ** 2) + ((corners[1][1] - corners[0][1]) ** 2))
    #         print("TAG SIZE: ", edge_size)

    #         # threshold of tag size on 50 cm distance
         
    #         if edge_size > 55:
    #             tag_id = first_detected_tag.tag_id
    #             # do not send a tag multiple times to avoid spamming
    #             if tag_id not in self.sent_tags:

    #                 print("tag id", tag_id)
    #                 # self.send_objective_msg(target_team_id, tag_id)
    #                 self.sent_tags.add(tag_id)

    def send_objective_msg(self, target_team_id, tag_id):
        if self.state != RobotState.STARTED:
            return None
        packet = struct.pack("<BBB", OBJECTIVE_FOUND_MSG, target_team_id, tag_id)

        url = f"http://{END_POINT}/objective?team_id={TEAM_ID}&robot_id={ROBOT_ID}"
        
        response = requests.post(url, auth=TEAM_CREDENTIALS, data=packet)
        
        return response.json()
    
    def drive_to(self, target_x, target_y, target_orientation):   
        msg = Twist()
        # go to target
        while True:
            rclpy.spin_once(self.node, timeout_sec=0)
            self.wait_if_stopped()
            dx = target_x - self.robot_position["x"]
            dy = target_y - self.robot_position["y"]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.linear_threshold:
                break
            
            target_heading: float = math.atan2(dy, dx)
            error = angle_difference(target_heading, -self.robot_position["angle"])
            
            # anything between the values between 0.01 and 0.1 you just clamp up to 0.1
            linear_speed = self.clamp_linear_speed(self.kp_linear * distance, 0.0, self.max_linear_speed)
            angular_speed = clamp(self.kp_angular * error, -self.max_angular_speed, self.max_angular_speed)
            
            if abs(error) > self.stop_driving_forward_angle:
                linear_speed = 0.0
            elif abs(error) > self.slower_driving_forward_angle:
                linear_speed *= 0.2
        
            msg = Twist()
            msg.angular.z = angular_speed
            msg.linear.x = linear_speed
            
            self.cmd_vel_pub.publish(msg)
            time.sleep(self.poll_freq)

        # orient in the desired direction
        while True:
            rclpy.spin_once(self.node, timeout_sec=0)
            self.wait_if_stopped()
            
            error = angle_difference(target_orientation, -self.robot_position["angle"])

            angular_speed = clamp(error, -self.max_angular_speed, self.max_angular_speed)

            if abs(error) < self.angular_threshold:
                break
            
            # I did not test yet whether this works
            if abs(error) < 0.1:
                msg.angular.z = angular_speed
            else:
                msg.angular.z = clamp(error, -self.max_angular_speed, self.max_angular_speed)
            
            msg = Twist()
            msg.angular.z = angular_speed
            self.cmd_vel_pub.publish(msg)
            time.sleep(self.poll_freq)
        
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.2)
    
    def drive(self, left_motor_speed, right_motor_speed):
        msg_right = Int32()
        msg_left = Int32()

        msg_right.data = right_motor_speed
        msg_left.data = left_motor_speed

        self.pub_motor_left.publish(msg_left)
        self.pub_motor_right.publish(msg_right)
    
    def on_receive_location(self, x, y, angle, visible, last_seen):
        self.robot_position = {"x": x, "y": y, "angle": angle, "visible": visible, "last_seen": last_seen}
    
    def on_receive_start(self):
        self.state = RobotState.STARTED
    
    def on_receive_stop(self):
        self.state = RobotState.STOPPED
        self.pub_motor_left.publish(Int32())
        self.pub_motor_right.publish(Int32())
    
    def on_receive_objective(self, team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
        new_objective = {
            "tag_id": tag_id,
            "x": x,
            "y": y,
            "found_by_team": team_id,
            "found_by_robot": robot_id,
            "angle": angle,
            "visible": visible,
            "last_seen": last_seen
        }
        
        self.objective_queue.put(new_objective)