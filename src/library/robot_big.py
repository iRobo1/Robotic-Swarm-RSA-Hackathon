import time
import rclpy
from rclpy.node import Node 
import math
import numpy as np
from geometry_msgs.msg import Twist 
from queue import Queue
from std_msgs.msg import Int32
from library.utils import angle_difference, clamp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class Robot:
    def __init__(self):    
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

        self.node = Node(f"robot")

        self.cmd_vel_pub = self.node.create_publisher(Twist, "/mirte_base_controller/cmd_vel", 10)
        self.arm_pub = self.node.create_publisher(JointTrajectory, "/mirte_master_arm_controller/joint_trajectory", 10)
        self.gripper_client = ActionClient(self.node, GripperCommand, "/mirte_master_gripper_controller/gripper_cmd")

    def clamp_linear_speed(self, value, low, high):
        if value > 0.0 and value < 0.05:
            return 0.1
        else:
            return clamp(value, low, high)
    
    
    def drive_to(self, target_x, target_y, target_orientation):   
        msg = Twist()
        # go to target
        while True:
            rclpy.spin_once(self.node, timeout_sec=0)
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
    
    def drive(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

     # keep the movement duration short
    def move_arm_to(self, shoulder_rotate, shoulder_lift, elbow_joint, wrist_joint, movement_duration):
        msg = JointTrajectory()
        # order of joint names
        msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"]
        
        # -2.5 to 2.5
        point = JointTrajectoryPoint()
        point.positions = [shoulder_rotate, shoulder_lift, elbow_joint, wrist_joint] 
        point.time_from_start = Duration(sec=int(movement_duration))  

        msg.points = [point]
        self.arm_pub.publish(msg)
    
    def open_gripper(self, max_effort=20.0):
        # max is 0.5 (closed) and (-0.5) open
        position = -0.5
        print(f"Opening gripper (target: {position})...")

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            print("Gripper Action Server not found!")
            return
        

        goal = GripperCommand.Goal()
        # if you want the gripper open put on -0.5 and if you want it closed put on 0.5?
        goal.command.position = position
        # this is also a nice standard value
        goal.command.max_effort = max_effort

        future = self.gripper_client.send_goal_async(goal)

        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        print("Gripper command sent.")

        # rclpy.spin_until_future_complete(self.node, future)

    # max effor of 10 holds the items but maybe more needed
    def close_gripper(self, max_effort=20.0):
        # max is 0.5 (closed) and (-0.7) open
        position= 0.5
        print(f"Closing gripper (target: {position})...")

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            print("Gripper Action Server not found!")
            return

        goal = GripperCommand.Goal()
        # if you want the gripper open put on -0.5 and if you want it closed put on 0.5?
        goal.command.position = position
        # this is also a nice standard value
        goal.command.max_effort = max_effort

        future = self.gripper_client.send_goal_async(goal)

        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        print("Gripper command sent.")
        
        #rclpy.spin_until_future_complete(self.node, future)