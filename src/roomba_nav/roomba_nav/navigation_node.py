#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt16, Int16, Int16MultiArray
from geometry_msgs.msg import Twist
import serial
import time


class SensorSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("navigation_node")
        
        self.wall_subscriber_ = self.create_subscription(
            Int16,
            '/roomba/sensors/wall',
            self.wall_callback, 
            10
        )
        self.battery_subscriber_ = self.create_subscription(
            UInt16,
            '/roomba/sensors/battery',
            self.battery_callback, 
            10
        )
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.send_velocity_command)
        
        self.get_logger().info("navigation_node has been started")
        
        self.start_time = time.time()
        

    def wall_callback(self, msg: Int16):
        self.get_logger().info(f"Wall Sensor value: {msg.data}")
    
    def battery_callback(self, msg: Int16):
        self.get_logger().info(f"Battery Sensor value: {msg.data}")
        
    def send_velocity_command(self):
        msg = Twist()
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time < 5:
            msg.linear.y = 200.0 #mm/s 
            msg.angular.z = 0.0
        elif elapsed_time < 10:
            msg.linear.y = 200.0 #mm/s 
            msg.angular.z = 1.0
        elif elapsed_time < 13:
            msg.linear.y = 0.0 #mm/s 
            msg.angular.z = 0.0
        elif elapsed_time < 16:
            msg.linear.y = 200.0 #mm/s 
            msg.angular.z = -1.0
        else:
            msg.linear.y = 0.0 #mm/s 
            msg.angular.z = 0.0
               
        self.cmd_vel_publisher_.publish(msg) 


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()