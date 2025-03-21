#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveNode(Node):
    
    def __init__(self):
        super().__init__("drive_node")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10) #1 creiamo un publisher
        self.timer_ = self.create_timer(0.1, self.send_velocity_command) #5 creiamo un timer che chiama la callback ogni 0.1 secondi 10Hz
        self.get_logger().info("Drive node has been started")
        
        self.start_time = time.time()
        self.state = 0 # Stato del comando di velocità

    
    def send_velocity_command(self): #2 creiamo una callback 
        msg = Twist() #3 creiamo un messaggio
        
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
               
        self.cmd_vel_pub_.publish(msg) #4 pubblichiamo il messaggio


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    rclpy.shutdown()

        