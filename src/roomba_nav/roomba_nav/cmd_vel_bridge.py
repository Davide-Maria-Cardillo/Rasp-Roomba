#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16, Int16, Int16MultiArray
import serial
import time


class VelSubscriberNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
           
        self.cmd_vel_subscriber_ = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.send_velocity_data, 
            10)
        self.get_logger().info("cmd_vel_bridge node has been started")
        
        self.cmd_serial_publisher_ = self.create_publisher(Int16MultiArray, '/roomba/cmd_serial', 10)
        
    def send_velocity_data(self, msg: Twist):
        linear_y = int(msg.linear.y)
        angular_z = int(msg.angular.z)
        
        # Limita i valori accettabili per Roomba
        linear_y = max(-500, min(500, linear_y))
        angular_z = max(-2000, min(2000, angular_z))  # angular_z è raggio,per il momento
        
        #Conversione in complemento a due
        vel_hb, vel_lb = divmod(linear_y & 0xFFFF, 256)  
        
        # Raggio di curvatura
        if angular_z == 0.0:
            radius_hb, radius_lb = 0x80, 0x00  # Muove dritto
        else:
            radius_hb, radius_lb = divmod(angular_z & 0xFFFF, 256)
    
        msg = Int16MultiArray()
        msg.data = [137, vel_hb, vel_lb, radius_hb, radius_lb]
        self.cmd_serial_publisher_.publish(msg)
        # Log dei valori calcolati
        self.get_logger().info(f"Sending: Vel=({vel_hb}, {vel_lb}), Rad=({radius_hb}, {radius_lb})")
        
            
def main(args=None):
    rclpy.init(args=args)
    node = VelSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
