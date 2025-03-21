#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import atexit 


class VelSubscriberNode(Node):
    def __init__(self):
        super().__init__("vel_serial_node")
        
        # Configura connessione seriale con Roomba        
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(0.2)
        
        # Inizializza Roomba in modalità Full
        self.ser.write(bytes([128]))  # Start
        time.sleep(0.2)
        self.ser.write(bytes([130]))  # Control (Safe Mode)
        time.sleep(0.2)
        self.ser.write(bytes([132]))  # Full Mode
        time.sleep(0.2)
        
        self.cmd_vel_subscriber_ = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.cmd_vel_callback, 
            10)
        self.get_logger().info("Serial node has been started")
        
        # Registra la funzione di chiusura
        atexit.register(self.close_connection)
        
    def cmd_vel_callback(self, msg: Twist):
        
        linear_y = int(msg.linear.y)
        angular_z = int(msg.angular.z)
        
        # Limita i valori accettabili per Roomba
        linear_y = max(-500, min(500, linear_y))
        angular_z = max(-2000, min(2000, angular_z))  # angular_z è raggio,per il momento
        
        # Conversione in complemento a due
        vel_hb, vel_lb = divmod(linear_y & 0xFFFF, 256)  
        
        # Raggio di curvatura
        if angular_z == 0.0:
            radius_hb, radius_lb = 0x80, 0x00  # Muove dritto
        else:
            radius_hb, radius_lb = divmod(angular_z & 0xFFFF, 256)
        
        # Log dei valori calcolati
        self.get_logger().info(f"Sending: Vel=({vel_hb}, {vel_lb}), Rad=({radius_hb}, {radius_lb})")

        # Invia il comando Drive (137) al Roomba
        self.ser.write(bytes([137, vel_hb, vel_lb, radius_hb, radius_lb]))
        
    def close_connection(self):
        """ Chiude la connessione seriale quando il nodo si spegne. """
        if self.ser.is_open:
            self.get_logger().info("Closing serial connection...")
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
    
    def destroy_node(self):
        """ Assicura che la connessione venga chiusa quando il nodo viene distrutto. """
        self.close_connection()
        super().destroy_node()
            
            
def main(args=None):
    rclpy.init(args=args)
    node = VelSubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
