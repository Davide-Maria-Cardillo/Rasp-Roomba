#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt16, Int16, Int16MultiArray
from geometry_msgs.msg import Twist
import serial
import time
import atexit


class SensorsNode(Node):
    
    def __init__(self):
        super().__init__("sensors_node")
        
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
        
        self.wall_pub_ = self.create_publisher(Int16, "/roomba/sensors/wall", 10) 
        self.battery_pub_ = self.create_publisher(UInt16, "/roomba/sensors/battery", 10)
        
        self.cmd_serial_subscriber_ = self.create_subscription(
            Int16MultiArray,
            '/roomba/cmd_serial',
            self.write_serial_velocity,
            10
        )
        
        self.timer_ = self.create_timer(0.1, self.read_sensors_data) 
        
        self.get_logger().info("sensor_node has been started")
        
        atexit.register(self.close_connection)

    def write_serial_velocity(self, msg: Int16MultiArray):
        """Scrive il comando di velocità sulla seriale quando riceve dati su /roomba/cmd_serial."""
        if len(msg.data) == 5 and msg.data[0] == 137:
            vel_hb, vel_lb, radius_hb, radius_lb = msg.data[1], msg.data[2], msg.data[3], msg.data[4]
            
            # Converte i dati ricevuti in un comando seriale per il Roomba
            command = bytes([137, vel_hb, vel_lb, radius_hb, radius_lb])
            self.ser.write(command)

            # Log dei dati inviati alla seriale
            self.get_logger().info(f"Sent to serial: {command}")
        else:
            self.get_logger().warn("Received malformed command on /roomba/cmd_serial")
    
    def read_sensors_data(self): 
        """Legge i dati dei sensori e li pubblica su topic separati."""
        self.ser.write(bytes([142, 0])) 
        data = self.ser.read(26)        
        
        if len(data) == 26:
            # Decodifica i dati
            wall = data[0]
            battery = int.from_bytes(data[22:24], "big", signed=False)
            
            self.publish_data(self.wall_pub_,wall)
            # self.publish_data(self.battery_pub_,battery)
            
            msg = UInt16()
            msg.data = battery
            self.battery_pub_.publish(msg)
    
    def publish_data(self, publisher, value): 
        msg = Int16()
        msg.data = value
        publisher.publish(msg)                   

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
    node = SensorsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
