#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import serial
import threading
import binascii

class MKRLoRaBridge(Node):
    def __init__(self):
        super().__init__('mkr_lora_bridge')
        
        # Serial config (Adjust /dev/ttyACM0 to your actual port)
        self.serial_port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to MKR WAN 1310 on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ROS 2 Topics
        self.rx_pub = self.create_publisher(String, '/bacs/lora_rx_events', 10)
        
        # Subscribe to outgoing constraints from the scheduler
        self.sub = self.create_subscription(
            UInt8MultiArray, 
            '/bacs/outgoing_payload', 
            self.send_callback, 
            10
        )
        
        # Start serial reading thread
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def send_callback(self, msg):
        # msg.data is a uint8 array. Byte 0 is seq, rest is payload.
        if len(msg.data) < 52:
            self.get_logger().warn("Payload too short, padding...")
            msg.data.extend([0] * (52 - len(msg.data)))
            
        seq = msg.data[0]
        # Convert bytes 1 to 51 into a hex string
        hex_payload = binascii.hexlify(bytes(msg.data[1:])).decode('ascii')
        
        # Format: T,<seq>,<104_hex_chars>
        cmd = f"T,{seq},{hex_payload}\n"
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().debug(f"Sent: {cmd.strip()}")

    def read_serial(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    if line.startswith("RX,"):
                        msg = String()
                        msg.data = line
                        self.rx_pub.publish(msg)
                    elif line.startswith("TX,"):
                        self.get_logger().info(f"TX Confirmed: {line}")
                    elif line.startswith("BOOT,"):
                        self.get_logger().info(f"Radio Booted: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = MKRLoRaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
