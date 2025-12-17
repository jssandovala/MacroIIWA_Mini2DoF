#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import struct

"""
 The purpose of this node is to decode the real value of the joint for the mini robot  

Karine Esmeral
"""

class UDPListenerNode(Node):
    def __init__(self):
        super().__init__('sensors_listener_node')

        # Publisher : the real value of the joint position of the mini robot
        self.publisher = self.create_publisher(JointState, 'RR/joint_states', 10) 

        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 5005

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))

        self.get_logger().info(f"Start listening for sensors on port {self.UDP_PORT}...")

    def listen(self):
        while rclpy.ok():
            data, addr = self.sock.recvfrom(1024)
            if len(data) >= 16:  
                try:

                    sensor1 = struct.unpack('<d', data[:8])[0]
                    sensor2 = struct.unpack('<d', data[8:16])[0]

                    msg = JointState()  
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'base_link'

                    msg.name = ['joint1', 'joint2']  
                    msg.position = [sensor1, sensor2] 
                    msg.velocity = [0.0, 0.0]
                    msg.effort = [0.0, 0.0]

                    self.publisher.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f"Failed to decode data: {e}")
            else:
                self.get_logger().warn(f"Received data too short: {data}")

def main(args=None):
    rclpy.init(args=args)

    udp_listener_node = UDPListenerNode()

    import threading
    udp_thread = threading.Thread(target=udp_listener_node.listen)
    udp_thread.daemon = True
    udp_thread.start()

    rclpy.spin(udp_listener_node)

    udp_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
