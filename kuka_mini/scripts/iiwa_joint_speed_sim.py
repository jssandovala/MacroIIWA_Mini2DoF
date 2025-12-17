#!/usr/bin/env python3

import rclpy 
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.wait_for_message import wait_for_message


"""
    The purpose of this node is to control the macro robot in velocity.
    It receives the macro's velocities from the 9-DOF Jacobian matrix and 
    will publish only the first seven values, which correspond to the seven DOFs of the macro robot.
Karine Esmeral 
"""


class speed_control(Node):
    def __init__(self):
        super().__init__('iiwa_joint_speed_sim')

        # Subscription : value of the speed for the iiwa and mini from the redundacy node 
        # For the macro only the 7 first value are using
        self.delta_joint_subscriber = self.create_subscription(
            Float64MultiArray,
            '/iiwa/delta_joint',
            self.delta_joint_callback,
            10
        )

        # Publisher : value of the speed for the macro
        self.joint_publisher = self.create_publisher(
           Float64MultiArray,
            '/iiwa/velocity_controller/commands',
            10
        )

       
        # Initial desired velocity of the macro joints
        _, msg_delta = wait_for_message(Float64MultiArray,self,'/iiwa/delta_joint')
        self.delta_q = np.array(msg_delta.data[0:7])
       
        # The vector message 
        self.dq = Float64MultiArray()
        self.dq.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def delta_joint_callback(self, msg: Float64MultiArray):
            self.delta_q =  np.array(msg.data[0:7])
           
            
    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.dq.data = [self.delta_q [0], self.delta_q [1], self.delta_q [2], self.delta_q [3], self.delta_q [4], self.delta_q [5], self.delta_q [6]]
            self.joint_publisher.publish(self.dq)

        
def main():
    rclpy.init()
    vel_control = speed_control()
    try:
        vel_control.main_loop()

    except KeyboardInterrupt:   
        pass
    vel_control.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
       


        


    