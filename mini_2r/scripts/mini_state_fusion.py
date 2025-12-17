#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
import numpy as np
from time import sleep

"""
  The purpose of this node is to filtered the values of the joints for the mini robot
Karine Esmeral

"""


class mini_state_fusion(Node):
    def __init__(self):
        super().__init__('state_fusion_node')

        # Subscription : the joints values of the mini robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states', 
            self.joint_states_callback,
            10
        )

        # Publisher :the filtered joint values of the mini robot
        self.publisher = self.create_publisher(
            JointState,
            'mini/joint_states',
            10
        ) 


        self.absolute = np.array([0.0, 0.0])

        # To initialize the joint's absolute position by reading the real sensor of the mini robot
        for i in range(20):
            _, smg = wait_for_message(JointState, self,'/RR/joint_states') 
            self.absolute = self.absolute + np.array(smg.position)
        self.absolute = self.absolute / 20.0

       # Initial value of the joint obtained via Gazebo 
        _, msg= wait_for_message(JointState, self,'/joint_states') 
        self.q_int = np.array(msg.position)

          
        self.previous_ema =  np.array(msg.velocity)
        self.offset = np.array([self.q_int[0]-self.absolute[0], self.q_int[1]-(self.absolute[1]+self.absolute[0])])
        self.get_logger().info('iiwa_vel Node has been started.')



    def joint_states_callback(self, msg: JointState):
        
        fused_msg = msg
        fused_msg.position = [msg.position[0]-self.offset[0], msg.position[1]-self.offset[1]] 

        # Filter for the velocity : The filter corresponds to giving more weight to the previous value and less to the new value.
        weighting_factor = 0.05 # the most recent data is given a weight of 5% and the older is givent 95 %
        self.current_vel = np.array(fused_msg.velocity)

        self.ema= (np.dot(self.current_vel,weighting_factor))+(np.dot(self.previous_ema,(1-weighting_factor)))
        self.previous_ema=self.ema
    
        # Publish the new filter message
        filtered_msg = JointState()
        filtered_msg.header = fused_msg.header
        filtered_msg.name = fused_msg.name
        filtered_msg.position = fused_msg.position
        filtered_msg.velocity = self.ema.tolist()
        filtered_msg.effort = fused_msg.effort
        self.publisher.publish(filtered_msg)



def main(args=None):
    rclpy.init(args=args)
    node = mini_state_fusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()