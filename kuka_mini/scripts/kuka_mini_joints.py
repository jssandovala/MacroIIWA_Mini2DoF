#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
import numpy as np


class nine_Joints(Node):
    def __init__(self):

        super().__init__('nine_Joints_node')
        self.update_rate = self.create_rate(1000)  

        
        self.iiwa_joint_state_sub = self.create_subscription(
            JointState,
            '/iiwa/joint_states',
            self.iiwa_joint_states_callback,
            10
        )

        self.mini_joint_state_sub = self.create_subscription(
            JointState,
            '/RR/joint_states',
            self.mini_joint_states_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState, 
            '/iiwa_mini_joint_states', 
            10
        )


        ret, smg_iiwa = wait_for_message(JointState, self,'/iiwa/joint_states')
        self.q_int_iiwa = np.array(smg_iiwa.position)
        self.q_iiwa = self.q_int_iiwa


        ret, smg_mini = wait_for_message(JointState, self,'/RR/joint_states')
        self.q_int_mini = np.array(smg_mini.position)
        self.q_mini = self.q_int_mini
        self.q_mini_vel = np.array(smg_mini.velocity)
        self.q_mini_effort = np.array(smg_mini.effort)

        self.q_iiwa_vel = np.array(smg_iiwa.velocity)
        self.q_iiwa_effort = np.array(smg_iiwa.effort)

        self.iiwa_joints_received = False
        self.mini_joints_received = False
        self.dq_iiwa_mini = JointState()

        self.get_logger().info('nine_Joints Node has been started.')


    def iiwa_joint_states_callback(self, msg: JointState):
        self.q_iiwa = np.array(msg.position)
        self.q_iiwa_vel = np.array(msg.velocity)
        self.q_iiwa_effort = np.array(msg.effort)
        self.iiwa_joints_received = True
    
    def mini_joint_states_callback(self, msg: JointState):
        self.q_mini = np.array(msg.position)
        self.q_mini_vel = np.array(msg.velocity)
        self.q_mini_effort = np.array(msg.effort)
        self.mini_joints_received = True



    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.iiwa_joints_received and self.mini_joints_received:
                
                self.dq_iiwa_mini.header.stamp = self.get_clock().now().to_msg()
                self.dq_iiwa_mini.name = [ 'joint_2',  'joint_1','joint_a1', 'joint_a2', 'joint_a3', 
                                          'joint_a4', 'joint_a5', 'joint_a6', 
                                          'joint_a7']
                
                self.dq_iiwa_mini.header.frame_id = 'iiwa_mini_joint_states'

                self.dq_iiwa_mini.position = [ self.q_mini[1],  self.q_mini[0], self.q_iiwa[0], self.q_iiwa[1], self.q_iiwa[2],
                                              self.q_iiwa[3], self.q_iiwa[4], self.q_iiwa[5], 
                                              self.q_iiwa[6]]
                self.dq_iiwa_mini.velocity = [ self.q_mini_vel[1],  self.q_mini_vel[0], self.q_iiwa_vel[0], self.q_iiwa_vel[1], self.q_iiwa_vel[2],
                                              self.q_iiwa_vel[3], self.q_iiwa_vel[4], self.q_iiwa_vel[5], 
                                              self.q_iiwa_vel[6]]   
                self.dq_iiwa_mini.effort = [ self.q_mini_effort[1],  self.q_mini_effort[0], self.q_iiwa_effort[0], self.q_iiwa_effort[1], self.q_iiwa_effort[2],
                                              self.q_iiwa_effort[3], self.q_iiwa_effort[4], self.q_iiwa_effort[5], 
                                              self.q_iiwa_effort[6]]                
    
            self.publisher.publish(self.dq_iiwa_mini)

                  
            

def main():
    rclpy.init()
    nine_Joints_ = nine_Joints()
    try:
        nine_Joints_.main_loop()
    except KeyboardInterrupt:   
        pass
    nine_Joints_.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()





    