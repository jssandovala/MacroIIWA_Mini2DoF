#!/usr/bin/env python3

import rclpy 
import numpy as np
import time 
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.wait_for_message import wait_for_message

"""
    The purpose of this node is to control the mini robot in impedance. It receives 
    the mini's velocities from the 9-DOF Jacobian matrix and will use the mini's velocities 
    only to compute the torques required to follow the interactive marker.
Karine Esmeral 
"""



class Mini_joint_impedance(Node):
    def __init__(self):
        super().__init__('mini2r_joint_impedance_sim')

        # Subscription :  value for all the joints for the iiwa and mini from the gazebo
        self.iiwa_joint_subscriber = self.create_subscription(
            JointState,
           '/iiwa/joint_states', 
            self.iiwa_joint_state_callback,
            10
        )

        # Subscription : value of the speed for the iiwa and mini from the redundacy node 
        # For the mini only the 2 last value are using
        self.delta_joint_subscriber = self.create_subscription(
            Float64MultiArray,
            '/iiwa/delta_joint',
            self.delta_joint_callback,
            10
        )

        # Publisher : value of the couple for the mini
        self.mini_coupler_publisher = self.create_publisher(
            Float64MultiArray,
           '/iiwa/joint_effort_controller/commands',
            10
        )

        # Initial joint positions and velocities
        _,msg = wait_for_message(JointState,self,'/iiwa/joint_states') 
        self.q = np.array([msg.position[1], msg.position[0]])
        self.dq =np.array([msg.velocity[1], msg.velocity[0]])
        self.q_prev = self.q.copy()
      
        # Initial desired velocity of the mini joints
        _, msg_delta = wait_for_message(Float64MultiArray,self,'/iiwa/delta_joint')
        self.delta_q = np.array([msg_delta.data[8], msg_delta.data[7]])

        # Initiale value for the time 
        self.delta_time = 0.001  
     

    def iiwa_joint_state_callback(self, msg: JointState):
            self.q = np.array([msg.position[1], msg.position[0]])
            self.dq =np.array([msg.velocity[1], msg.velocity[0]])


    
    def delta_joint_callback(self, msg: Float64MultiArray):
            self.delta_q =np.array((msg.data[8], msg.data[7]))

         
    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            start_time = time.perf_counter()

            # Desired joint positions 
            q_desired = self.delta_q*self.delta_time + self.q_prev 
            self.q_prev = q_desired.copy()


            # Proportional gain matrix
            kp=  np.zeros((2, 2)) 
            kp[0,0]=10.0
            kp[1,1]=10.0

            # Derivative gain matrix
            kd =np.zeros((2,2))
            kd[0,0]=2*np.sqrt(kp[0,0])
            kd[1,1]=2*np.sqrt(kp[1,1])

            # Couple controller 
            tau = kp * (q_desired - self.q) - kd * self.dq
    

            # Publish the coupler position to the mini
            tau_mini = [tau[0,0], tau[1,1]]
            mini_coupler_msg = Float64MultiArray()
            mini_coupler_msg.data =tau_mini
            self.mini_coupler_publisher.publish(mini_coupler_msg)

            end_time = time.perf_counter()
            self.delta_time = end_time - start_time

           
def main():
    rclpy.init()
    joint_impedadance = Mini_joint_impedance()
    try:
        joint_impedadance.main_loop()
    except KeyboardInterrupt:   
        pass
    joint_impedadance.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
       


        


    