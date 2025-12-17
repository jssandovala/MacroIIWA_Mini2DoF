#!/usr/bin/env python3

import rclpy 
import numpy as np
import threading
import time 
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.wait_for_message import wait_for_message

"""
  The purpose of this node is  to test the mini with a cartesian impedance and a joint impedance controller using the joint desired as input.  

Karine Esmeral

"""



class Mini_joint_impedance(Node):
    def __init__(self):
        super().__init__('mini2r_joint_impedance_sim')

        # The value of the joint for the iiwa and mini
        self.iiwa_joint_subscriber = self.create_subscription(
            JointState,
           '/joint_states',
            self.iiwa_joint_state_callback,
            10
        )

    
        # Publisher for the coupler position to the mini
        self.mini_coupler_publisher = self.create_publisher(
            Float64MultiArray,
           "/joint_effort_controller/commands",
            10
        )


        self.joints_publisher = self.create_publisher(
            Float64MultiArray,
            'mini/joints_pose',
            10

        )

       
        self.data_mutex = threading.Lock()
        # Initial joint positions and velocities
        ret,msg = wait_for_message(JointState,self,'/joint_states') 
        self.q = np.array([msg.position[0], msg.position[1]])
        self.dq = np.array([msg.velocity[0], msg.velocity[1]])
        self.q_prev = self.q.copy()
      
        self.delta_time = 0.001  ## Random value  Initial delta time
      
        self.t_acc = 0.0
        # Sinus parameters
        self.f = 1.0       # frequency (Hz)
        self.A1 = 1.0      # amplitude joint 1
        self.A2 = 1.0      # amplitude joint 2"""

        
    def iiwa_joint_state_callback(self, msg: JointState):
            self.q = np.array([msg.position[0], msg.position[1]])
            self.dq = np.array([msg.velocity[0], msg.velocity[1]])

    
    def delta_joint_callback(self, msg: Float64MultiArray):
            self.delta_q =np.array((msg.data[8], msg.data[7]))

         
    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            start_time = time.perf_counter()

             # Update time accumulator
            self.t_acc += self.delta_time

            # Desired delta joint positions (sinusoidal)
            self.delta_q = np.array([
                self.A1 * math.sin(1 * math.pi * self.f * self.t_acc),
                self.A2 * math.sin(2 * math.pi * self.f * self.t_acc)
            ])


            # Desired joint positions 
            q_desired = self.delta_q*self.delta_time + self.q_prev 
            self.q_prev = q_desired.copy()


            # Cartesian position a

            l0 = 0.043+0.084
            l1 = 0.2
            l2 = 0.2

            pos = np.array(np.zeros(2))
            pos[0]=l1*math.sin(self.q[0])+l2*math.sin(self.q[0]+self.q[1])
            pos[1]=l0+l1*math.cos(self.q[0])+l2*math.cos(self.q[0]+self.q[1])

            d_pos = np.array(np.zeros(2))
            d_pos[0]=l1*math.sin(q_desired[0])+l2*math.sin(q_desired[0]+q_desired[1])
            d_pos[1]=l0+l1*math.cos(q_desired[0])+l2*math.cos(q_desired[0]+q_desired[1])

            

            jacobian = np.zeros((2, 2)) 
            jacobian[0,0]=l1 * math.cos(self.q[0])+l2*math.cos(self.q[0]+self.q[1])
            jacobian[0,1]=l2 * math.cos(self.q[0]+self.q[1])
            jacobian[1,0]=-l1 * math.sin(self.q[0])-l2 * math.sin(self.q[0]+self.q[1])
            jacobian[1,1]=-l2 * math.sin(self.q[0]+self.q[1])

            erreur_pos = d_pos-pos

            # Proportional gain matrix
            kp_pos =  np.zeros((2, 2)) 
            kp_pos[0,0]=15.5
            kp_pos[1,1]=14.0

            # Derivative gain matrix
            kd_pos =np.zeros((2,2))
            kd_pos[0,0]=0.9
            kd_pos[1,1]=0.8

            # Cartesian controller 
            tau_pos = np.transpose(jacobian)*(kp_pos*erreur_pos-kd_pos*(jacobian*self.dq))
            tau_mini = [tau_pos[0,0], tau_pos[1,1]]

            # Proportional gain matrix
            kp=  np.zeros((2, 2)) 
            kp[0,0]=10.0
            kp[1,1]=10.0

            # Derivative gain matrix
            kd =np.zeros((2,2))
            kd[0,0]=2*np.sqrt(kp[0,0])
            kd[1,1]=2*np.sqrt(kp[1,1])

            # Joint controller 
            #tau = kp * (q_desired - self.q) - kd * self.dq
        
            # Publish the coupler position to the mini
            tau_mini = [tau_pos[0,0], tau_pos[1,1]]
            mini_coupler_msg = Float64MultiArray()
            mini_coupler_msg.data =tau_mini
            self.mini_coupler_publisher.publish(mini_coupler_msg)

            joints = [ q_desired[0],q_desired[1],self.q[0],self.q[1]]
            joint_pub = Float64MultiArray ()
            joint_pub.data = joints
            self.joints_publisher.publish(joint_pub)

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
    
       


        


    