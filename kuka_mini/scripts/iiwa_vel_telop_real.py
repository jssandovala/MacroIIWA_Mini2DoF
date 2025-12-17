#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped

import roboticstoolbox as rtb
from pyquaternion import Quaternion
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl,rpy2tr
import numpy as np


class iiwa_Vel(Node):
    def __init__(self):

        super().__init__('iiwa_vel_node')
        self.update_rate = self.create_rate(1000)  

        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/iiwa/joint_states',
            self.joint_states_callback,
            10
        )

        self.marker_sub = self.create_subscription(
            PoseStamped,
            '/iiwa/pose',
            self.Pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/iiwa/velocity_controller/commands',
            10
        )

        #To obtain info for the graphs
        # self.error_mini_pub = self.create_publisher(
        #     Float64MultiArray,
        #     '/error_mini',
        #     10
        # )

        self.fk_mini_sub = self.create_subscription(
            Float64MultiArray,
            '/fk_mini',
            self.fk_mini_callback,
            10
        )


        self.initial_pose_mode_sub = self.create_subscription(
            Bool,
            '/initial_pose_mode',
            self.initial_pose_mode_callback,
            10
        )


        ret, smg = wait_for_message(JointState, self,'/iiwa/joint_states')
        self.q_int = np.array(smg.position)
        self.q = self.q_int

        self.q_null=np.array(self.q )
        self.dq_com=Float64MultiArray()
        self.dq_com.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.max_postions = [2.9147, 2.0594, 2.9147, 2.0594 ,2.9147, 2.0594, 3.0194]
        self.max_velocity = [1.5, 1.5, 1.5, 2.0, 2.0, 3.0, 3.0]  # real data [1.71, 1.71, 1.74, 2.26, 2.44, 3, 3]  

        self.kuka_robot()
        
  
        self.d_cart_pose=PoseStamped()  
        self.cart_pose=PoseStamped()
        # TODO changed with wait_for_message
        self.d_cart_pose= self.MGD(self.q_int)

        self.fk_mini_info=Float64MultiArray() 
        self.warned_about_fk = False

        # self.fk_mini_pin_info=PoseStamped()
        # self.fk_kuka_pin_sub_info=PoseStamped()

        self.initial_pose_mode = True

        
        self.d_cart_pose = self.MGD(self.q_int)


        self.get_logger().info('iiwa_vel Node has been started.')


    def joint_states_callback(self, msg: JointState):
        self.q = np.array(msg.position)
    
    def Pose_callback(self, msg: PoseStamped):
        self.d_cart_pose=msg
    
    #Mini's end-effector position by hand, w.r.t the mini's frame
    def fk_mini_callback(self, msg: Float64MultiArray):
        self.fk_mini_info=msg
        # self.get_logger().info(f'FK mini message trial: {msg.data}')

    def initial_pose_mode_callback (self, msg: Bool):
        self.initial_pose_mode = msg.data

        # if self.initial_pose_mode:
        #     self.get_logger().info("Kuka in initial position.")
        # else:
        #     self.get_logger().info("Kuka using the mini's error as input.")

    def Pos_error (self, pos_d ,pos):
        target_quaternion = Quaternion(pos_d.pose.orientation.w, pos_d.pose.orientation.x, pos_d.pose.orientation.y, pos_d.pose.orientation.z)
        current_quaternion= Quaternion (pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z)
        error_quaternion= target_quaternion * current_quaternion.conjugate
        dso3=error_quaternion.axis *error_quaternion.angle
        dX=np.array([(pos_d.pose.position.x-pos.pose.position.x),(pos_d.pose.position.y-pos.pose.position.y),(pos_d.pose.position.z-pos.pose.position.z),dso3[0],dso3[1],dso3[2]])
        return(dX)


    def kuka_robot(self):
        L = [
            RevoluteDH(a=0.0, d=0.360, alpha=-np.pi/2, qlim=np.array([-self.max_postions[0], self.max_postions[0]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,  qlim=np.array([-self.max_postions[1], self.max_postions[1]])),
            RevoluteDH(a=0.0, d=0.420, alpha=np.pi/2,  qlim=np.array([-self.max_postions[2], self.max_postions[2]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=-np.pi/2, qlim=np.array([-self.max_postions[3], self.max_postions[3]])),
            RevoluteDH(a=0.0, d=0.400, alpha=-np.pi/2, qlim=np.array([-self.max_postions[4], self.max_postions[4]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,  qlim=np.array([-self.max_postions[5], self.max_postions[5]])),
            RevoluteDH(a=0.0, d=0.154, alpha=0.0,      qlim=np.array([-self.max_postions[6], self.max_postions[6]])),
        ]
        self.kuka = DHRobot( L, name="kuka")

    
    def MGD (self, data_q):
        T=self.kuka.fkine(data_q)
        TT=np.array([[T.n[0],T.o[0],T.a[0],T.t[0]], [T.n[1],T.o[1],T.a[1],T.t[1]], [T.n[2],T.o[2],T.a[2],T.t[2]], [0, 0, 0, 1]])
        quat = Quaternion(matrix=TT)
        FK= PoseStamped()
        FK.pose.position.x = TT[0,3]
        FK.pose.position.y = TT[1,3]
        FK.pose.position.z = TT[2,3]
        FK.pose.orientation.x = quat[1]
        FK.pose.orientation.y = quat[2]
        FK.pose.orientation.z = quat[3]
        FK.pose.orientation.w = quat[0]
        return(FK)
    

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            # update data
            self.cart_pose=self.MGD(self.q)

            ref_mini = np.array([0, 0.3]) #you can chhange this values if desired, it's the reference position of the mini in its frame

            if len(self.fk_mini_info.data) >= 2:
                error_mini_x = self.fk_mini_info.data[0] - ref_mini[0]
                error_mini_z = self.fk_mini_info.data[1] - ref_mini[1]
                # print('error: ', error_mini_x, error_mini_z)
                # print('fk_mini_info: ', self.fk_mini_info.data)
            

            #Check if it will use the original pose
            if self.initial_pose_mode:
                dX_c = self.Pos_error(self.d_cart_pose,self.cart_pose)
                # self.get_logger().info("Kuka is staying at its initial pose.")

            else:
                if len(self.fk_mini_info.data) >= 2:

                    # error_mini_x = self.fk_mini_info.data[0] - ref_mini[0]
                    # error_mini_z = self.fk_mini_info.data[1] - ref_mini[1]

                    dX_c = self.Pos_error(self.d_cart_pose,self.cart_pose)

                    #Cleaner so I can easily see the info in the terminal
                    print(f"Distance mini in its frame: {self.fk_mini_info.data[1]:.3f}m | Error in x and z: {error_mini_x:.3f}, {error_mini_z:.3f}", end='\r')

                    dX_c[0] = error_mini_z
                    dX_c[1] = error_mini_x

                    # # To obtain information to plot graphs
                    # self.error_mini_msg = Float64MultiArray()
                    # self.error_mini_msg.data = [error_mini_x, error_mini_z, self.get_clock().now().nanoseconds * 1e-9]  #seconds
                    # self.error_mini_pub.publish(self.error_mini_msg)
                   
                    
                else:
                    if not self.warned_about_fk:
                        self.get_logger().warn("Waiting for /fk_mini to publish valid FK data.")
                        self.warned_about_fk = True
                    continue

            # dq  
            J=self.kuka.jacob0(self.q)
            J_t= np.transpose(J)
            J_p= np.dot(J_t,np.linalg.inv(np.dot(J,J_t)))
            dq= np.dot(J_p,(1.1*dX_c)) # the 1.5 is the gain, it can be changed if desired

            # null space
            dq_N1= np.dot( (np.eye(7) - np.dot(J_p,J)), (self.q_null - self.q))
    
            # final velocity vector
            dq = dq+ dq_N1

            
            # limit velocity
            for i in range(7):
                if  abs(dq[i])>self.max_velocity[i]:
                    if (dq[i]<0):
                        dq[i]=-self.max_velocity[i]
                    else:
                        dq[i]=self.max_velocity[i]

            # limit position
            for i in range(7):
                if  (self.q[i]>(self.max_postions[i]-0.0523599)) and (dq[i]>0): # -3 degree offset
                    dq[i]=0
                if  (self.q[i]<-(self.max_postions[i]-0.0523599)) and (dq[i]<0): # +3 degree offset
                    dq[i]=0
                
                 
            self.dq_com.data = [dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6]]
            self.publisher.publish(self.dq_com)
                  
            

def main():
    rclpy.init()
    vel_control = iiwa_Vel()
    try:
        vel_control.main_loop()
    except KeyboardInterrupt:   
        pass
    vel_control.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()