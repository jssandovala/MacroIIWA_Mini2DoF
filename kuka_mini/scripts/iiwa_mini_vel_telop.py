#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
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
            '/mini/pose',
            self.Pose_callback,
            10
        )

        self.publisher_kuka = self.create_publisher(
            Float64MultiArray, 
            '/iiwa/velocity_controller/commands', 
            10
        )

        self.publisher_mini = self.create_publisher(
            Float64MultiArray, 
            '/iiwa/joint_vel_controller/commands',
            10
        )

        ret, msg = wait_for_message(JointState, self,'/iiwa/joint_states') 
        self.q_int = np.array([msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7], msg.position[8], msg.position[0], msg.position[1]])
        self.q = self.q_int

        self.q_null=np.array(self.q )
        self.dq_com_kuka=Float64MultiArray()
        self.dq_com_kuka.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.dq_com_mini=Float64MultiArray()
        self.dq_com_mini.data = [0.0, 0.0]

        self.max_postions = [2.9147, 2.0594, 2.9147, 2.0594 ,2.9147, 2.0594, 3.0194, 3.0194, 3.0194]
        self.max_velocity = [1.3, 1.3, 1.3, 1.9, 1.9, 3.0, 3.0, 3.0, 3.0]  # real data [1.71, 1.71, 1.74, 2.26, 2.44, 3, 3]  

        """self.declare_parameter('tool_xyz', '0.0 0 0.0')
        tool_xyz= self.get_parameter('tool_xyz').value
        self.declare_parameter('tool_rpy', '0 0 0')
        tool_rpy= self.get_parameter('tool_rpy').value
        tool_xyz = tool_xyz.split()
        tool_rpy = tool_rpy.split()"""

        self.kuka_robot()
        #self.tool=transl(float(tool_xyz[0]), float(tool_xyz[1]), float(tool_xyz[2])) @ rpy2tr(float(tool_rpy[0]), float(tool_rpy[1]), float(tool_rpy[2]))
        #self.kuka.tool=self.tool
  
        self.d_cart_pose=PoseStamped()  
        self.cart_pose=PoseStamped()
        # TODO changed with wait_for_message

        # test
        #self.q_int=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.d_cart_pose= self.MGD(self.q_int)

        print(self.q_int)
        print('fkm')
        print(self.d_cart_pose.pose.position.x,self.d_cart_pose.pose.position.y,self.d_cart_pose.pose.position.z)
        

        self.get_logger().info('iiwa_vel Node has been started.')


    def joint_states_callback(self, msg: JointState):
        self.q = np.array([msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7], msg.position[8], msg.position[0], msg.position[1]])
    

    def Pose_callback(self, msg: PoseStamped):
        self.d_cart_pose=msg


    def Pos_error (self, pos_d ,pos):
        target_quaternion = Quaternion(pos_d.pose.orientation.w, pos_d.pose.orientation.x, pos_d.pose.orientation.y, pos_d.pose.orientation.z)
        current_quaternion= Quaternion (pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z)
        error_quaternion= target_quaternion * current_quaternion.conjugate
        dso3=error_quaternion.axis *error_quaternion.angle
        dX=np.array([(pos_d.pose.position.x-pos.pose.position.x),(pos_d.pose.position.y-pos.pose.position.y),(pos_d.pose.position.z-pos.pose.position.z),dso3[0],dso3[1],dso3[2]])
        return(dX)


    def kuka_robot(self):
        L = [
            RevoluteDH(a=0.0, d=0.360, alpha=-np.pi/2,            qlim=np.array([-self.max_postions[0], self.max_postions[0]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,             qlim=np.array([-self.max_postions[1], self.max_postions[1]])),
            RevoluteDH(a=0.0, d=0.420, alpha=np.pi/2,             qlim=np.array([-self.max_postions[2], self.max_postions[2]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=-np.pi/2,            qlim=np.array([-self.max_postions[3], self.max_postions[3]])),
            RevoluteDH(a=0.0, d=0.400, alpha=-np.pi/2,            qlim=np.array([-self.max_postions[4], self.max_postions[4]])),
            RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,             qlim=np.array([-self.max_postions[5], self.max_postions[5]])),
            RevoluteDH(a=0.0, d=0.281, alpha=np.pi/2,             qlim=np.array([-self.max_postions[6], self.max_postions[6]])),
            RevoluteDH(a=0.2, d=0.0,   alpha=0, offset = np.pi/2, qlim=np.array([-self.max_postions[7], self.max_postions[7]])),
            RevoluteDH(a=0.2, d=0.0,   alpha=0,                   qlim=np.array([-self.max_postions[8], self.max_postions[8]]))
        
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

            print('current pos:')
            print(self.cart_pose.pose.position.x,self.cart_pose.pose.position.y,self.cart_pose.pose.position.z)
            

            # close loop 
            dX_c=self.Pos_error(self.d_cart_pose,self.cart_pose)

            # dq  
            J=self.kuka.jacob0(self.q)
            J_t= np.transpose(J)
            J_p= np.dot(J_t,np.linalg.inv(np.dot(J,J_t)))
            dq= np.dot(J_p,(2.0*dX_c))

            # null space
            dq_N1= np.dot( (np.eye(9) - np.dot(J_p,J)), (self.q_null - self.q))
    
            # final velocity vector
            dq = dq + dq_N1

            
            # limit velocity
            for i in range(9):
                if  abs(dq[i])>self.max_velocity[i]:
                    if (dq[i]<0):
                        dq[i]=-self.max_velocity[i]
                    else:
                        dq[i]=self.max_velocity[i]

            # limit position
            for i in range(9):
                if  (self.q[i]>(self.max_postions[i]-0.0523599)) and (dq[i]>0): # -3 degree offset
                    dq[i]=0
                if  (self.q[i]<-(self.max_postions[i]-0.0523599)) and (dq[i]<0): # +3 degree offset
                    dq[i]=0
                
                 
            self.dq_com_kuka.data = [dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6]]
            self.publisher_kuka.publish(self.dq_com_kuka)

            self.dq_com_mini.data = [dq[8], dq[7]]
            self.publisher_mini.publish(self.dq_com_mini)
                  
            

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





    