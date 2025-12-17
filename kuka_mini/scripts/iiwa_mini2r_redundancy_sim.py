#!/usr/bin/env python3

import rclpy 
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from roboticstoolbox import DHRobot, RevoluteMDH,RevoluteDH
from rclpy.wait_for_message import wait_for_message
from pyquaternion import Quaternion
from spatialmath.base import transl,rpy2tr

"""
    The purpose of this node is to compute the velocity for all the joints
    of the robot (macro and mini together). The joints jointa1 to jointa7 
    are associated with the macro robot, while joint1 and joint2 are associated 
    with the mini robot.

Karine Esmeral
"""

class Redundancy(Node):
    def __init__(self):
        super().__init__('redundancy_control')

        # Subscription : the value for all the joints for the iiwa and mini from the gazebo
        self.joints_subscriber = self.create_subscription(
            JointState,
            '/iiwa/joint_states',
            self.joints_state_callback,
            10
        )

        # Subscription : the desired position of the end-effector from the interactive marker
        self.marker_subscriber = self.create_subscription(
            PoseStamped,
            '/mini/pose',
            self.marker_callback,
            10
        )


        # Publisher: the  value of  the velocity commands to the iiwa and mini
        self.delta_joint_desired = self.create_publisher(
            Float64MultiArray,
            '/iiwa/delta_joint',
            10
        )

        # Order of  joints : jointa1,jointa2,jointa3,jointa4,jointa5,jointa6,jointa7,joint2,joint1
        self.max_positions = [2.9147, 2.0594, 1.5708, 1.5708 ,1.5708, 1.5708, 3.0194, 2.00, 3.1416]  
        self.max_velocity = [1.5, 1.5, 1.5, 2.0, 2.0, 3.0, 3.0, 10.0, 10.0]  

        # Initial joint positions 
        _,msg = wait_for_message(JointState,self,'/iiwa/joint_states') 
        self.q = np.array((msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7], msg.position[8], msg.position[0], msg.position[1]))
        self.q_null = np.array((msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7], msg.position[8], msg.position[0], msg.position[1]))
        

        # The initial pose of the interactive marker
        _,msg_marker = wait_for_message(PoseStamped,self,'/mini/pose') 
        self.d_pos = msg_marker 

        # Position and orientation of the tool”
        self.declare_parameter('tool_xyz', '0.2 0 0')
        tool_xyz= self.get_parameter('tool_xyz').value
        self.declare_parameter('tool_rpy', '0 0 0')
        tool_rpy= self.get_parameter('tool_rpy').value
        tool_xyz = tool_xyz.split()
        tool_rpy = tool_rpy.split()

        self.iiwa_mini_robot()
        self.tool=transl(float(tool_xyz[0]), float(tool_xyz[1]), float(tool_xyz[2])) @ rpy2tr(float(tool_rpy[0]), float(tool_rpy[1]), float(tool_rpy[2]))
        self.iiwa_mini.tool=self.tool
    

    def joints_state_callback(self, msg: JointState):
       
        self.q = np.array([
                        msg.position[2], 
                        msg.position[3],
                        msg.position[4], 
                        msg.position[5],
                        msg.position[6], 
                        msg.position[7],
                        msg.position[8], 
                        msg.position[0], 
                        msg.position[1]
                        ])
           
    def marker_callback(self, msg: PoseStamped):
            self.d_pos = msg


    # The DH parameters of the macro and mini robot according to Bruno Siciliano’s convention
    def iiwa_mini_robot(self):
        L = [
            RevoluteMDH(a=0.0, d=0.360, alpha=0.0,                       qlim=np.array([-self.max_positions[0], self.max_positions[0]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=-np.pi/2,                  qlim=np.array([-self.max_positions[1], self.max_positions[1]])),
            RevoluteMDH(a=0.0, d=0.420, alpha=np.pi/2,                   qlim=np.array([-self.max_positions[2], self.max_positions[2]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=np.pi/2,                   qlim=np.array([-self.max_positions[3], self.max_positions[3]])),
            RevoluteMDH(a=0.0, d=0.400, alpha=-np.pi/2,                  qlim=np.array([-self.max_positions[4], self.max_positions[4]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=-np.pi/2,                  qlim=np.array([-self.max_positions[5], self.max_positions[5]])),
            RevoluteMDH(a=0.0, d=0.281, alpha=np.pi/2, offset =np.pi ,    qlim=np.array([-self.max_positions[6], self.max_positions[6]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=np.pi/2, offset = np.pi/2, qlim=np.array([-self.max_positions[7], self.max_positions[7]])),
            RevoluteMDH(a=0.2, d=0.0,   alpha=0,                         qlim=np.array([-self.max_positions[8], self.max_positions[8]]))
            ]
        self.iiwa_mini = DHRobot( L, name="iiwa_mini" )

   
    """
    # The modified DH parameters of the macro and mini robot according to Craig's convention
    
        RevoluteDH(a=0.0, d=0.360, alpha=-np.pi/2,                    qlim=np.array([-self.max_positions[0], self.max_positions[0]])),
        RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,                     qlim=np.array([-self.max_positions[1], self.max_positions[1]])),
        RevoluteDH(a=0.0, d=0.420, alpha=np.pi/2,                     qlim=np.array([-self.max_positions[2], self.max_positions[2]])),
        RevoluteDH(a=0.0, d=0.0,   alpha=-np.pi/2,                    qlim=np.array([-self.max_positions[3], self.max_positions[3]])),
        RevoluteDH(a=0.0, d=0.400, alpha=-np.pi/2,                    qlim=np.array([-self.max_positions[4], self.max_positions[4]])),
        RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,                     qlim=np.array([-self.max_positions[5], self.max_positions[5]])),
        RevoluteDH(a=0.0, d=0.281, alpha=np.pi/2,  offset =np.pi ,    qlim=np.array([-self.max_positions[6], self.max_positions[6]])),
        RevoluteDH(a=0.2, d=0.0,   alpha=0.0, offset = np.pi/2,       qlim=np.array([-self.max_positions[7], self.max_positions[7]])),
        RevoluteDH(a=0.0, d=0.0,   alpha=0,                           qlim=np.array([-self.max_positions[8], self.max_positions[8]]))

    """ 


    def Pos_error (self, pos_d ,pos):
        target_quaternion = Quaternion(pos_d.pose.orientation.w, pos_d.pose.orientation.x, pos_d.pose.orientation.y, pos_d.pose.orientation.z)
        current_quaternion= Quaternion (pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z)
        error_quaternion= target_quaternion * current_quaternion.conjugate
        dso3=error_quaternion.axis *error_quaternion.angle
        dX=np.array([(pos_d.pose.position.x-pos.pose.position.x),(pos_d.pose.position.y-pos.pose.position.y),(pos_d.pose.position.z-pos.pose.position.z),dso3[0],dso3[1],dso3[2]])#0.0,0.0,0.0])
        return(dX)

    def MGD(self,q):
        FK= self.iiwa_mini.fkine(q)
        # Homogeneous matrix of the end-effector
        TT=np.array([[FK.n[0],FK.o[0],FK.a[0],FK.t[0]], [FK.n[1],FK.o[1],FK.a[1],FK.t[1]], [FK.n[2],FK.o[2],FK.a[2],FK.t[2]], [0, 0, 0, 1]])
        quat = Quaternion(matrix=TT)
        pos = PoseStamped()
        pos.pose.position.x = TT[0,3]
        pos.pose.position.y = TT[1,3]
        pos.pose.position.z = TT[2,3]
        pos.pose.orientation.x = quat[1]
        pos.pose.orientation.y = quat[2]
        pos.pose.orientation.z = quat[3]
        pos.pose.orientation.w = quat[0]

        return pos
    

    def jacobian(self,q):
        # Jacobian
        J = self.iiwa_mini.jacob0(q)
        J_t = np.transpose(J)

        # Weighting matrix 
        macro = 1.0
        mini = 1.0
        W= np.array((
                     [macro,0,0,0,0,0,0,0,0],
                     [0,macro,0,0,0,0,0,0,0],
                     [0,0,macro,0,0,0,0,0,0],
                     [0,0,0,macro,0,0,0,0,0],
                     [0,0,0,0,macro,0,0,0,0],
                     [0,0,0,0,0,macro,0,0,0],
                     [0,0,0,0,0,0,macro,0,0],
                     [0,0,0,0,0,0,0,mini,0],
                     [0,0,0,0,0,0,0,0,mini]
                     ))
        
        W_inv= np.linalg.inv(W)
        J_pw1 = np.dot(W_inv, J_t)
        J_pw2 = np.linalg.inv(np.dot(J, J_pw1))
        J_pw = np.dot(J_pw1, J_pw2)

        return J_pw,J

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
           
            pos_actual = self.MGD(self.q)
            J_pw,J     = self.jacobian(self.q)
            dX_c       = self.Pos_error(self.d_pos, pos_actual)
        
            matrice_indentity = np.eye(9)
            error_joint = self.q_null - self.q 
            null_space = np.dot((matrice_indentity-np.dot(J_pw,J)),error_joint)
            dq = np.dot(J_pw, dX_c) + null_space
           
            # limit velocity
            for i in range(9):
                if  abs(dq[i])>self.max_velocity[i]:
                    if (dq[i]<0):
                        dq[i]=-self.max_velocity[i]
                    else:
                        dq[i]=self.max_velocity[i]

            # limit position
            for i in range(9):
                if  (self.q[i]>(self.max_positions[i]-0.0523599)) and (dq[i]>0): # -3 degree offset
                    dq[i]=0
                if  (self.q[i]<-(self.max_positions[i]-0.0523599)) and (dq[i]<0): # +3 degree offset
                    dq[i]=0
                

            dq_msg = Float64MultiArray()
            dq_msg.data = [dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6], dq[7], dq[8]]
            self.delta_joint_desired.publish(dq_msg)


def main():
    rclpy.init()
    redundancy_joint = Redundancy()
    try:
        redundancy_joint.main_loop()
    except KeyboardInterrupt:   
        pass
    redundancy_joint.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
       


        


    