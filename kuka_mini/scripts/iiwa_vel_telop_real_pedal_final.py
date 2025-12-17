#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped

import roboticstoolbox as rtb
from pyquaternion import Quaternion
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl,rpy2tr
import numpy as np

from numpy.linalg import inv


from scipy.spatial.transform import Rotation as R

#ros2 run joy joy_node

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

        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/iiwa/velocity_controller/commands',
            10
        )

        self.error_mini_pub = self.create_publisher(
            Float64MultiArray,
            '/error_mini',
            10
        )

        self.fk_mini_world_frame_pub = self.create_publisher(
            Float64MultiArray,
            '/fk_mini_world_frame',
            10
        )


        self.fk_mini_sub = self.create_subscription(
            Float64MultiArray,
            '/fk_mini',
            self.fk_mini_callback,
            10
        )

        self.pedal_sub = self.create_subscription(
            Joy,
            '/joy',
            self.pedal_callback,
            10
        )


        self.pedal_sub = self.create_subscription(
            PoseStamped,
            '/world_T_ee_IM',
            self.world_T_ee_callback,
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

        self.fk_mini_info=Float64MultiArray() 

        self.pedal_on = False
        self.fk_done = False

        self.fk_kuka_ = True

        self.initial_pose_mode = 0.0

        self.d_cart_pose = self.MGD(self.q_int) #To make the initial pose of the robot be the current one

        self.get_logger().info('iiwa_vel Node has been started.')


    def joint_states_callback(self, msg: JointState):
        self.q = np.array(msg.position)

    def world_T_ee_callback(self, msg: PoseStamped):
        self.world_T_ee = msg
    
    
    #Mini's end-effector position by hand, w.r.t the mini's frame
    def fk_mini_callback(self, msg: Float64MultiArray):
        self.fk_mini_info=msg
        # self.get_logger().info(f'FK mini message trial: {msg.data}')

    def pedal_callback (self, msg: Joy):
        self.initial_pose_mode = msg.axes[1]

        if self.initial_pose_mode == 1.0:
            self.pedal_on = True

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

            self.error_mini = np.array([0.0, 0.0])  # Initialize error_mini
            self.fk_mini_wrt_world = np.array([0.0, 0.0]) 

            # update data
            self.cart_pose=self.MGD(self.q)

            if self.initial_pose_mode == 0.0:
                if not self.fk_done:
                    self.d_cart_pose = self.cart_pose
                    self.fk_done = True
                    # print("Waiting for pedal to be pressed to set the initial pose of the robot...")
                    dX_c = self.Pos_error(self.d_cart_pose,self.cart_pose)


            else:
                if self.pedal_on and len(self.fk_mini_info.data) >= 2 and self.fk_done:
                    self.ref_mini = [self.fk_mini_info.data[0], self.fk_mini_info.data[1]]                   

                    self.fk_done = False
                    # self.get_logger().info(f"Mini's end-effector position set to: {self.ref_mini}")
            
                

                if len(self.fk_mini_info.data) >= 2:

                    dX_c = self.Pos_error(self.d_cart_pose,self.cart_pose)

                    quaternion = [self.cart_pose.pose.orientation.x, 
                                  self.cart_pose.pose.orientation.y, 
                                  self.cart_pose.pose.orientation.z, 
                                  self.cart_pose.pose.orientation.w]
                    r = R.from_quat(quaternion)
                    r_matrix = r.as_matrix()

                    transl = [self.cart_pose.pose.position.x,
                              self.cart_pose.pose.position.y,
                              self.cart_pose.pose.position.z,
                              1.0]
                    
                    world_T_tool0 = np.zeros((4,4))
                    world_T_tool0[:3, :3] = r_matrix #upper left 3x3
                    world_T_tool0[:4, 3] = transl

            
                    #To obtain the mini's FK in the world frame ROTATION of -90 about z-axis 
                    tool0_T_ee_mini = np.zeros((4,4))
                    tool0_T_ee_mini[:3, :3] = [[0.0, -1.0, 0.0],
                                                [1.0, 0.0, 0.0],
                                                [0.0, 0.0, 1.0],]
                    tool0_T_ee_mini[3, 3] = 1.0

                    FK_mini_tool0_frame = np.dot(tool0_T_ee_mini, [self.fk_mini_info.data[0], 0.0, self.fk_mini_info.data[1], 1.0])
 

                    fk_mini_in_world_frame = np.dot(world_T_tool0, FK_mini_tool0_frame)
           
                    print("FK mini world frame: ", fk_mini_in_world_frame)

                    # print("FK mini in world frame: ", fk_mini_in_world_frame[0,3], fk_mini_in_world_frame[1,3])
                    m_X_ref = np.dot(tool0_T_ee_mini, [self.ref_mini[0], 0.0, self.ref_mini[1], 1.0])
                    M_X_ref= np.dot(world_T_tool0, m_X_ref)

                    M_error = fk_mini_in_world_frame - M_X_ref

                    # print(M_error.shape)
                    # print("M_ref: ", M_error)
       
                    dX_c[0] = M_error[0]
                    dX_c[1] = M_error[2]
                    dX_c[2] = M_error[1]

                    self.error_mini[0] = M_error[0]
                    self.error_mini[1] = M_error[1]

                    self.fk_mini_wrt_world[0]= fk_mini_in_world_frame[0]
                    self.fk_mini_wrt_world[1]= fk_mini_in_world_frame[1]

                    # print("dX_C: ", dX_c)
            
            self.error_mini_msg = Float64MultiArray()
            self.error_mini_msg.data = [self.error_mini[0], self.error_mini[1], self.get_clock().now().nanoseconds * 1e-9]  #seconds
            self.error_mini_pub.publish(self.error_mini_msg)

            self.fk_mini_world_msg = Float64MultiArray()
            self.fk_mini_world_msg.data = [self.fk_mini_wrt_world[0], self.fk_mini_wrt_world[1], self.get_clock().now().nanoseconds * 1e-9]  #seconds
            self.fk_mini_world_frame_pub.publish(self.fk_mini_world_msg)

            # dq  
            J=self.kuka.jacob0(self.q)
            J_t= np.transpose(J)
            J_p= np.dot(J_t,np.linalg.inv(np.dot(J,J_t)))
            dq= np.dot(J_p,(1.3*dX_c))

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
