#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

import roboticstoolbox as rtb
from pyquaternion import Quaternion
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl,rpy2tr

from geometry_msgs.msg import PoseStamped

from rclpy.wait_for_message import wait_for_message

class iiwa_FKM(Node):
    def __init__(self):

        super().__init__('iiwa_FKM_node')
        self.update_rate = self.create_rate(1000)  
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/iiwa/joint_states',
            self.joint_states_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped, 
            '/iiwa/state/FKM', 
            10
        )

        ret, smg = wait_for_message(JointState, self,'/iiwa/joint_states')
        self.q_int = np.array(smg.position)
        self.q = self.q_int

        self.max_postions = [2.9147, 2.0594, 2.9147, 2.0594 ,2.9147, 2.0594, 3.0194]  
        self.declare_parameter('tool_xyz', '0 0 0')
        tool_xyz= self.get_parameter('tool_xyz').value
        self.declare_parameter('tool_rpy', '0 0 0')
        tool_rpy= self.get_parameter('tool_rpy').value
        tool_xyz = tool_xyz.split()
        tool_rpy = tool_rpy.split()
        self.kuka_robot()
        self.tool=transl(float(tool_xyz[0]), float(tool_xyz[1]), float(tool_xyz[2])) @ rpy2tr(float(tool_rpy[0]), float(tool_rpy[1]), float(tool_rpy[2]))
        self.kuka.tool=self.tool

        self.FK= PoseStamped()      

        self.get_logger().info('FKM Node has been started.')


    def joint_states_callback(self, msg: JointState):
        self.q = np.array(msg.position)

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
    

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            T=self.kuka.fkine(self.q)
            #print(T)

            TT=np.array([[T.n[0],T.o[0],T.a[0],T.t[0]], [T.n[1],T.o[1],T.a[1],T.t[1]], [T.n[2],T.o[2],T.a[2],T.t[2]], [0, 0, 0, 1]])
            quat = Quaternion(matrix=TT)
            self.FK.pose.position.x = TT[0,3]
            self.FK.pose.position.y = TT[1,3]
            self.FK.pose.position.z = TT[2,3]
            self.FK.pose.orientation.x = quat[1]
            self.FK.pose.orientation.y = quat[2]
            self.FK.pose.orientation.z = quat[3]
            self.FK.pose.orientation.w = quat[0]

            self.publisher.publish(self.FK)
            
            pos = self.FK.pose.position
            ori = self.FK.pose.orientation

            msg = (f"Position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] | "
                f"Orientation (quat): [{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}]")
            
            # Add enough trailing spaces to clear any previous line leftovers
            print(f"\r{msg}{' ' * 10}", end='', flush=True)
            

def main():
    rclpy.init()
    FKM = iiwa_FKM()
    try:
        FKM.main_loop()
    except KeyboardInterrupt:   
        pass
    FKM.destroy_node()          
    rclpy.shutdown()
if __name__ == '__main__':
    main()





    