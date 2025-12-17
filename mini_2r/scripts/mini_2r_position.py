#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


"""
  The purpose of this node is to compute the cartesian position of the end-effector 
  of the mini for the simulation.  

Karine Esmeral

"""

class Kinematics_sim(Node):
    def __init__(self):
        super().__init__('kinematics')

        # Subscription : The joints value of the mini robot
        self.subscription = self.create_subscription(
            JointState,
            'mini/joint_states',
            self.joint_state_callback,10) 

        # Publisher : The cartesian position of the end-effector 
        self.publisher = self.create_publisher(
            PoseStamped,
            'mini/end_effector_pose', 10)
        
        # The lengths of the mini robot
        self.l0 = 0.043 + 0.084
        self.l1 = 0.2
        self.l2 = 0.2



    def joint_state_callback(self, msg: JointState):

       
        if len(msg.position) < 2:
            self.get_logger().warn('Error  mini/joint_states !')
            return

        q0 = msg.position[0]
        q1 = msg.position[1]

        # The kinematic
        x = self.l0 + self.l1 * math.cos(q0) + self.l2 * math.cos(q1) 
        y = self.l1 * math.sin(q0)+ self.l2 *math.sin(q1)             
        z = 0.0033                                          

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"  
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Kinematics_sim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
