#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class FKPathPublisher(Node):
    def __init__(self):
        super().__init__('fk_path_publisher')

        self.path_pub = self.create_publisher(Path, '/executed_path', 10)

        self.mini_ee_pose_world_sub = self.create_subscription(
            PoseStamped,
            '/world_T_ee_IM',
            self.ee_pose_callback,
            10
        )

        self.pose_list = []  # accumulate poses here
        self.fixed_frame = 'world'
        self.Mm_pose_ = None 
        self.create_timer(0.1, self.ee_pose_info)

    def ee_pose_callback(self, msg: PoseStamped):
        self.Mm_pose_ = msg

    def ee_pose_info(self):
        if self.Mm_pose_ is None:
            # self.get_logger().warn("No pose received yet.")
            return
        
        # self.get_logger().info("Received FK update")

        ee_pose_stamped = PoseStamped()
        ee_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ee_pose_stamped.header.frame_id = self.fixed_frame

        ee_pose_stamped.pose.position.x = self.Mm_pose_.pose.position.x
        ee_pose_stamped.pose.position.y = self.Mm_pose_.pose.position.y
        ee_pose_stamped.pose.position.z = self.Mm_pose_.pose.position.z
        ee_pose_stamped.pose.orientation.x = self.Mm_pose_.pose.orientation.x
        ee_pose_stamped.pose.orientation.y = self.Mm_pose_.pose.orientation.y
        ee_pose_stamped.pose.orientation.z = self.Mm_pose_.pose.orientation.z
        ee_pose_stamped.pose.orientation.w = self.Mm_pose_.pose.orientation.w

        self.pose_list.append(ee_pose_stamped)
        path_msg = Path()
        path_msg.header.stamp = ee_pose_stamped.header.stamp
        path_msg.header.frame_id = self.fixed_frame
        path_msg.poses = self.pose_list.copy()

        self.path_pub.publish(path_msg)
        # self.get_logger().info(f"Published path with {len(self.pose_list)} poses.")
        # print(f'Desired pose: {ee_pose_stamped.pose.position.x}, {ee_pose_stamped.pose.position.y}, {ee_pose_stamped.pose.position.z}')


def main():
    rclpy.init()
    node = FKPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()