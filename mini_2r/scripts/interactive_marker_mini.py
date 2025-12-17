#!/usr/bin/env python3

# This is the first version of the marker node ( still needs to be updated).
# TODO : parameter 'frame_id:world' 'topic/marker_s:mini_pose'

import sys
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
from rosidl_runtime_py import set_message_fields
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from spatialmath.base import transl,rpy2tr
from math import sin

node = None
server = None
menu_handler = MenuHandler()
br = None
counter = 0
marker_pose= PoseStamped()


def pub_Pose():
    global node, marker_pose
    time = node.get_clock().now()
    publisher = node.create_publisher( PoseStamped, '/mini/trajectory_points', 10)
    marker_pose.header.stamp = time.to_msg()
    marker_pose.header.frame_id= 'world'
    publisher.publish(marker_pose)


def processFeedback(feedback):
    global marker_pose
    log_prefix = (
        f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"
    )

    log_mouse = ''
    if feedback.mouse_point_valid:
        log_mouse = (
            f'{feedback.mouse_point.x}, {feedback.mouse_point.y}, '
            f'{feedback.mouse_point.z} in frame {feedback.header.frame_id}'
        )
    marker_pose.pose=feedback.pose


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def saveMarker(int_marker):
    server.insert(int_marker, feedback_callback=processFeedback)


def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def wait_for_initial_pose():
    global marker_pose
    _, smg = wait_for_message(JointState, node,'mini/joint_states') 
    q_int = np.array(smg.position)
    l0 = 0.043 + 0.084
    l1 = 0.2
    l2 = 0.2
    marker = PoseStamped()
    marker.pose.position.y = l1 * math.sin(q_int[0]) + l2 * math.sin(q_int[1]) #X
    marker.pose.position.z = 0.0033+0.1
    marker.pose.position.x = l0 + l1 * math.cos(q_int[0]) + l2 * math.cos(q_int[1]) #Z
    marker.pose.orientation.x = 0.0 #quat[1] Because only 2D, I do not care about orientation
    marker.pose.orientation.y = 0.0 #quat[2]
    marker.pose.orientation.z = 0.0 #quat[3]
    marker.pose.orientation.w = 1.0 #quat[0]
    return(marker)


def make2DofMarker(marker_pose, scale=0.1):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'world' #world
    int_marker.pose= marker_pose.pose
    int_marker.scale = scale

    int_marker.name = 'mini_TCP_2D'
    int_marker.description = 'mini_TCP'
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    normalizeQuaternion(control.orientation)
    control.name = 'move_y'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = 'move_x'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)
    menu_handler.apply(server, int_marker.name)
    


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('mini_pose')
    br = TransformBroadcaster(node)
    node.get_logger().info('le node est lancee')

    server = InteractiveMarkerServer(node, 'mini_pose')

    menu_handler.insert('First Entry', callback=processFeedback)
    menu_handler.insert('Second Entry', callback=processFeedback)
    sub_menu_handle = menu_handler.insert('Submenu')
    menu_handler.insert('First Entry', parent=sub_menu_handle, callback=processFeedback)
    menu_handler.insert('Second Entry', parent=sub_menu_handle, callback=processFeedback)

    node.declare_parameter('tool_xyz', '0 0 0')
    tool_xyz= node.get_parameter('tool_xyz').value
    node.declare_parameter('tool_rpy', '0 0 0')
    tool_rpy= node.get_parameter('tool_rpy').value
    tool_xyz = tool_xyz.split()
    tool_rpy = tool_rpy.split()
    
    marker_pose = PoseStamped()
    marker_pose=wait_for_initial_pose()
    scale = 0.1

    test= make2DofMarker(marker_pose,scale)

    timer = node.create_timer(0.01, pub_Pose)
    server.applyChanges()
    rclpy.spin(node)
    server.shutdown()