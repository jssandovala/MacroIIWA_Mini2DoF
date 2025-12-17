#!/usr/bin/env python3

# This is the first version of the marker node ( still needs to be updated).
# TODO : parameter 'frame_id:world' 'topic/marker_s:mini_pose'

import copy
from math import sin
import sys

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
import rclpy
from rosidl_runtime_py import set_message_fields
from tf2_ros.transform_broadcaster import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
import numpy as np
from pyquaternion import Quaternion
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import transl,rpy2tr


import time

node = None
server = None
menu_handler = MenuHandler()
br = None
counter = 0
marker_pose= PoseStamped()

def pub_Pose():
    global node, marker_pose
    time = node.get_clock().now()
    publisher = node.create_publisher( PoseStamped, '/mini/pose', 10)
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

    marker = PoseStamped()
    # print('hola')
    marker.pose.position.x = -1.0031  # this is when mini is aprox at the middle of its distance --> -0.9181, I changed it to -1.00031 because ot the real macro-mini (offset)
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.4213
    marker.pose.orientation.x = 0.502 #quat[1] Because only 2D, I do not care about orientation
    marker.pose.orientation.y = 0.500#0.7071 #quat[2]
    marker.pose.orientation.z = 0.483 #quat[3]
    marker.pose.orientation.w = -0.514#-0.7071#quat[0]
    return(marker)

def pose_callback2(msg: Float64MultiArray):
        global t_vector_mini_msg
        t_vector_mini_msg=msg


# def make6DofMarker(fixed, interaction_mode, marker_pose, scale, show_6dof=False):
def make2DofMarker(marker_pose, scale=0.1):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'world' #world..  it as tool0 (23 june 10:38pm)
    int_marker.pose= marker_pose.pose
    int_marker.scale = scale

    int_marker.name = 'mini_TCP_2D'
    int_marker.description = 'mini_TCP'

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = 'move_z'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    normalizeQuaternion(control.orientation)
    control.name = 'move_y'
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=processFeedback)
    menu_handler.apply(server, int_marker.name)
    


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('mini_pose')
    br = TransformBroadcaster(node)

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
    marker_pose = wait_for_initial_pose()
    scale = 0.1

    test= make2DofMarker(marker_pose,scale)

    timer = node.create_timer(0.01, pub_Pose)
    
    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()