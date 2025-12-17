#!/usr/bin/env python3

import copy
from math import sin
import sys
import math

from geometry_msgs.msg import PoseStamped
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
from roboticstoolbox import DHRobot, RevoluteMDH,RevoluteDH
from spatialmath.base import transl,rpy2tr

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
    marker_pose.header.frame_id= 'iiwa_base'
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




def iiwa_mini_robot():
         # jointa1,jointa2,jointa3,jointa4,jointa5,jointa6,jointa7,joint2,joint1
        max_positions = [2.9147, 2.0594, 1.5708, 1.5708 ,1.5708, 1.5708, 3.0194, 3.1416, 3.1416] 
        
        L = [
            RevoluteMDH(a=0.0, d=0.360, alpha=0.0,                      qlim=np.array([-max_positions[0], max_positions[0]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=-np.pi/2,                 qlim=np.array([-max_positions[1], max_positions[1]])),
            RevoluteMDH(a=0.0, d=0.420, alpha=np.pi/2,                  qlim=np.array([-max_positions[2], max_positions[2]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=np.pi/2,                  qlim=np.array([-max_positions[3], max_positions[3]])),
            RevoluteMDH(a=0.0, d=0.400, alpha=-np.pi/2,                 qlim=np.array([-max_positions[4], max_positions[4]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=-np.pi/2,                 qlim=np.array([-max_positions[5], max_positions[5]])),
            RevoluteMDH(a=0.0, d=0.281, alpha=np.pi/2,  offset = np.pi,  qlim=np.array([-max_positions[6], max_positions[6]])),
            RevoluteMDH(a=0.0, d=0.0,   alpha=np.pi/2, offset = np.pi/2, qlim=np.array([-max_positions[7], max_positions[7]])),
            RevoluteMDH(a=0.2, d=0.0,   alpha=0,                        qlim=np.array([-max_positions[8], max_positions[8]]))


            #RevoluteDH(a=0.0, d=0.360, alpha=-np.pi/2,                qlim=np.array([max_positions[0], max_positions[0]])),
            #RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,                 qlim=np.array([max_positions[1], max_positions[1]])),
            #RevoluteDH(a=0.0, d=0.420, alpha=np.pi/2,                 qlim=np.array([max_positions[2], max_positions[2]])),
            #RevoluteDH(a=0.0, d=0.0,   alpha=-np.pi/2,                qlim=np.array([max_positions[3], max_positions[3]])),
            #RevoluteDH(a=0.0, d=0.400, alpha=-np.pi/2,                qlim=np.array([max_positions[4], max_positions[4]])),
            #RevoluteDH(a=0.0, d=0.0,   alpha=np.pi/2,                 qlim=np.array([max_positions[5], max_positions[5]])),
            #RevoluteDH(a=0.0, d=0.281, alpha=np.pi/2,  offset = np.pi, qlim=np.array([max_positions[6], max_positions[6]])),
            #RevoluteDH(a=0.2, d=0.0,   alpha=0.0, offset = np.pi/2,  qlim=np.array([max_positions[7], max_positions[7]])),
            #RevoluteDH(a=0.0, d=0.0,   alpha=0,                      qlim=np.array([max_positions[8], max_positions[8]])) 
            
        ]
        iiwa_mini = DHRobot( L, name="iiwa_mini" )
        return(iiwa_mini)

def wait_for_initial_pose():
    global marker_pose
    _, smg = wait_for_message(JointState, node,'iiwa/joint_states') 
    #q_int = np.array(smg.position)
    q_int = [smg.position[2],
             smg.position[3],
             smg.position[4],
             smg.position[5],
             smg.position[6],
             smg.position[7],
             smg.position[8],
             smg.position[0],
             smg.position[1],
             ]
    kuka=iiwa_mini_robot()
    kuka.tool=transl(float(tool_xyz[0]), float(tool_xyz[1]), float(tool_xyz[2])) @ rpy2tr(float(tool_rpy[0]), float(tool_rpy[1]), float(tool_rpy[2]))
    T=kuka.fkine(q_int)
    #TT = T.A
 
    TT=np.array([[T.n[0],T.o[0],T.a[0],T.t[0]], [T.n[1],T.o[1],T.a[1],T.t[1]], [T.n[2],T.o[2],T.a[2],T.t[2]], [0, 0, 0, 1]])
    quat = Quaternion(matrix=TT)
    marker = PoseStamped()
    marker.pose.position.x = TT[0,3]
    marker.pose.position.y = TT[1,3]
    marker.pose.position.z = TT[2,3]
    marker.pose.orientation.x = quat[1]
    marker.pose.orientation.y = quat[2]
    marker.pose.orientation.z = quat[3]
    marker.pose.orientation.w = quat[0]

    return(marker)



def make6DofMarker(fixed, interaction_mode, marker_pose, scale, show_6dof=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'iiwa_base'
    int_marker.pose= marker_pose.pose

    int_marker.scale = scale

    int_marker.name = 'iiwa_TCP'
    int_marker.description = 'iiwa_TCP'

    # insert a box
    #makeBoxControl(int_marker)
    #int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += '_fixed'
        int_marker.description += '\n(fixed orientation)'

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: 'MOVE_3D',
            InteractiveMarkerControl.ROTATE_3D: 'ROTATE_3D',
            InteractiveMarkerControl.MOVE_ROTATE_3D: 'MOVE_ROTATE_3D'
        }
        int_marker.name += '_' + control_modes_dict[interaction_mode]
        int_marker.description = '3D Control'
        if show_6dof:
            int_marker.description += ' iiwa'
        int_marker.description += '\n' + control_modes_dict[interaction_mode]

    if show_6dof:
        """control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)"""

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        """control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)"""

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        """control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        normalizeQuaternion(control.orientation)
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)"""

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        normalizeQuaternion(control.orientation)
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
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

    node.declare_parameter('tool_xyz', '0.2 0 0')
    tool_xyz= node.get_parameter('tool_xyz').value
    node.declare_parameter('tool_rpy', '0 0 0')
    tool_rpy= node.get_parameter('tool_rpy').value
    tool_xyz = tool_xyz.split()
    tool_rpy = tool_rpy.split()
    
    marker_pose = PoseStamped()
    marker_pose=wait_for_initial_pose()
    scale = 0.1

    test=  make6DofMarker(False, InteractiveMarkerControl.MOVE_ROTATE_3D, marker_pose,scale, True)
    #make6DofMarker(False, InteractiveMarkerControl.NONE, marker_pose,scale, True)


    timer = node.create_timer(0.01, pub_Pose)
    
    server.applyChanges()

    rclpy.spin(node)
    server.shutdown()
