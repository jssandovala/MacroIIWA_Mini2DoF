#!/usr/bin/env python3


import rclpy
import ruckig
import time
import csv 
import os 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from datetime import datetime
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import  Bool
from sensor_msgs.msg import Joy
from datetime import datetime
from ruckig import Result


"""
  The purpose of this node is to compute the Cartesian point that the mini 
  system has to follow in real time for the simulation.  

Karine Esmeral

"""

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Subscription : the cartesian coordinates of the end-effector
        self.position_sub = self.create_subscription(
            PoseStamped, 
            'mini/end_effector_pose_sim', 
            self.end_effector_pose_callback, 
            10)
        

        # Publisher : the desired cartesian coordinates for the mini
        self.publisher = self.create_publisher(
            PoseStamped, 
            'mini/trajectory_points',
              10)
       
        # Initial cartesian position of the end-effector
        _,msg = wait_for_message(PoseStamped,self, 'mini/end_effector_pose_sim') 
        self.end_effector_x = msg.pose.position.x
        self.end_effector_y = msg.pose.position.y
        self.end_effector_z = msg.pose.position.z
        

        # Ruckig setup
        self.degrees_of_freedom = 3  # x, y, z
        self.dt = 0.01  # Time step in seconds
        self.otg = ruckig.Ruckig(self.degrees_of_freedom,self.dt)
        self.inp = ruckig.InputParameter(self.degrees_of_freedom)
        self.out = ruckig.OutputParameter(self.degrees_of_freedom)

        # Initial state
        self.current_position = [self.end_effector_x, self.end_effector_y, self.end_effector_z]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_acceleration = [0.0, 0.0, 0.0]

        self.dx = 0.08  #  m step in x
        self.dy = 0.08  #  m step in y

        self.first_point = [self.end_effector_x, self.end_effector_y, self.end_effector_z]

    
        # Target positions to visit
        self.target_positions = [
            #Square pattern
            #[self.first_point[0], self.first_point[1] + self.dy, self.first_point[2]],  # second point
            #[self.first_point[0]-self.dx, self.first_point[1]+self.dy, self.first_point[2]],  # third point
            #[self.first_point[0]-self.dx, self.first_point[1], self.first_point[2]],  # fourth point
            #[self.first_point[0], self.first_point[1], self.first_point[2]],  # return to first point
          
            [0.5,   0.1, 0.0033],    # approach
            [0.45,  0.1, 0.0033],    # first point
            [0.35,  0.1, 0.0033],    # second point
            [0.35, -0.1, 0.0033],    # third point
            [0.45, -0.1, 0.0033],    # fourth point
            [0.45,  0.1, 0.0033],    # return to first point
        ]
        self.current_target_index = 0


        # CSV File 
        now = datetime.now()
        folder = "/home/amir/ros2_ws/src/mini_2r/data"
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = f"trajectory_data_{timestamp}.csv"
        self.path = os.path.join(folder,self.csv_filename)

        with open(self.path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time', 'p_x', 'p_y', 'p_z', 
                                'd_x', 'd_y', 'd_z'])
        

    def end_effector_pose_callback(self, msg: PoseStamped):
        self.end_effector_x = msg.pose.position.x
        self.end_effector_y = msg.pose.position.y
        self.end_effector_z = msg.pose.position.z


    def update_trajectory(self):

        while rclpy.ok() and self.current_target_index < len(self.target_positions):
            rclpy.spin_once(self)

            self.inp.current_position = self.current_position
            self.inp.current_velocity = self.current_velocity
            self.inp.current_acceleration = self.current_acceleration


            self.inp.target_position = self.target_positions[self.current_target_index]
            self.inp.target_velocity = [0.0, 0.0, 0.0]
            self.inp.target_acceleration = [0.0, 0.0, 0.0]
        
            self.inp.max_velocity = [0.2 , 0.2 , 0.2 ] # Units: m/s
            self.inp.max_acceleration = [5.0,5.0,5.0]  # Units: m/s^2
            self.inp.max_jerk =[10.0,10.0,10.0]        # Units: m/s^3

            result = self.otg.update(self.inp, self.out)

            self.current_position = self.out.new_position
            self.current_velocity = self.out.new_velocity
            self.current_acceleration = self.out.new_acceleration

            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose.position.x = self.current_position[0]
            pose_msg.pose.position.y = self.current_position[1]
            pose_msg.pose.position.z = self.current_position[2]
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            self.publisher.publish(pose_msg)

            with open(self.path, mode='a', newline='') as file:
                writer = csv.writer(file)
                current_time = self.get_clock().now().to_msg()
                writer.writerow([
                    f"{current_time.sec}.{current_time.nanosec:09d}",
                    self.end_effector_x, # Real-time end-effector position
                    self.end_effector_y,
                    self.end_effector_z,
                    self.current_position[0],
                    self.current_position[1],
                    self.current_position[2],
                ])

            error_x = self.target_positions[self.current_target_index][0] - self.end_effector_x
            error_y = self.target_positions[self.current_target_index][1] - self.end_effector_y


            if abs(error_x) < 0.001 and abs(error_y) < 0.001 and result == Result.Finished :  # 1 mm tolerance
                self.get_logger().info(f"Reached target position {self.target_positions[self.current_target_index]} and error is < 1 mm")
                self.current_target_index += 1
                time.sleep(1)  # Pause for a second before moving to the next target

            elif abs(error_x) < 0.01 and abs(error_y) < 0.01 and result == Result.Finished :  # 1 cm tolerance
                self.get_logger().info(f"Reached target position {self.target_positions[self.current_target_index]} and error is < 1 cm")
                self.current_target_index += 1
                time.sleep(1)  # Pause for a second before moving to the next target

            elif abs(error_x) < 0.02 and abs(error_y) < 0.02 and result == Result.Finished :  # 2 cm tolerance
                self.get_logger().info(f"Reached target position {self.target_positions[self.current_target_index]} and error is < 2 cm")
                self.current_target_index += 1
                time.sleep(1)  # Pause for a second before moving to the next target
            
            time.sleep(self.dt)


def main():
    rclpy.init()
    node = TrajectoryGeneratorNode()
    try: 
        node.update_trajectory()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()