#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped 
from rclpy.wait_for_message import wait_for_message
import matplotlib.pyplot as plt 
import numpy as np 

 

class TrajectoryPlotter(Node): 
    def __init__(self): 
        super().__init__('trajectory_plotter') 

        self.subscription_fkm = self.create_subscription( 
            PoseStamped, 
            'mini/end_effector_pose', 
            self.pose_fkm_callback, 
            10) 

        self.subscription_pose = self.create_subscription( 
            PoseStamped, 
            'mini/trajectory_points', 
            self.pose_pose_callback, 
            10) 

        # Intial value of the end-effector
        _,msg = wait_for_message(PoseStamped,self, 'mini/end_effector_pose',) 
        self.pos_FKM_int= [msg.pose.position.x, msg.pose.position.y]
        

        # The list of the actual and desired position 
        self.x_points_fkm = [] 
        self.y_points_fkm = [] 
        self.x_points_pose = [] 
        self.y_points_pose = [] 


        # Set up the plot 
        plt.ion() # Turn on interactive mode 
        self.fig, self.ax = plt.subplots() 
        self.line_fkm, = self.ax.plot([], [], 'b-', lw=2, label='FKM') 
        self.line_pose, = self.ax.plot([], [], 'r-', lw=2, label='trajectory') 
        self.ax.set_xlabel('X') 
        self.ax.set_ylabel('Y') 
        self.ax.set_title('Real-Time Trajectories') 
        self.ax.grid(True) 
        self.ax.set_xlim(self.pos_FKM_int[0]-0.10, self.pos_FKM_int[0]+0.10) #0.33,0.47
        self.ax.set_ylim(self.pos_FKM_int[1]-0.10, self.pos_FKM_int[1]+0.10) #-0.13,0.13
        self.ax.legend() 


    def pose_fkm_callback(self, msg): 
        x = msg.pose.position.x 
        y = msg.pose.position.y 

        self.x_points_fkm.append(x) 
        self.y_points_fkm.append(y) 
        self.update_plot() 

    
    def pose_pose_callback(self, msg): 
        x = msg.pose.position.x 
        y = msg.pose.position.y 

        self.x_points_pose.append(x) 
        self.y_points_pose.append(y) 
        self.update_plot() 


    def update_plot(self): 

        self.line_fkm.set_data(self.x_points_fkm, self.y_points_fkm) 
        self.line_pose.set_data(self.x_points_pose, self.y_points_pose) 
        self.ax.relim() 
        self.ax.autoscale_view() 
        self.fig.canvas.draw() 
        self.fig.canvas.flush_events() 

    

def main(args=None): 
    rclpy.init(args=args) 
    node = TrajectoryPlotter() 
    executor = rclpy.executors.SingleThreadedExecutor() 
    executor.add_node(node) 
    try: 
        executor.spin() 
    except KeyboardInterrupt: 
        pass 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 

 