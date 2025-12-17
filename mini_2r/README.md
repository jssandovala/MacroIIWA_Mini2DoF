# Amir TRABELSI LS2N and Karine Esmeral 

The following package contains the controllers for the mini robot. 

# Dependencies
**iiwa_ros2:** https://github.com/ICube-Robotics/iiwa_ros2

**EtherCAT Driver:** https://icube-robotics.github.io/ethercat_driver_ros2/

**Pinocchio library:** https://github.com/stack-of-tasks/pinocchio



# Start the communication and visualization with the real robot
1. step 1
- start the EtherCAT master:
```bash
	sudo /etc/init.d/ethercat start 
```
check connected slaves:
	ethercat slaves 

2. step 2 
- lauch the real robot ( ethercat comunication for controller and socket for encoder )
```bash
	ros2 launch mini_2r mini_2r_bringup.launch.py
```
	 

3. step 3 
- launch the rviz
```bash
	ros2 launch mini_2r mini_rviz.launch.py 
```


# Real Robot 

**To move the mini following a trajectory**
1. Step 1: 
-  Follow the steps in the section ‘Start the communication and visualization with the real robot'

2. Step 2: 
- Launch the controller of the mini:
```bash
	ros2 launch mini_2r mini_control_trajectory.launch.py 
```
	In the launch file, you can comment out the trajectory generator and uncomment the marker if you want to control the actual robot using a marker. 


# Simulation

**To move the  mini with the interactive marker** :

1. Step 1:
- Launch the Gazebo and rviz simulation:
```bash
	ros2 launch mini_2r mini_control_sim.launch.py 
```

	It might be needed to select "InteractiveMarkers, Interactive Markers Namespace, /mini_pose_sim" in rviz.
	In the launch file, you can comment out the marker and uncomment the mini_speed_impedance if you want to control the actual robot using a impedance that have as input the joint desired. 

**To move the mini following a trajectory** : 

1. Step 1:
- Launch the Gazebo and rviz simulation:
```bash
	ros2 launch mini_2r mini_control_trajectory_sim.launch.py 
```
	


# General explanation about each code file 

**Header files**
- mini_impedance.h : 
For mini_impedance_sim.cpp, 
mini_impedance_trajectory_sim.cpp,
mini_impedance_trajectory.cpp

**Launch files**
- mini_2r_bringup.launch.py : To connect to the real robot to control it
- mini_control_sim.launch.py: To control the mini with the interactive marker in simulation 
- mini_control_trajectory_sim.launch.py : To control the mini following a trajectory in simulation 
- mini_control_trajectory.launch.py : To control the mini following a trajectory in real 
- mini_impedance_trajectory_sim.launch.py : To control the mini with impedance position control in     	simulation 
- mini_impedance_trajectory.launch.py : To control the mini with impedance position control in real
- mini_rviz.launch.py : To connect to the rviz environment
- rrbot_gazebo.launch.py : To connect to the gazebo environment


**Python scripts**
- data_plot_joints.py : To plot the joints positions of the mini
- data_plot.py : To plot the cartesian position of the mini
- generator_trajectory.py: To generate the trajectory in real time  for the mini 
- generator_trajectory_sim.py: To generate the trajectory in real time  for the mini for the simulation
- interactive_marker_mini_sim.py : The marker to use to control the mini in simulation 
- interactive_marker_mini.py : The marker to use to control the mini in real
- mini_2r_position_sim.py : To have the position of the end effector of the mini for simulation
- mini_2r_position.py : To have the position of the end effector of the mini in real 
- mini_speed_and_impedance.py : To test the mini2r with a cartesian controller and joint controller with velocity as input
- mini_state_fusion.py : To filter disturbances in the joint values 
- RR_joint_states.py : To have the value of the real joints of the mini
- trajectoty_plot.py : To plot the trajectory of the mini and the desired trajectory 

**C++**
- mini_impedance_sim.cpp : To control the impedance position the mini in the simulation 
- mini_impedance_trajectory_sim.cpp : To control the impedance position the mini in real
- mini_impedance_trajectory.cpp : To control the impedance for the trajectory 

