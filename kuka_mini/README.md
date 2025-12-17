# Cecilia Orozco and Karine Esmeral 

The following package contains the controllers for the macro and mini robot. 

There is a coupled controller and a decoupled controller.

# Dependencies
**iiwa_ros2:** https://github.com/ICube-Robotics/iiwa_ros2

**EtherCAT Driver:** https://icube-robotics.github.io/ethercat_driver_ros2/

**Pinocchio library:** https://github.com/stack-of-tasks/pinocchio



# Simulation : coupled controller

This launch file executes the coupled command of the robot. The mini robot is controlled in joint impedance, and the macro robot is controlled in velocity.

**Interactive marker**
1. Step 1:
- Launch the Gazebo and rviz simulation:

```bash
	ros2 launch kuka_mini nine_DDL_marker_sim.launch.py 
```
	It might be needed to select "RobotModel, Description Topic, /iiwa/robot_description" in rviz.


# Simulation : decoupled controller

There is a macro passive – mini active (passive-active) mode and a macro active – mini active (active-active) mode. This mode is controlled by a boolean value at step 3.

# Interactive marker
1. Step 1:
- Launch the Gazebo and rviz simulation:

```bash
	ros2 launch kuka_mini kuka_mini_marker_sim.launch.py 
```

	It might be needed to select "RobotModel, Description Topic, /iiwa/robot_description" in rviz.

2. Step 2:
- Run the interactive marker in the simulation:
```bash
	ros2 run kuka_mini interactive_marker_mini_decoupled.py 
```
	
3. Step 3:
- Make the iiwa follow the mini:
```bash
	ros2 topic pub /initial_pose_mode std_msgs/msg/Bool "{data: false}" -1
```

4. Step 4:
- Use the interactive marker from rviz to control the system.

# Trajectory

1. Step 1:
- Launch the Gazebo and rviz simulation:
```bash
	ros2 launch kuka_mini kuka_mini_marker_traj.launch.py 
```

2. Step 2:
- Make the iiwa follow the mini:
```bash
	ros2 topic pub /initial_pose_mode std_msgs/msg/Bool "{data: false}" -1
```

3. Step 3:
- Run the desired trajectory:

	To visually see the trajectory in rviz do "Add, rviz_default_plugins, path, Topic, /executed_path" before running the file.
	
```bash
	ros2 run kuka_mini desired_trajectory 
```
	 
	

# Experimental 

# macro mini (active - active )

```bash
	sudo /etc/init.d/ethercat start 
```

steps: 

Launch the mini robot (mini_2r)

```bash
	ros2 launch mini_2r mini_2r_bringup.launch.py
```

launch the kuka 

```bash
	ros2 launch kuka_mini iiwa.launch.py 
```

launch rviz 

```bash
    ros2 launch kuka_mini kuka_mini_2r_rviz.launch.py 
```



## Active - active using the interactive marker** 

```bash
	ros2 launch kuka_mini active_active_marker.launch.py 
```


Make the iiwa follow the mini:
```bash
	ros2 topic pub /initial_pose_mode std_msgs/msg/Bool "{data: false}" -1
```

## Active-active pedal 

```bash
	ros2 launch kuka_mini active_active_pedal.launch.py
```


# General explanation about each code file
**Header files**
- desired_trajectory.h: For desired_trajectory.cpp.
- mini_effort_tm_traj.h: For mini_effort_traj.cpp.
- mini_effort_tm.h: For the mini control codes.

**Launch files**
- iiwa.launch.py: To connect to the real robot to control it.
- kuka_mini_marker_sim.launch.py: To launch the iiwa and the mini in simulation.
- kuka_mini_marker_traj.launch.py: To launch the iiwa and the mini in simulation for the "Trajectory" section.
- kuka_mini_rviz.launch.py: To launch the iiwa and mini in rviz for the real robots.
- nine_DDL_marker_sim.launch.py : To launch the iiwa and mini for the coupled controller


**Python scripts**
- iiwa_FK_tool_real.py: To obtain the FK of the iiwa in the real robot.
- iiwa_FK_tool_sim.py: To obtain the FK of the iiwa in the simulation.
- iiwa_vel_real_final.py: To control the iiwa when switching between active-passive and active-active in the real robot.
- iiwa_vel_telop_real_pedal_final.py: To control the iiwa in active-passive mode (using the pedal).
- iiwa_vel_telop_real.py: To control the iiwa in active-passive mode (using a topic).
- iiwa_vel_telop_sim.py: To control the iiwa in simulation.
- interactive_marker_mini_real.py: To show the interactive marker in rviz for the experiment in the real robot.
- interactive_marker_mini.py: To show the interactive marker in rviz for the simulation.
- mini_ee_visual.py: To visually see the trajectory in rviz.
- nine_joints.py: For experiments in the real robot. It combines the joint name topics from the real iiwa (7 joints) and mini system (2 joints). This code is already integrated in the kuka_mini_rviz.launch.py launch.

**File for the coupled controller**
- iiwa_joint_speed.py : To control the iiwa joint speed 
- iiwa_mini2r_redundancy_sim.py : To find the jacobian for the 9DDL ( macro +mini)
- mini2r_joint_impedance_sim.py : To control the mini impedance joint


**C++**
- desired_trajectory.cpp: Defined trajectory to be followed by the system.
- mini_effort_tm_real_final.cpp: To control the real mini robot for the "Active-active and active-passive" section.
- mini_effort_tm_real.cpp: To control the real mini robot in the world for the "Active - active using the interactive marker" section.
- mini_effort_tm_sim.cpp: To control the mini system in simulation.
- mini_effort_traj.cpp: To control the mini system in simulation to follow the trajectory defined in desired_trajectory.cpp
- mini_kuka_transform_trial.cpp: It has information used in other codes like the FK of the real mini w.r.t. its frame and FK of the real mini w.r.t. the world.
