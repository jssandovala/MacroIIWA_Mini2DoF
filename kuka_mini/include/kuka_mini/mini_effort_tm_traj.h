#ifndef MINI_EFFORT_TM_TRAJ_H
#define MINI_EFFORT_TM_TRAJ_H

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <string>
#include <sstream>

#include "std_msgs/msg/bool.hpp"
#include <iomanip>


class MiniEffortNode : public rclcpp::Node
{
public:
  MiniEffortNode();

private:

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  
  std::string eigen_vector_to_string(const Eigen::VectorXd& vec);

  void update();
  void timer_callback();
  void freeze_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void iiwa_fk_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // variables
  rclcpp::TimerBase::SharedPtr timer_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_ ;
  Eigen::VectorXd dq_ ;

  Eigen::VectorXd d_pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd mini_fk_xz = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd mini_fk_ref_xz = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd mini_fk_frozen = Eigen::VectorXd::Zero(2);
  

  Eigen::VectorXd ee_pose_pinocchio = Eigen::VectorXd::Zero(3);
  
  Eigen::MatrixXd final_jacobian;

  Eigen::Matrix4d world_T_tool0;

  Eigen::Matrix4d tool0_T_ee_mini_desired;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  Eigen::Vector3d iiwa_position;

  Eigen::Vector3d des_pos_mini_wrt_tool0;

  Eigen::Vector4d t_world_frozen;

  Eigen::VectorXd desired_trajectory_ = Eigen::VectorXd::Zero(3);


  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr freeze_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr FK_kuka_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub_;

  //Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fk_publisher_pub;  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_traj_mini_frame_pub; 

  // Mutex for thread safety
  std::mutex data_mutex_;


  // update safety
  bool has_joint_states_ = false;
  bool has_pose_ = false;

  bool mini_initial_pose = true;

  bool freeze_mode_;
  geometry_msgs::msg::PoseStamped frozen_desired_pose_;
  geometry_msgs::msg::PoseStamped d_pose_;

  double x_current_;
  double z_current_;

  rclcpp::Time last_traj_msg_time_;

};

#endif 