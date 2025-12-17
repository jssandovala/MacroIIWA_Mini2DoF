#ifndef MINI_IMPEDANCE_H
#define MINI_IMPEDANCE_H

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
#include <fstream> 
class MiniImpedanceNode : public rclcpp::Node
{
public:
  MiniImpedanceNode();

private:
  std::ofstream csv_file_;
  std::string cleanDate(const char* date) {
    std::string d(date);
    for (auto& c : d) {
        if (c == ' ' || c == ':') c = '_';
    }
    return d;
 }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  
  std::string eigen_vector_to_string(const Eigen::VectorXd& vec);
  void update();
  void timer_callback();

 
  rclcpp::TimerBase::SharedPtr timer_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_ ;
  Eigen::VectorXd dq_ ;
  Eigen::VectorXd d_pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd pos_ = Eigen::VectorXd::Zero(3);
  Eigen::Matrix4d tool0_T_ee_mini_desired;
  Eigen::Matrix4d world_T_tool0;

 
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub_;


  //publichers 
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_torque_pub_;
 

  std::mutex data_mutex_;
  bool has_joint_states_ = false;
  bool has_pose_ = false;
  rclcpp::Time last_traj_msg_time_;

  
};

#endif 