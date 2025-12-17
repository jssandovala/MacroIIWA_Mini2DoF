#include "mini_2r/mini_impedance.h"
#include <iostream>
using namespace std;
#include <vector>

#include <fstream>   
#include <chrono>  


MiniImpedanceNode::MiniImpedanceNode() : Node("mini_impedance_node")
{ 
  
  this->declare_parameter("urdf_path", "mini.urdf");
  std::string urdf_path = this->get_parameter("urdf_path").as_string();

  // create pinocchio model from urdf
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", e.what());
    rclcpp::shutdown();
    return;
  }
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model_.gravity.linear(gravity);
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/mini/joint_states", 10, 
    std::bind(&MiniImpedanceNode::joint_state_callback, this, std::placeholders::_1));

  // trajectory subscriber
  traj_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mini/trajectory_points", 10,
    std::bind(&MiniImpedanceNode::trajectory_callback, this, std::placeholders::_1));


  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_effort_controller/commands", 10);


 

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(1000),  // microseconds >> 1000hz
    std::bind(&MiniImpedanceNode::timer_callback, this));


    std::string date_str = cleanDate(__DATE__);
    std::string time_str = cleanDate(__TIME__);
  
    /*csv_file_.open("/home/amir/ros2_ws/src/mini_2r/data/joints_"+ date_str+"_"+time_str+".csv", std::ios::out | std::ios::trunc);
  
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
    } else {
      csv_file_ << "time,q_0,q_1,d_q0,d_q1\n";  // header
    }*/
 
}


void MiniImpedanceNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (q_.size() != static_cast<int>(msg->position.size())) {
    q_.resize(msg->position.size());
    dq_.resize(msg->velocity.size());

  }

  for (size_t i = 0; i < msg->position.size(); ++i) {
    q_(i) = msg->position[i];
    dq_(i) = msg->velocity[i];
  }

  has_joint_states_ = true;  
  
}



void MiniImpedanceNode::trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  d_pos_<< msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  has_pose_ = true;
  last_traj_msg_time_ = this->get_clock()->now();
 
}



void MiniImpedanceNode::timer_callback()
{
  // to run the update at 1KHz
  update();
}



void MiniImpedanceNode::update()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Safety checks
  if (!has_joint_states_ || !has_pose_) {
    RCLCPP_WARN(this->get_logger(), "Missing data: %s%s",
                !has_joint_states_ ? "joint_states " : "",
                !has_pose_ ? "pose" : "");
    return;
  }

  if (q_.size() < 2 || dq_.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "q_ or dq_ size < 2, skipping update");
    return;
  }

  // Gravity torques
  Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(model_, data_, q_);

  // Analytical FKM
  double l0 = 0.043 + 0.084; 
  double l1 = 0.2;
  double l2 = 0.2;

  // Current end-effector position
  pos_(0) = l0 + l1 * std::cos(q_[0]) + l2 * std::cos(q_[1]); // X
  pos_(1) = (l1 * std::sin(q_[0]) + l2 * std::sin( q_[1])); // Y
  pos_(2) = 0.0033;                                              // Z
  

  // Analytical Jacobian
  Eigen::MatrixXd jacobian(2, 2);
  jacobian(0, 0) = -l1 * std::sin(q_[0]); 
  jacobian(0, 1) = -l2 * std::sin( q_[1]); 
  jacobian(1, 0) = l1 * std::cos(q_[0]);
  jacobian(1, 1) = l2 * std::cos(q_[1]);

   //Log data to CSV
   /*auto now = this->get_clock()->now();
   double time_sec = now.seconds();
 
   if (csv_file_.is_open()) {
     csv_file_ << time_sec << ","
               << q_[0] << "," << q_[1] << ","
               << d_q0 << "," << d_q1  << "\n";
   }*/
   


  // Position error (in X-Y plane)
  Eigen::VectorXd error(2);
  error(0) = d_pos_(0) - pos_(0); // X
  error(1) = d_pos_(1) - pos_(1); // Y 
  

 
  // Control gains
  Eigen::MatrixXd CS = Eigen::MatrixXd::Zero(2, 2);
  CS(0, 0) = 15.5;
  CS(1, 1) = 14.0;

  Eigen::MatrixXd CD = Eigen::MatrixXd::Zero(2, 2);
  CD(0, 0) =0.9;
  CD(1, 1) =0.8;

  // Compute task torque
  Eigen::VectorXd tau_task = jacobian.transpose() *((CS * error) - CD * (jacobian * dq_));

  // Prepare and publish torque message
  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  torque_msg->data.resize(tau_task.size());

  for (int i = 0; i < tau_task.size(); ++i) {
    torque_msg->data[i] = tau_task(i); // Or add gravity: tau_task(i) + tau_g(i)
  }

  gravity_torque_pub_->publish(std::move(torque_msg));
  
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MiniImpedanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}