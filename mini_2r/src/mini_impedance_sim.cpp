#include "mini_2r/mini_impedance.h"



MiniImpedanceNode::MiniImpedanceNode() : Node("mini_effort_node")
{

  this->declare_parameter("urdf_path", "mini.urdf");
  std::string urdf_path = this->get_parameter("urdf_path").as_string();
  
  pinocchio::urdf::buildModel(urdf_path, model_);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_.name.c_str());
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model_.gravity.linear(gravity);
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, 
    std::bind(&MiniImpedanceNode::joint_state_callback, this, std::placeholders::_1));

  // marker subscriber
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mini/pose_sim", 10,
    std::bind(&MiniImpedanceNode::pose_callback, this, std::placeholders::_1));

  
  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_effort_controller/commands", 10);


  timer_ = this->create_wall_timer(
    std::chrono::microseconds(1000),  // microseconds >> 1000hz
    std::bind(&MiniImpedanceNode::timer_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Gravity compensation node started");
}


void MiniImpedanceNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Resize q_ vector if necessary
  if (q_.size() != static_cast<int>(msg->position.size())) {
    q_.resize(msg->position.size());
    dq_.resize(msg->velocity.size());

  }

  for (size_t i = 0; i < msg->position.size(); ++i) {
    q_(i) = msg->position[i];
    dq_(i) = msg->velocity[i];
  }
  // like wait for a massage
  has_joint_states_ = true;
}

void MiniImpedanceNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  d_pos_(0)= msg->pose.position.x;
  d_pos_(1)= msg->pose.position.y;
  d_pos_(2)= msg->pose.position.z;

  // like wait for a massage
  has_pose_ = true;
}


void MiniImpedanceNode::timer_callback()
{
  // to run the update at 1KHz
  update();
}



void MiniImpedanceNode::update()
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  // like wait for a massage
  if (!has_joint_states_ || !has_pose_) {
    return;
  }

  
  // gravity torques
  Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(model_, data_, q_);


  // analytical FKM
  double l0 = 0.043 + 0.084;
  double l1 = 0.2;
  double l2 = 0.2;
  
  pos_(0) = l1 * std::sin(q_[0]) + l2 * std::sin(q_[0]+q_[1]); // X
  pos_(1) = 0.0033; // Y
  pos_(2) = l0+ l1 * std::cos(q_[0]) + l2 * std::cos(q_[0]+q_[1]); // Z

  
  // analytical jacobian
  Eigen::MatrixXd jacobian(2,2);
 
  jacobian(0,0) = l1 * std::cos(q_[0]) + l2 * std::cos(q_[0]+q_[1]); // dX/dq1
  jacobian(0,1) = l2 * std::cos(q_[0]+q_[1]); // dX/dq2
  jacobian(1,0) = -l1 * std::sin(q_[0]) - l2 * std::sin(q_[0]+q_[1]); // dZ/dq1
  jacobian(1,1) = -l2 * std::sin(q_[0]+q_[1]); // dZ/dq2



  // Position error (in X-Z plane)
  Eigen::VectorXd error(2);
  error(0) = d_pos_(1) - pos_(0); // Y
  error(1) = d_pos_(0) - pos_(2); // X



  // task
  Eigen::VectorXd tau_task(2);
  

  // Matrice de gain 
  // the choice of 25 is to make more rigid the control 
  Eigen::MatrixXd CS(2,2);
  CS(0,0)=25; CS(0,1)=0;
  CS(1,0)=0; CS(1,1)=25;

  Eigen::MatrixXd CD(2,2);
  CD(0,0)=2*sqrt(CS(0,0)); CD(0,1)=0;
  CD(1,0)=0; CD(1,1)=2*sqrt(CS(1,1));

  Eigen::MatrixXd CD2(2,2);
  CD2(0,0)=0.01; CD2(0,1)=0;
  CD2(1,0)=0; CD2(1,1)=0.01;


  
  // Compute the task torque
  tau_task=jacobian.transpose()*(CS*error - CD*(jacobian * dq_));


  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  torque_msg->data.resize(tau_g.size());
  

  
  for (int i = 0; i < tau_g.size(); ++i) {  
      torque_msg->data[i] = tau_task(i)+ tau_g(i);   
  }

  gravity_torque_pub_->publish(std::move(torque_msg));
  
}

std::string MiniImpedanceNode::eigen_vector_to_string(const Eigen::VectorXd& vec) {
  std::ostringstream oss;
  for (int i = 0; i < vec.size(); ++i) {
    oss << vec(i);
    if (i < vec.size() - 1) oss << ", ";
  }
  return oss.str();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MiniImpedanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}