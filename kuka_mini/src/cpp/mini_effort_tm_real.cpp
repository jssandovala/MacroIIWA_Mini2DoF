#include "kuka_mini/mini_effort_tm.h"



MiniEffortNode::MiniEffortNode() : Node("mini_effort_node")
{

  this->declare_parameter("urdf_path", "/home/amir/ros2_ws/src/kuka_mini/urdf/kuka_mini.urdf");
  std::string urdf_path = this->get_parameter("urdf_path").as_string();
  
  pinocchio::urdf::buildModel(urdf_path, model_);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_.name.c_str());
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model_.gravity.linear(gravity);
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/mini/joint_states", 10, 
    std::bind(&MiniEffortNode::joint_state_callback, this, std::placeholders::_1));

  // marker subscriber
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mini/pose", 10,
    std::bind(&MiniEffortNode::pose_callback, this, std::placeholders::_1));

  
  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_effort_controller/commands", 10);
  
  //To obtain information for the graphs
  desired_trajectory_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/desired_trajectory", 10);



    //Forward Kinematic Model from the KUKA
  FK_kuka_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/iiwa/state/FKM", 10, 
    std::bind(&MiniEffortNode::iiwa_fk_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(10000),  // microseconds >> 100hz
    std::bind(&MiniEffortNode::timer_callback, this));

    has_pose_ = true;
    
  RCLCPP_INFO(this->get_logger(), "Gravity compensation node started");
}


void MiniEffortNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void MiniEffortNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);
 
  //Position I want in world frame
  Eigen::Matrix4d world_T_IM = Eigen::Matrix4d::Identity();
  world_T_IM(0,3) = msg->pose.position.x;
  world_T_IM(1,3) = msg->pose.position.y;
  world_T_IM(2,3) = msg->pose.position.z;

  //For debuggin
  // pose_interactive_market << world_T_IM(0,3), world_T_IM(1,3), world_T_IM(2,3);

  //Now I transform it into tool0 frame
  tool0_T_ee_mini_desired = world_T_tool0.inverse() * world_T_IM;

  //Now extract the desired position in mini frame
  d_pos_(0) = tool0_T_ee_mini_desired(0,3);
  d_pos_(1) = tool0_T_ee_mini_desired(1,3);
  d_pos_(2) = tool0_T_ee_mini_desired(2,3);
  
  // like wait for a massage
  has_pose_ = true;
}


void MiniEffortNode::timer_callback()
{
  // to run the update at 1KHz
  update();
}



void MiniEffortNode::update()
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  // like wait for a massage
  if (!has_joint_states_ || !has_pose_) {
    return;
  }

  Eigen::VectorXd q_mini(2), dq_mini(2);
  q_mini << q_(0), q_(1);
  dq_mini << dq_(0), dq_(1);
  
  // gravity torques
  // Eigen::VectorXd tau_g_full = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  // Eigen::Vector2d tau_g_mini;
  // tau_g_mini << tau_g_full[7], tau_g_full[8]; //from the urdf order
  
  // Forward Kinematics
  double l0 = 0.043 + 0.084;
  double l1 = 0.2;
  double l2 = 0.2;
  double theta1 = q_[0];
  double theta2 = q_[1];
  
  
  double x = l1 * std::sin(theta1) + l2 * std::sin(theta1+theta2);
  double z = l1 * std::cos(theta1) + l2 * std::cos(theta1+theta2) + l0;

  // Eigen::Vector2d mini_fk_xz;
  mini_fk_xz << x, z;

  //Jacobian
  Eigen::MatrixXd jacobian(2,2);
  jacobian(0,0)= l1*std::cos(theta1)+l2*std::cos(theta1+theta2);
  jacobian(0,1)= l2*std::cos(theta1+theta2);
  jacobian(1,0)= -l1*std::sin(theta1)-l2*std::sin(theta1+theta2);
  jacobian(1,1)= -l2*std::sin(theta1+theta2);

  
  // error 
  Eigen::VectorXd error(2);
  error(0)= d_pos_(1) - x ;
  error(1)= d_pos_(2) - z ;

  auto desired_traj_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  desired_traj_msg->data.resize(4);
  desired_traj_msg->data[0] = this->get_clock()->now().seconds(); //seconds
  desired_traj_msg->data[1] = d_pos_(0);
  desired_traj_msg->data[2] = d_pos_(1);
  desired_traj_msg->data[3] = d_pos_(2);

  desired_trajectory_pub->publish(std::move(desired_traj_msg));


  // std::cout << "Desired position: " << d_pos_ << std::endl;
  // std::cout << "Error: " << error << std::endl;

  // task
  Eigen::VectorXd tau_task(2);
  // Eigen::MatrixXd jacobian_p(2,2);

  //Proportional gain matrix
  Eigen::MatrixXd CS(2,2);
  CS(0,0)=1.05; CS(0,1)=0;
  CS(1,0)=0; CS(1,1)=1.05;
  // CS(0,0)=300; CS(0,1)=0;
  // CS(1,0)=0; CS(1,1)=300;

  //Derivative gain matrix
  Eigen::MatrixXd CD(2,2);
  // CD(0,0)=2*sqrt(CS(0,0)); CD(0,1)=0;
  // CD(1,0)=0; CD(1,1)=2*sqrt(CS(1,1));
  CD(0,0)=0.1; CD(0,1)=0;
  CD(1,0)=0; CD(1,1)=0.1;


  tau_task=jacobian.transpose()*(CS*error - CD*(jacobian * dq_mini));


  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  torque_msg->data.resize(2);
  

  // for (int i = 0; i < tau_g_mini.size(); ++i) {
  //   torque_msg->data[i] = tau_g_mini(i)+tau_task(i);
  // }

  for (int i = 0; i < 2; ++i) {
    torque_msg->data[i] = tau_task(i);
  }

  gravity_torque_pub_->publish(std::move(torque_msg));

  // std::cout << "Mini Effort: " << tau_task << std::endl;

}

void MiniEffortNode::iiwa_fk_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Eigen::Vector3d iiwa_position(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );

    Eigen::Quaterniond quat(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    quat.normalize();

    T.block<3,3>(0,0) = quat.toRotationMatrix(); //R 
    T.block<3,1>(0,3) = iiwa_position;
    
    this-> world_T_tool0 = T; //Matrix4d


}

std::string MiniEffortNode::eigen_vector_to_string(const Eigen::VectorXd& vec) {
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
  auto node = std::make_shared<MiniEffortNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}