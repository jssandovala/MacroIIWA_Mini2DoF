#include "kuka_mini/mini_impedance_trajectory_world.h"


MiniImpedanceNode::MiniImpedanceNode() : Node("mini_impedance_node")
{

  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/mini/joint_states", 10, 
    std::bind(&MiniImpedanceNode::joint_state_callback, this, std::placeholders::_1));

  // trajectory subscriber
  traj_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/kuka_mini/pose", 10,
    std::bind(&MiniImpedanceNode::trajectory_callback, this, std::placeholders::_1));

  
  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_effort_controller/commands", 10);
  
  //To obtain information for the graphs
  desired_trajectory_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/desired_trajectory", 10);

  fk_publisher_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/fk_mini", 10);



  //Forward Kinematic Model from the KUKA
  FK_kuka_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/iiwa/state/FKM", 10, 
    std::bind(&MiniImpedanceNode::iiwa_fk_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(10000),  // microseconds >> 100hz
    std::bind(&MiniImpedanceNode::timer_callback, this));

    has_pose_ = true;
    
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

void MiniImpedanceNode::trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);
 
  //Position I want in world frame
  Eigen::Matrix4d world_T_IM = Eigen::Matrix4d::Identity();
  world_T_IM(0,3) = msg->pose.position.x;
  world_T_IM(1,3) = msg->pose.position.y;
  world_T_IM(2,3) = msg->pose.position.z;

  
  //Now I transform it into tool0 frame
  tool0_T_ee_mini_desired = world_T_tool0.inverse() * world_T_IM;

  //Now extract the desired position in mini frame
  d_pos_(0) = tool0_T_ee_mini_desired(0,3);
  d_pos_(1) = tool0_T_ee_mini_desired(1,3);
  d_pos_(2) = tool0_T_ee_mini_desired(2,3);
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

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Safety checks
  if (!has_joint_states_ || !has_pose_) {

    return;
  }

  if (q_.size() < 2 || dq_.size() < 2) {
    //RCLCPP_WARN(this->get_logger(), "q_ or dq_ size < 2, skipping update");
    return;
  }

  if (d_pos_[0]==0 && d_pos_[1]==0 && d_pos_[2]==0 ){
    //RCLCPP_WARN(this->get_logger(), "d_pos_ size == 0, skipping update");
    return;
  }


  
 
  // Forward Kinematics
  double l0 = 0.043 + 0.084;
  double l1 = 0.2;
  double l2 = 0.2;

  pos_(0) = l0 + l1 * std::cos(q_[0]) + l2 * std::cos(q_[1]); // X
  pos_(1) = (l1 * std::sin(q_[0]) + l2 * std::sin( q_[1])); // Y
  pos_(2) = 0.0033;  
  
  
  //double x = l1 * std::sin(theta1) + l2 * std::sin(theta1+theta2);
  //double z = l1 * std::cos(theta1) + l2 * std::cos(theta1+theta2) + l0;


  mini_fk_xz << pos_(1), pos_(0);

  // Analytical Jacobian
  Eigen::MatrixXd jacobian(2, 2);
  jacobian(0, 0) = -l1 * std::sin(q_[0]); 
  jacobian(0, 1) = -l2 * std::sin( q_[1]); 
  jacobian(1, 0) = l1 * std::cos(q_[0]);
  jacobian(1, 1) = l2 * std::cos(q_[1]);

  
  // error 
  Eigen::VectorXd error(2);
  error(0)= d_pos_(2) - pos_(0) ;
  error(1)= d_pos_(0) - pos_(1) ;


  //std::cout << "d_pos_: " << d_pos_.transpose() << std::endl;
  //std::cout << "pos_: " << pos_.transpose() << std::endl;
  //std::cout << "error: " << error.transpose() << std::endl;

  /*
  auto desired_traj_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  desired_traj_msg->data.resize(4);
  desired_traj_msg->data[0] = this->get_clock()->now().seconds(); //seconds
  desired_traj_msg->data[1] = d_pos_(0);
  desired_traj_msg->data[2] = d_pos_(1);
  desired_traj_msg->data[3] = d_pos_(2);

  desired_trajectory_pub->publish(std::move(desired_traj_msg));
  */


  // Control gains
  Eigen::MatrixXd CS = Eigen::MatrixXd::Zero(2, 2);
  CS(0, 0) = 3.0;//3.0;
  CS(1, 1) = 3.0;//3.3;

  Eigen::MatrixXd CD = Eigen::MatrixXd::Zero(2, 2);
  CD(0, 0) =0.07;//0.1;//0.1
  CD(1, 1) =0.05;//0.22;


 
  // Compute task torque
  Eigen::VectorXd tau_task = jacobian.transpose() *((CS * error) - CD * (jacobian * dq_));



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


  auto fk_mini_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  fk_mini_msg->data.resize(2);

  for (int i = 0; i < 2; ++i) {
    fk_mini_msg->data[i] = mini_fk_xz(i);
  }
  fk_publisher_pub->publish(std::move(fk_mini_msg));

}

void MiniImpedanceNode::iiwa_fk_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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