#include "kuka_mini/iiwa_mini_active_real.h"



MiniActiveNode::MiniActiveNode() : Node("mini_active_node")
{

  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/mini/joint_states", 10, 
    std::bind(&MiniActiveNode::joint_state_callback, this, std::placeholders::_1));

  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_effort_controller/commands", 10);
  

  error_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/error_mini", 10);


  // pedal subscriber
  pedal_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&MiniActiveNode::pedal_callback, this, std::placeholders::_1));

    //Forward Kinematic Model from the KUKA
  FK_kuka_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/iiwa/state/FKM", 10, 
    std::bind(&MiniActiveNode::iiwa_fk_callback, this, std::placeholders::_1));
  

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(10000),  // microseconds >> 100hz
    std::bind(&MiniActiveNode::timer_callback, this));

    // has_pose_ = true;

  pose_mini = false;
  pedal_on = false;
  has_joint_states_ = false;
  has_FK_kuka_ = false;
  start_ = false; 
  

  initial_pose_mode = 0.0;


    
  RCLCPP_INFO(this->get_logger(), "Gravity compensation node started");
}


void MiniActiveNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
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


void MiniActiveNode::pedal_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  initial_pose_mode = msg->axes[1];

  if (initial_pose_mode == 1.0){
    pedal_on = true;
  }
  else{
    pedal_on = false;
  }


}


void MiniActiveNode::iiwa_fk_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{   

  //kuka to tool0
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
    has_FK_kuka_ = true;
    

}

void MiniActiveNode::timer_callback()
{
  // to run the update at 1KHz
  update();
}



void MiniActiveNode::update()
{
  // Lock the mutex to ensure thread safety
  std::lock_guard<std::mutex> lock(data_mutex_);

  // like wait for a massage
  if (!has_joint_states_ or !has_FK_kuka_){
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

  // Eigen::Vector2d mini_fk_xz;
  mini_fk_xz << pos_(0), pos_(1);

  
  // Analytical Jacobian
  Eigen::MatrixXd jacobian(2, 2);
  jacobian(0, 0) = -l1 * std::sin(q_[0]); 
  jacobian(0, 1) = -l2 * std::sin( q_[1]); 
  jacobian(1, 0) = l1 * std::cos(q_[0]);
  jacobian(1, 1) = l2 * std::cos(q_[1]);

  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  torque_msg->data.resize(2);

  Eigen::Matrix4d tool0_T_ee = Eigen::Matrix4d::Identity();

  //This is because of the frames between mini and kuka --> kuka x = mini z   and   kuka y = mini x
  //tool0_T_ee(0,0) = 1.0;
  //tool0_T_ee(0,1) = 0.0;
  //tool0_T_ee(0,2) = 0.0;
  tool0_T_ee(0,3) = mini_fk_xz(1);
  //tool0_T_ee(1,0) = 0.0;
  //tool0_T_ee(1,1) = 1.0;
  //tool0_T_ee(1,2) = 0.0;
  //tool0_T_ee(1,3) = 0.0;
  //tool0_T_ee(2,0) = 0.0;
  //tool0_T_ee(2,1) = 0.0;
  //tool0_T_ee(2,2) = 1.0;
  tool0_T_ee(2,3) = mini_fk_xz(0);
  //tool0_T_ee(3,0) = 0.0;
  //tool0_T_ee(3,1) = 0.0;
  //tool0_T_ee(3,2) = 0.0;
  //tool0_T_ee(3,3) = 1.0;


  //if the pedal is off, mini is active
  if (!start_){ 
    // start with the current pose
    // error for the iiwa
    // world_T_ee = world_T_tool0 * tool0_T_ee;
    // tool0_T_ee_mini_desired = world_T_tool0.inverse() * world_T_ee;
    // d_pos0_(0) = tool0_T_ee_mini_desired(1,3);
    // d_pos0_(1) = tool0_T_ee_mini_desired(2,3);

    // std::cout << tool0_T_ee_mini_desired(1,3) << tool0_T_ee_mini_desired(2,3) << x << z << std::endl;
    d_pos0_(0) = mini_fk_xz(0);
    d_pos0_(1) = mini_fk_xz(1);


    start_ = true;
      
  }

  Eigen::VectorXd error_iiwa(2);
  error_iiwa(1)= d_pos0_(0) - mini_fk_xz(0);
  error_iiwa(0)= d_pos0_(1) - mini_fk_xz(1);


  if (!pose_mini){ //if pose_mini is false, it means we are in the current pose mode
    // std::cout << "Pedal is off, mini is active" << std::endl;
    world_T_ee = world_T_tool0 * tool0_T_ee;

    d_pos_(0) = pos_(0);
    d_pos_(1) = pos_(1);


    pose_mini = true;

  }


  if (!pedal_on){
    tool0_T_ee_mini_desired = world_T_tool0.inverse() * world_T_ee;

    d_pos_(0) = tool0_T_ee_mini_desired(2,3);
    d_pos_(1) = tool0_T_ee_mini_desired(0,3);

  

    Eigen::VectorXd error(2);
    error(0)= d_pos_(0) - mini_fk_xz(0);
    error(1)= d_pos_(1) - mini_fk_xz(1);

    


    //std::cout << "error mini: " << eigen_vector_to_string(error) << std::endl;



    // Control gains
    Eigen::MatrixXd CS = Eigen::MatrixXd::Zero(2, 2);
    CS(0, 0) = 3.0;//3.0;
    CS(1, 1) = 3.0;//3.3;

    Eigen::MatrixXd CD = Eigen::MatrixXd::Zero(2, 2);
    CD(0, 0) =0.07;//0.1;//0.1
    CD(1, 1) =0.05;//0.22;

    Eigen::VectorXd tau_task = jacobian.transpose() *((CS * error) - CD * (jacobian * dq_));


      
    for (int i = 0; i < 2; ++i) {
      torque_msg->data[i] = tau_task(i);
    }

    //std::cout << "Torque: " << eigen_vector_to_string(tau_task) << std::endl;
  }

  
  else{
    for (int i = 0; i < 2; ++i) { 
      torque_msg->data[i] = 0.0;
    }
    pose_mini = false;
    // std::cout << "Zero torque" << std::endl;
  }


  gravity_torque_pub_->publish(std::move(torque_msg));

  auto error_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  error_msg->data.resize(2);

  for (int i = 0; i < 2; ++i) {
    error_msg->data[i] = error_iiwa(i);
  }
        
  error_pub->publish(std::move(error_msg));


  //To obtain information for the graphs
  // auto desired_pose_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  // desired_pose_msg->data.resize(3);
  // desired_pose_msg->data[0] = this->get_clock()->now().seconds(); //seconds
  // desired_pose_msg->data[1] = d_pos_(0);
  // desired_pose_msg->data[2] = d_pos_(1);

  // desired_pose_pub->publish(std::move(desired_pose_msg));

  //Info for the graphs
  // auto error_msg_graphs = std::make_unique<std_msgs::msg::Float64MultiArray>();
  // error_msg_graphs->data.resize(3);

  // for (int i = 0; i < 2; ++i) {
  //   error_msg_graphs->data[i] = error_iiwa(i);
  // }

  // error_msg_graphs->data[2] = this->get_clock()->now().seconds(); // seconds
        
  // error_graph_pub->publish(std::move(error_msg_graphs));

}



std::string MiniActiveNode::eigen_vector_to_string(const Eigen::VectorXd& vec) {
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
  auto node = std::make_shared<MiniActiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}