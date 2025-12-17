#include "kuka_mini/mini_effort_tm.h"



MiniEffortNode::MiniEffortNode() : Node("mini_effort_node")
{
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/mini/joint_states", 10, 
    std::bind(&MiniEffortNode::joint_state_callback, this, std::placeholders::_1));


  //forward kinematic mini publisher. By hand
  fk_publisher_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/fk_mini", 10);
  
  //To obtain information for the graphs
  // fk_publisher_graph_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
  //   "/fk_mini_graph", 10);

    //For the graph
  // fk_mini_wrt_world_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
  //   "/fk_mini_wrt_world", 10);

  //world_T_ee_IM publisher
  world_T_ee_IM_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/world_T_ee_IM", 10);

    //Forward Kinematic Model from the KUKA
  FK_kuka_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/iiwa/state/FKM", 10, 
    std::bind(&MiniEffortNode::iiwa_fk_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(10000),  // microseconds >> 100hz
    std::bind(&MiniEffortNode::timer_callback, this));
    
    has_pose_ = true; 

  RCLCPP_INFO(this->get_logger(), "Mini Kuka Transform Trial node started");
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

  q_(0) = msg->position[0];
  q_(1) = msg->position[1];
  dq_(0) = msg->velocity[0];
  dq_(1) = msg->velocity[1];

  // like wait for a massage
  has_joint_states_ = true;
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
  
  // Forward Kinematics

  double l1 = 0.130;
  double l2 = 0.150;
  double theta1 = q_[0];
  double theta2 = q_[1];
  
  double x = l1 * std::sin(theta1) + l2 * std::sin(theta1+theta2);
  double z = l1 * std::cos(theta1) + l2 * std::cos(theta1+theta2) + 0.127; //offset is now 0.043 + 0.084 = 0.127 (real robot)


  // Eigen::Vector2d mini_fk_xz;
  mini_fk_xz << x, z;
  //For debuging
  // std::cout << std::fixed << std::setprecision(3);
  // std::cout << "x and z: "
  //           << mini_fk_xz(0) << ", "
  //           << mini_fk_xz(1) << std::endl;


  auto fk_mini_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  fk_mini_msg->data.resize(2);

  for (int i = 0; i < 2; ++i) {
    fk_mini_msg->data[i] = mini_fk_xz(i);
  }
  fk_publisher_pub->publish(std::move(fk_mini_msg));



  //For the graph 
  // auto fk_publisher_graph_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  // fk_publisher_graph_msg->data.resize(3);

  // for (int i = 0; i < 2; ++i) {
  //   fk_publisher_graph_msg->data[i] = mini_fk_xz(i);
  // }
  // fk_publisher_graph_msg->data[2] = this->get_clock()->now().seconds(); //seconds

  // fk_publisher_graph_pub->publish(std::move(fk_publisher_graph_msg));

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

  
    Eigen::Matrix4d tool0_T_ee = Eigen::Matrix4d::Identity();

    //This is because of the frames between mini and kuka
    //kuka x = mini z
    //kuka y = mini x
    tool0_T_ee(0,0) = 0.0;
    tool0_T_ee(0,1) = -1.0;
    tool0_T_ee(0,2) = 0.0;
    tool0_T_ee(0,3) = 0.0;
    tool0_T_ee(1,0) = 1.0;
    tool0_T_ee(1,1) = 0.0;
    tool0_T_ee(1,2) = 0.0;
    tool0_T_ee(1,3) = mini_fk_xz(0); //x
    tool0_T_ee(2,0) = 0.0;
    tool0_T_ee(2,1) = 0.0;
    tool0_T_ee(2,2) = 1.0;
    tool0_T_ee(2,3) = mini_fk_xz(1); //z
    tool0_T_ee(3,0) = 0.0;
    tool0_T_ee(3,1) = 0.0;
    tool0_T_ee(3,2) = 0.0;
    tool0_T_ee(3,3) = 1.0;

    Eigen::Matrix4d world_T_ee_IM = Eigen::Matrix4d::Identity();
    world_T_ee_IM = world_T_tool0 * tool0_T_ee;

    auto world_T_ee_IM_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    world_T_ee_IM_msg->header.stamp = this->get_clock()->now();
    world_T_ee_IM_msg->header.frame_id = "world";

    //Translation
    world_T_ee_IM_msg->pose.position.x = world_T_ee_IM(0, 3);
    world_T_ee_IM_msg->pose.position.y = world_T_ee_IM(1, 3);
    world_T_ee_IM_msg->pose.position.z = world_T_ee_IM(2, 3);

    //Rotation
    Eigen::Quaterniond quat_world_T_ee(world_T_ee_IM.block<3,3>(0,0));
    world_T_ee_IM_msg->pose.orientation.w = quat_world_T_ee.w();
    world_T_ee_IM_msg->pose.orientation.x = quat_world_T_ee.x();
    world_T_ee_IM_msg->pose.orientation.y = quat_world_T_ee.y();
    world_T_ee_IM_msg->pose.orientation.z = quat_world_T_ee.z();

    world_T_ee_IM_pub->publish(std::move(world_T_ee_IM_msg));

    // To obtain information the graph
    // auto fk_mini_wrt_world_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    // fk_mini_wrt_world_msg->data.resize(3);
    // fk_mini_wrt_world_msg->data[0] = world_T_ee_IM(0, 3); // x
    // fk_mini_wrt_world_msg->data[1] = world_T_ee_IM(1, 3); // y
    // fk_mini_wrt_world_msg->data[2] = this->get_clock()->now().seconds(); // seconds
    // fk_mini_wrt_world_pub->publish(std::move(fk_mini_wrt_world_msg));
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