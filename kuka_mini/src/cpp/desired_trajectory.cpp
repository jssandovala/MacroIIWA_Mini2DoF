#include "kuka_mini/desired_trajectory.h"

DesiredTrajectoryNode::DesiredTrajectoryNode() : Node("desired_trajectory_node")
{
    //Desired trajectory publisher
    desired_trajectory_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/desired_trajectory", 10);

    timer_ = this->create_wall_timer(
        std::chrono::microseconds(10000),  // microseconds >> 100hz
        std::bind(&DesiredTrajectoryNode::timer_callback, this));


    waypoints_ ={

        {-1.0021, 0.0, 0.4213},
        {-1.0021, -0.08, 0.4213},
        {-0.9231, -0.08, 0.4213},
        {-0.9231, 0.0, 0.4213},
        {-1.0021, 0.0, 0.4213},

    };

    tf_ = 7.0; //final time in seconds
    t_ = 0.0;
    i = 0;

}

void DesiredTrajectoryNode::timer_callback()
{
  // to run the update at 1KHz
  update();
}

void DesiredTrajectoryNode::update()
{
    double t = t_;
    double tf = tf_;
    double s = t/tf;

    //Polynomial of degree 3 s= 3*(t/tf)^2 - 2*(t/tf)^3  --> for trajectories
    double alpha = 3*s*s - 2*s*s;

    //linear interpolation between two points
    auto p0 = waypoints_[i]; //current waypoint
    auto p1 = waypoints_[i+1]; //next waypoint

    std::array<double, 3> position_;
    for (size_t i_=0; i_ < 3; ++i_){
        //(1-t)*vo + t*v1
        position_[i_] = (1 - alpha)*p0[i_] + alpha*p1[i_];
    }

    geometry_msgs::msg::PoseStamped traj_msg;
    traj_msg.header.stamp = this->get_clock()->now();

    traj_msg.header.frame_id = "world";
    traj_msg.pose.position.x = position_[0];
    traj_msg.pose.position.y = position_[1];
    traj_msg.pose.position.z = position_[2];

    desired_trajectory_pub_->publish(traj_msg);

    t_ += 0.01;

    if (s >= 1.0){
        t_ = 0.0; //reset time for next segment
        i += 1; //move to the next waypoint

        if (i >= waypoints_.size()-1){
            RCLCPP_INFO(this->get_logger(), "Trajectory finished.");
            rclcpp::shutdown();
            return;
        }
    }

    // t_ += 0.02; //To increase the progress in the trajectory

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesiredTrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
