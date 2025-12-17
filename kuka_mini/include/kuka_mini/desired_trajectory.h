#ifndef DESIRED_TRAJECTORY_H
#define DESIRED_TRAJECTORY_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <array>


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

using std::placeholders::_1;

class DesiredTrajectoryNode : public rclcpp::Node
{
    public:
        DesiredTrajectoryNode();
    
    private:
    void update();
    void timer_callback();

    std::vector<std::array<double, 3>> waypoints_; //3 values per waypoint

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_trajectory_pub_;

    double t_;
    double tf_;
    size_t i;

};

#endif 