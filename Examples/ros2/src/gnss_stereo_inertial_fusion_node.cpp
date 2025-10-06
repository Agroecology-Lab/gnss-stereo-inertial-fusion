#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "System.h"
#include <functional>

class GnssStereoInertialFusionNode : public rclcpp::Node
{
public:
  GnssStereoInertialFusionNode() : Node("gnss_stereo_inertial_fusion_node")
  {
    RCLCPP_INFO(this->get_logger(), "GNSS-Stereo-Inertial Fusion ROS2 Node started.");

    // TODO: Initialize ORB_SLAM3::System with actual paths and sensor type
    // For now, using placeholder paths and IMU_STEREO as an example.
    // These paths should ideally come from ROS2 parameters.
    mpSystem = new ORB_SLAM3::System("vocabulary.fbow", "settings.yaml", ORB_SLAM3::System::IMU_STEREO, true, 0, "");

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_sub_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_sub_right_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    // TODO: Create publishers for pose/odometry
    // pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_slam/pose", 10);
  }

private:
  ORB_SLAM3::System* mpSystem;

  // TODO: Callbacks for subscribers
  // void stereoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  // {
  //   // Process stereo image
  // }

  // TODO: Publish results
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssStereoInertialFusionNode>());
  rclcpp::shutdown();
  return 0;
}
