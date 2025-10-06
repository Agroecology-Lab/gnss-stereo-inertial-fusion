#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "System.h"

class GnssStereoInertialFusionNode : public rclcpp::Node
{
public:
  GnssStereoInertialFusionNode() : Node("gnss_stereo_inertial_fusion_node")
  {
    RCLCPP_INFO(this->get_logger(), "GNSS-Stereo-Inertial Fusion ROS2 Node started.");

    // TODO: Initialize ORB_SLAM3::System
    // mpSystem = new ORB_SLAM3::System("path_to_vocabulary", "path_to_settings", ORB_SLAM3::System::IMU_STEREO, true, 0, "");

    // TODO: Create subscribers for stereo images, IMU, and GPS data
    // stereo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/camera/left/image_raw", 10, std::bind(&GnssStereoInertialFusionNode::stereoCallback, this, _1));

    // TODO: Create publishers for pose/odometry
    // pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_slam/pose", 10);
  }

private:
  // ORB_SLAM3::System* mpSystem;

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
