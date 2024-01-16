#ifndef TF_DETECTOR__OBSTACLEDETECTORNODE_HPP_
#define TF_DETECTOR__OBSTACLEDETECTORNODE_HPP_

#include <memory>

#include <tf2_ros/static_transform_broadcaster.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tf2_detector
{

class ObstacleDetectorNode : rclcpp::Node
{
public:
    ObstacleDetectorNode();

private:
void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

}   // namespace tf2_detector

#endif  // TF_DETECTOR__OBSTACLEDETECTORNODE_HPP_