#include "rclcpp/rclcpp.hpp"    // ros2 C++ client libraries, rcl = ros client layer    
#include "geometry_msgs/msg/transform_stamped.hpp"  // include headers for message type = geomtery_msgs
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class TfBroadcaster : public rclcpp::Node
{
public:
    TfBroadcaster() : Node("tf_broadcaster")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Broadcast transform in a loop
        timer_ = this->create_wall_timer(1s, std::bind(&TfBroadcaster::broadcastTf, this));
    }

private:    
    void broadcastTf()
    {
        geometry_msgs::msg::TransformStamped detection_tf;
        detection_tf.header.frame_id = "base_footprint";
        detection_tf.header.stamp = this->get_clock()->now();
        detection_tf.child_frame_id = "detected_obstacle";
        detection_tf.transform.translation.x = 1.0;
        tf_broadcaster_->sendTransform(detection_tf);

        // RCLCPP_INFO(get_logger(), "TF transform sent from '%s' to '%s'", 
        //             detection_tf.header.frame_id.c_str(), detection_tf.child_frame_id.c_str());
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
