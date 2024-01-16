#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include <chrono>

using namespace std::chrono_literals;

class TfListner : public rclcpp::Node
{
public:
    TfListner() : Node("tf_listner")
    {   
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listner_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(1s, std::bind(&TfListner::listenTf, this));
    }

private:
    void listenTf()
    {
        try{
            geometry_msgs::msg::TransformStamped odom2obstacle;
            odom2obstacle = tf_buffer_->lookupTransform("odom", "detected_obstacle", tf2::TimePointZero);
            RCLCPP_INFO(get_logger(), "Received TF transform: translation(%.2f, %.2f, %.2f), rotation(%.2f, %.2f, %.2f, %.2f)",
                        odom2obstacle.transform.translation.x, odom2obstacle.transform.translation.y, odom2obstacle.transform.translation.z,
                        odom2obstacle.transform.rotation.w, odom2obstacle.transform.rotation.x, odom2obstacle.transform.rotation.y, odom2obstacle.transform.rotation.z);
        }
        catch(const tf2::TransformException & ex){
            RCLCPP_ERROR(get_logger(), "Failed to get TF transform: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listner_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListner>());
    rclcpp::shutdown();
    return 0;
}