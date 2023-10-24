#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <custom_messages/msg/robot_action.hpp> // Replace 'custom_messages' with your message package
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointTFBroadcaster : public rclcpp::Node
{
public:
    PointTFBroadcaster()
        : Node("PointTFBroadcaster")
    {
        subscription_ = this->create_subscription<custom_messages::msg::RobotAction>(
            "Path", 10, std::bind(&PointTFBroadcaster::topic_callback, this, _1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void topic_callback(const custom_messages::msg::RobotAction::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->x.size(); i++) // Loop through message arrays
        {
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = now(); // Set the timestamp
            transform_msg.header.frame_id = "paper_corner_1";
            transform_msg.child_frame_id = "next_point";


              transform_msg.transform.translation.x = msg->x[i];
              transform_msg.transform.translation.y = msg->y[i];
              transform_msg.transform.translation.z = 80; // Random arbitrary choice at the moment

              tf2::Quaternion q;
              q.setRPY(0.0, 0.0, 0.0); // -30 degrees pitch in radians
              transform_msg.transform.rotation.x = q.x();
              transform_msg.transform.rotation.y = q.y();
              transform_msg.transform.rotation.z = q.z();
              transform_msg.transform.rotation.w = q.w();
            

            tf_broadcaster_->sendTransform(transform_msg);
        }
    }

private:
    rclcpp::Subscription<custom_messages::msg::RobotAction>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}