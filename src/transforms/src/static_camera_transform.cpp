#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTFBroadcaster : public rclcpp::Node
{
public:
  StaticTFBroadcaster() : Node("static_tf_broadcaster")
  {
    // Create a static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


    // Fill out the transformation stamped message
    transformStamped_.header.frame_id = "base_link";
    transformStamped_.child_frame_id = "camera_frame";

    transformStamped_.transform.translation.x = 0.075;
    transformStamped_.transform.translation.y = 1.0;
    transformStamped_.transform.translation.z = 1.0;

    tf2::Quaternion q;
    q.setRPY(4.5*M_PI / 6.0, 0.0, 0.0); // -30 degrees pitch in radians
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();


    // Broadcaster static transformation
    tf_static_broadcaster_->sendTransform(transformStamped_);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
