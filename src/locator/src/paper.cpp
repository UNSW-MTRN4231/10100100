#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <tf2_ros/transform_broadcaster.h>


#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>




using namespace std::chrono_literals;
using std::placeholders::_1;

class PaperTFBroadcaster : public rclcpp::Node
{
  public:
    PaperTFBroadcaster()
    : Node("PaperTFBroadcaster")
    {
      //publish_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamicTFBroadcaster::publishDynamicTransform, this));
      subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>( "aruco_markers", 10, std::bind(&PaperTFBroadcaster::topic_callback, this, _1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  public:
    void topic_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) const
    {

      geometry_msgs::msg::TransformStamped transform_msg;
      for (size_t i = 0; i < msg->marker_ids.size(); ++i) {
        if (msg->marker_ids[i] <= 3) {

          transform_msg.header.frame_id = "camera_frame";
          transform_msg.child_frame_id = "paper_corner_" + std::to_string(msg->marker_ids[i]);
          if (!msg->poses.empty()) {
            // std::cout << "X: " << msg->marker_ids[i] << std::endl;
            //  std::cout << "X: " << msg->poses[i].position.x << std::endl;  
            //  std::cout << "Y: " << msg->poses[i].position.y << std::endl;
            //  std::cout << "Z: " << msg->poses[i].position.z << std::endl;
            transform_msg.transform.translation.x = msg->poses[i].position.x;
            transform_msg.transform.translation.y = msg->poses[i].position.y;
            transform_msg.transform.translation.z = msg->poses[i].position.z;
          } 
          // else {
          //   transform_msg.transform.translation.x = 0;
          //   transform_msg.transform.translation.y = 0;
          //   transform_msg.transform.translation.z = 0;
          // }
        } else if (msg->marker_ids[i] <= 5){
          transform_msg.header.frame_id = "camera_frame";
          transform_msg.child_frame_id = "marker_side_" + std::to_string(msg->marker_ids[i]);
          if (!msg->poses.empty()) {
            transform_msg.transform.translation.x = msg->poses[i].position.x;
            transform_msg.transform.translation.y = msg->poses[i].position.y;
            transform_msg.transform.translation.z = msg->poses[i].position.z;
          } 
          // else {
          //   transform_msg.transform.translation.x = 0;
          //   transform_msg.transform.translation.y = 0;
          //   transform_msg.transform.translation.z = 0;
          // }
        }
      }
      


      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0); // -30 degrees pitch in radians
      transform_msg.transform.rotation.x = q.x();
      transform_msg.transform.rotation.y = q.y();
      transform_msg.transform.rotation.z = q.z();
      transform_msg.transform.rotation.w = q.w();

      // 
       tf_broadcaster_->sendTransform(transform_msg);
    }




    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    private:
   // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PaperTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}