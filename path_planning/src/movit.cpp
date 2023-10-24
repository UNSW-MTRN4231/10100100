#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//Function to generate a collision object
auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

//Function to generate a target position message
auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

using std::placeholders::_1;

class move_to_marker : public rclcpp::Node
{
  public:
    move_to_marker() : Node("move_to_marker")
    {

      // Initalise the transformation listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Look up the transformation ever 200 milliseconds
      timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&move_to_marker::tfCallback, this));



      // Generate the movegroup interface
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      move_group_interface->setPlanningTime(10.0);

      std::string frame_id = move_group_interface->getPlanningFrame();

      // Generate the objects to avoid
      // Generate a table collision object based on the lab task
      auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
      auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // Apply table as a collision object
      planning_scene_interface.applyCollisionObject(col_object_backWall);
      planning_scene_interface.applyCollisionObject(col_object_sideWall);

      // I cept this here for innniial debuging
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input", 10, std::bind(&move_to_marker::handle_keypress, this, _1));
      
      // Put subscription to brain here 

    }

    auto generatePoseMsg(double x,double y, double z,float qx,float qy,float qz,float qw) {
      geometry_msgs::msg::Pose msg;
      msg.orientation.x = qx;
      msg.orientation.y = qy;
      msg.orientation.z = qz;
      msg.orientation.w = qw;
      msg.position.x = x;
      msg.position.y = y;
      msg.position.z = z;
      return msg;
    }

  private:

    void tfCallback()
    {
      
      try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

      // Check if the transformation is between "ball_frame" and "base_link" 
      std::cout << "updating tf" << std::endl;
    }

    void handle_keypress(const std_msgs::msg::String & msg) {
      std::cout << msg.data << std::endl;
      if (msg.data == "space") {
        std::cout << "moving to point" << std::endl;
        auto msg = generatePoseMsg(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, 0.0, 0.0, 0.0, 0.0);
        moveit::planning_interface::MoveGroupInterface::Plan planMessage;

        move_group_interface->setPoseTarget(msg);
        auto success = static_cast<bool>(move_group_interface->plan(planMessage));

        //Execute movement to point 1
        if (success) {
          move_group_interface->execute(planMessage);
        } else {
          std::cout << "Planning failed!" << std::endl;
        }
        std::cout << "here" << std::endl;
      }
    }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  std::string fromFrameRel = "base_link";
  std::string toFrameRel = "next_point";
   geometry_msgs::msg::TransformStamped t;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<move_to_marker>());
  rclcpp::shutdown();
  return 0;
}