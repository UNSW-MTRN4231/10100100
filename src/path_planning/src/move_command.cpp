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
#include "custom_messages/msg/robot_action.hpp"
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;

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

class move_command : public rclcpp::Node
{
  public:
    move_command() : Node("move_command")
    {
      arduino_callback_group = nullptr;
      move_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions options;
      options.callback_group = move_callback_group;
      // Initalise the transformation listener
      // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Look up the transformation ever 200 milliseconds
      // timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&move_command::tfCallback, this));
      
      // create subscription
      subscriber_ = this->create_subscription<custom_messages::msg::RobotAction>("/robot_action", 10, std::bind(&move_command::robot_action_callback, this,  std::placeholders::_1), options);
      // Initalise the transformation listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      // Generate the movegroup interface
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      move_group_interface->setPlanningTime(30.0);

      std::string frame_id = move_group_interface->getPlanningFrame();

      // Generate the objects to avoid
      // Generate a table collision object based on the lab task
      auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
      auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
      auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // Apply table as a collision object
      planning_scene_interface.applyCollisionObject(col_object_backWall);
      planning_scene_interface.applyCollisionObject(col_object_sideWall);
      planning_scene_interface.applyCollisionObject(col_object_table);
      // Publish to /arduinoCommand
      commands_publisher_ = create_publisher<std_msgs::msg::String>("/arduinoCommand", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&move_command::timer_callback, this), arduino_callback_group);

      visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
        std::shared_ptr<rclcpp::Node>(this),
        "base_link",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface->getRobotModel()
      );
      visual_tools_->deleteAllMarkers();
      visual_tools_->loadRemoteControl();


    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        if (end_effector_control == true) {
            message.data = "R-6\n"; // close
        } else {
            message.data = "R-50\n"; // open
        }

        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        commands_publisher_->publish(message);
    }

    auto generatePoseMsg(float x, float y, float  z,float qx,float qy,float qz,float qw) {
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

    // Visualises a cartesian path in RVIZ with lines between each waypoint, and axes of each waypoint (optional) 
    void visualize_cartesian_path(std::vector<geometry_msgs::msg::Pose> waypoints,std::string ns){ 
      // Publish lines 
      visual_tools_->publishPath(waypoints, rviz_visual_tools::RED, rviz_visual_tools::SMALL,ns); 
      // Label points bool 

      if (false) { 
        for (size_t i = 0; i < waypoints.size(); i++) { 
          visual_tools_->publishAxis(waypoints[i], rviz_visual_tools::SMALL, ns); 
        } 
      } 
      visual_tools_->trigger(); 
    }

    void move_pen(bool close_gripper, tf2::Quaternion q, geometry_msgs::msg::Pose home) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      moveit_msgs::msg::RobotTrajectory trajectory;
          // Pick up the pen
      //get the transform
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

      double roll;
      double pitch;
      double yaw;
      waypoints.push_back(home);
      // get offset position on rack
      quaternionToEulerAngles(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w, roll, pitch, yaw);
      z_hight = 0.5;
      float x = -(0.12 + x_rack_offset*penIndex) * cos(yaw + M_PI/2)-griper_offset_x;
      float y = -(0.12 + x_rack_offset*penIndex) * sin(yaw+ M_PI/2)+0.02;
      // float x = 0;
      // float y = 0;
      // move above rack
      auto poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_hight << yaw << std::endl;
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_hight -(z_hight-z_pen)/5  << std::endl;
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -2*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_hight -2*(z_hight-z_pen)/5  << std::endl;
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -3*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_hight -3*(z_hight-z_pen)/5  << std::endl;
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -4*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_hight -4*(z_hight-z_pen)/5  << std::endl;
      waypoints.push_back(poseMsg);
      // move down to pick up the pen
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_pen, q.x(), q.y(), q.z(), q.w());
      std::cout << "Z: " <<  z_pen  << std::endl;
      waypoints.push_back(poseMsg);

      move_group_interface->setMaxVelocityScalingFactor(0.3);
      move_group_interface->setMaxAccelerationScalingFactor(0.3);
      move_group_interface->computeCartesianPath(waypoints, 0.1, 0.0, trajectory);
      move_group_interface->execute(trajectory);
      waypoints.clear();
      usleep(2000000);
      std::cout << "end effector change" << std::endl;
      end_effector_control = close_gripper;
      usleep(1000000);
      // ######### //
      //Need to tell griper to close or open
      //move up 
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -4*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -3*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -2*(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight -(z_hight-z_pen)/5, q.x(), q.y(), q.z(), q.w());
      waypoints.push_back(poseMsg);
      poseMsg = generatePoseMsg(t.transform.translation.x - x, t.transform.translation.y + y, z_hight, q.x(), q.y(), q.z(), q.w());
      waypoints.push_back(poseMsg);

      // home = generatePoseMsg(-0.4301,-0.1435, 0.4, q.x(), q.y(), q.z(), q.w());
      // waypoints.push_back(home);

      move_group_interface->setMaxVelocityScalingFactor(0.3);
      move_group_interface->setMaxAccelerationScalingFactor(0.3);
      move_group_interface->computeCartesianPath(waypoints, 0.1, 0.0, trajectory);
      move_group_interface->execute(trajectory);
      waypoints.clear();
      
      std::cout << "X: " << x << "Y: " << y << "Yaw: " << yaw << std::endl;
      std::cout << "here" << std::endl;
    }
      

  private:

    void quaternionToEulerAngles(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw) {
      // Create a quaternion from individual values
      Eigen::Quaterniond quaternion(qw, qx, qy, qz);

      // Convert quaternion to rotation matrix
      Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

      // Extract angles in radians
      roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
      pitch = asin(-rotationMatrix(2, 0));
      yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    }

    void robot_action_callback(const custom_messages::msg::RobotAction msg)
    {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      moveit_msgs::msg::RobotTrajectory trajectory;

      tf2::Quaternion q;
      
      q.setRPY(M_PI, 0.0, 0.0);
      auto home = generatePoseMsg(msg.x[0]-griper_offset_x, msg.y[0], 0.5, q.x(), q.y(), q.z(), q.w());
      usleep(1000000);
      // pick up pen
      move_pen(true, q, home);
      
      for(double i = 0; i < msg.x.size(); i++) {

        z_hight = 0.2025;
        auto poseMsg = generatePoseMsg(msg.x[i]-griper_offset_x, msg.y[i], z_hight + msg.z[i], q.x(), q.y(), q.z(), q.w());
        waypoints.push_back(poseMsg);
        
      }
      // put down pen
      waypoints.push_back(home);
      visualize_cartesian_path(waypoints, "Path");
      
      
      std::cout << "index: " << penIndex << std::endl;
      
      std::cout << "moving to point" << std::endl;
      move_group_interface->setMaxVelocityScalingFactor(0.3);
      move_group_interface->setMaxAccelerationScalingFactor(0.3);
      move_group_interface->computeCartesianPath(waypoints, 0.1, 0.0, trajectory);
      std::cout << "Length path: " << waypoints.size() << std::endl;
      move_group_interface->execute(trajectory);
      waypoints.clear();
      move_pen(false, q, home);
      penIndex += 2;
    }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  rclcpp::Subscription<custom_messages::msg::RobotAction>::SharedPtr subscriber_;
  std::string fromFrameRel = "pen_rack_4";
  std::string toFrameRel = "base_link";
  geometry_msgs::msg::TransformStamped t;
  
  int prev = 0;      // used as arrray index in topic call back
  int penIndex = 0;
  float z_hight = 0.5;
  float z_pen = 0.3;
  float x_rack_offset = 0.055;
  float y_rack_offset = 0.0;
  // control end effector, true -> close, false -> open
  bool end_effector_control;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_publisher_;
  rclcpp::CallbackGroup::SharedPtr move_callback_group;
  rclcpp::CallbackGroup::SharedPtr arduino_callback_group;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  float griper_offset_x = 0.037;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<move_command>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}