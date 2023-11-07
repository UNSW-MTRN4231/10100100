# 10100100
Final project MTRN4231
pip install opencv-contrib-python==4.6.0.66
sudo apt install ros-humble-tf-transformations




// Initalise the transformation listener
      // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Look up the transformation ever 200 milliseconds
      // timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&move_to_marker::tfCallback, this));
      
      // create subscription
      subscriber_ = this->create_subscription<custom_messages::msg::RobotAction>("/comands", 10, std::bind(&move_to_marker::robot_action_callback, this,  std::placeholders::_1));



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

  private:

    void robot_action_callback(const custom_messages::msg::RobotAction msg)
    {
      
      
      for(int i = 0; i < msg.x.size(); i++) {
        // waiting period
        int scaler = 1;
        float distance = sqrt((msg.x[i] - msg.x[prev]) * (msg.x[i] - msg.x[prev]) + (msg.y[i] - msg.y[prev]) * (msg.y[i] - msg.y[prev]));
        prev = i;   //set the prev current itorator
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(scaler * distance))); 

        std::cout << "moving to point" << std::endl;
        auto poseMsg = generatePoseMsg(msg.x[i], msg.y[i], msg.z[i], 0.0, 0.0, 0.0, 0.0);
        moveit::planning_interface::MoveGroupInterface::Plan planMessage;

        move_group_interface->setPoseTarget(poseMsg);
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
  // rclcpp::TimerBase::SharedPtr timer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  rclcpp::Subscription<custom_messages::msg::RobotAction>::SharedPtr subscriber_;
  // std::string fromFrameRel = "base_link";
  // std::string toFrameRel = "robot_action";
   geometry_msgs::msg::TransformStamped t;
   int prev = 0;      // used as arrray index in topic call back
};
