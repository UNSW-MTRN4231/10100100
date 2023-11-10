#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_messages/msg/robot_action.hpp"
#include "custom_messages/srv/path_client.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <vector>


#include <chrono>
using namespace std::chrono_literals;


class PathPlanningNode : public rclcpp::Node {
public:
    PathPlanningNode() : Node("path_planning_node") {
        
        end_effector_control = true;
        
        // Publish to /arduinoCommand
        commands_publisher_ = create_publisher<std_msgs::msg::String>("/arduinoCommand", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&PathPlanningNode::timer_callback, this));
    
    }

private:

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        if (end_effector_control == true) {
            message.data = "R-2\n"; // close
        } else {
            message.data = "R-50\n"; // open
        }

        // end_effector_control = !end_effector_control;
        
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        commands_publisher_->publish(message);
    }

    // control end effector, true -> close, false -> open
    bool end_effector_control;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_publisher_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}
