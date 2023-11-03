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

class path_planning_node : public rclcpp::Node {
public:
    path_planning() : Node("path_planning_node") {

        // Subscribe to  /image_obtained
        //image_obtained_subscriber_ = create_subscription<std_msgs::msg::Bool>("/image_obtained", 10, std::bind(&BrainNode::handle_start_process, this,  std::placeholders::_1));

        // Publish to /arduinoCommand
        commands_publisher_ = create_publisher<std_msgs::msg::String>("/arduinoCommand", 10);

        path_client_ = this->create_client<custom_messages::srv::PathClient>("path_client");

        while (!path_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available. Waiting...");
        }

        
        // Publish to /robot_action
        //robot_action_publisher_ = create_publisher<custom_messages::msg::RobotAction>("/robot_action", 10);

    }


    // control end effector, true -> close, false -> open
    void end_effector_control(bool command) {

        std_msgs::msg::String message;

        if (command == true) {
            message.data = "R-7\n";

        } else {
            message.data = "R-60\n";

        }

        commands_publisher_->publish(message);

    }


private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_subscriber_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_publisher_;
    rclcpp::Client<custom_messages::srv::PathClient>::SharedPtr path_client_;

    std::vector<int> colours;
    std::vector<int> colours_processed;
    bool process_started;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planning_node>());
    rclcpp::shutdown();
    return 0;
}