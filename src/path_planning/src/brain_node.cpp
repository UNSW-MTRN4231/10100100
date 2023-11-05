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

class BrainNode : public rclcpp::Node {
public:
    BrainNode() : Node("brain_node") {
        auto sub_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = sub_cb_group;
        // Subscribe to /pen_colours
        pen_colours_subscriber_ = create_subscription<std_msgs::msg::Int32MultiArray>("/pen_colours", 10, std::bind(&BrainNode::handle_pen_colours, this,  std::placeholders::_1), options);

        // Subscribe to  /image_obtained
        image_obtained_subscriber_ = create_subscription<std_msgs::msg::Bool>("/image_obtained", 10, std::bind(&BrainNode::handle_start_process, this,  std::placeholders::_1), options);

        // Publish to /commands
        // commands_publisher_ = create_publisher<std_msgs::msg::String>("/commands", 10);

        path_client_ = this->create_client<custom_messages::srv::PathClient>("path_service", rmw_qos_profile_services_default, client_cb_group);

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

    void handle_pen_colours(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (colours.size() == 0) {
            const std::vector<int>& colours_arr = msg->data;
            colours.insert(colours.begin(), colours_arr.begin(), colours_arr.end());
        } else if (colours.size() != msg->data.size()) {
            const std::vector<int>& colours_arr = msg->data;
            std::vector<int> received_colours(colours_arr.begin(), colours_arr.end());
            std::vector<int> missing_colours = colours;

            remove_values_within_range(missing_colours, received_colours, 15);
            remove_values_within_range(missing_colours, colours_processed, 15);

            if (missing_colours.size() != 0) {
                std_msgs::msg::String message;
                message.data = "restart";
                commands_publisher_->publish(message);
            }
        }
    }

    void remove_values_within_range(std::vector<int>& firstArray, const std::vector<int>& secondArray, int range) {
        for (auto it = firstArray.begin(); it != firstArray.end(); ) {
            bool removeValue = false;
            for (const int& secondValue : secondArray) {
                if (std::abs(*it - secondValue) <= range) {
                    removeValue = true;
                    break; // No need to check other values in the second array
                }
            }
            if (removeValue) {
                it = firstArray.erase(it);
            } else {
                ++it;
            }
        }
    }

    void handle_start_process(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data == true && process_started == false) {
            process_started = true;
            handle_make_request(1);
        }
    }

    void handle_make_request(int index) {
        auto request = std::make_shared<custom_messages::srv::PathClient::Request>();
        request->colour = {index}; // Set the request parameters
        response_received = false;
        // Send the request to the service
        auto result_future = path_client_->async_send_request(request, std::bind(&BrainNode::handle_service_response, this, std::placeholders::_1));
        while (rclcpp::ok() && !response_received) {
            usleep(1000);
        }
    }


    void handle_service_response(rclcpp::Client<custom_messages::srv::PathClient>::SharedFuture result_future) {
        response_received = true;
        if (result_future.get()) {
            RCLCPP_INFO(this->get_logger(), "Received response");
            process_response();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response.");
        }
    }

    void process_response(){}


private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pen_colours_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr image_obtained_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr path_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_publisher_;
    rclcpp::Client<custom_messages::srv::PathClient>::SharedPtr path_client_;
    //rclcpp::Publisher<custom_messages::msg::RobotAction>::SharedPtr robot_action_publisher_;
    std::vector<int> colours;
    std::vector<int> colours_processed;
    bool process_started;
    bool response_received;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}