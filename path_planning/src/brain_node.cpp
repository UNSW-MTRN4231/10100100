#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "custom_messages/msg/robot_action.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <vector>

class BrainNode : public rclcpp::Node {
public:
    BrainNode() : Node("my_node") {
        // Subscribe to /tf2
        // tf2_subscriber_ = create_subscription<tf2_msgs::msg::TFMessage>(
        //     "/tf2", 10,
        //     [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                
        //     });

        // Subscribe to /pen_colours
        pen_colours_subscriber_ = create_subscription<std_msgs::msg::Int32MultiArray>("/pen_colours", 10, std::bind(&BrainNode::handle_pen_colours, this,  std::placeholders::_1));

        // Subscribe to  /image_obtained
        image_obtained_subscriber_ = create_subscription<std_msgs::msg::Bool>("/image_obtained", 10, std::bind(&BrainNode::handle_start_process, this,  std::placeholders::_1));

        // Subscribe to /path
        // path_subscriber_ = create_subscription<geometry_msgs::msg::Point32>(
        //     "/path", 10,
        //     [this](const geometry_msgs::msg::Point32::SharedPtr msg) {
        //         // Process /path data here
        //     });

        // Publish to /commands
        commands_publisher_ = create_publisher<std_msgs::msg::String>("/commands", 10);

        // Publish to /robot_action
        //robot_action_publisher_ = create_publisher<custom_msgs::msg::RobotAction>("/robot_action", 10);

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
            
        }
    }


private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf2_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pen_colours_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr image_obtained_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr path_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_publisher_;
    //rclcpp::Publisher<custom_msgs::msg::RobotAction>::SharedPtr robot_action_publisher_;
    std::vector<int> colours;
    std::vector<int> colours_processed;
    bool process_started;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}