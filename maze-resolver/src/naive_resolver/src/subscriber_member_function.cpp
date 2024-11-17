
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"       // ROS 2 C++ client library for node creation and management
#include "std_msgs/msg/string.hpp" // Standard message type for string messages
using std::placeholders::_1;       // Create a placeholder for the first argument of the function

class MinimalSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a MinimalSubscriber node.
     *
     * Sets up a subscriber for 'std_msgs::msg::String' messages on the "topic_cpp" topic.     *
     */
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        // Create a subscriber object for listening to string messages on
        // the "topic_cpp" topic with a queue size of 10.
        subscriber_ = create_subscription<std_msgs::msg::String>(
            "/topic",
            10,
            std::bind(
                &MinimalSubscriber::topicCallback,
                this,
                _1));
    }

    /**
     * @brief This function runs every time a message is received on the topic.
     *
     * This is the callback function of the subscriber. It publishes a string message
     * every time a message is received on the topic.
     *
     * @param msg The string message received on the topic
     * @return Void.
     */
    void topicCallback(const std_msgs::msg::String &msg) const
    {
        // Write a message every time a new message is received on the topic.
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
    }

private:
    // Member variables.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // The subscriber object.
};

/**
 * @brief Main function.
 *
 * Initializes the ROS 2 system and runs the minimal_subscriber node. It keeps the node
 * alive until it is manually terminated.
 */
int main(int argc, char *argv[])
{

    // Initialize ROS 2.
    rclcpp::init(argc, argv);

    // Create an instance of the MinimalSubscriber node and keep it running.
    auto minimal_subscriber_node = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(minimal_subscriber_node);

    // Shutdown ROS 2 upon node termination.
    rclcpp::shutdown();

    // End of program.
    return 0;
}