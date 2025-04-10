#pragma once

// #include <ros2_introspection/ros2_introspection.hpp>  // rosbag2_cpp 의존성 문제 해결을 위해 제거
#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace Ros2Introspection {
    // 필요한 타입만 직접 선언
    typedef std::vector<std::pair<std::string, double>> RenamedValues;
}

namespace ros2_introspection
{

/**
 * @brief TopicIntrospector class provides utilities for introspecting ROS 2 topics.
 * It allows monitoring topics, getting their types, and extracting message fields.
 */
class TopicIntrospector
{
public:
  /**
   * @brief Construct a new Topic Introspector object
   */
  TopicIntrospector();

  /**
   * @brief Destroy the Topic Introspector object
   */
  ~TopicIntrospector() = default;

  /**
   * @brief Initialize the introspector
   * 
   * @param node ROS 2 node to use for introspection
   * @return true if initialization was successful
   * @return false if initialization failed
   */
  bool initialize(rclcpp::Node* node);

  /**
   * @brief Add a topic to introspect
   * 
   * @param topic_name Topic name to introspect
   * @param topic_type Topic type (e.g., "std_msgs/msg/String")
   * @return true if topic was added successfully
   * @return false if topic could not be added
   */
  bool addTopic(const std::string& topic_name, const std::string& topic_type);

  /**
   * @brief Get the list of introspected topics
   * 
   * @return std::vector<std::string> List of topic names
   */
  std::vector<std::string> getTopics() const;

  /**
   * @brief Get the type of a topic
   * 
   * @param topic_name Topic name
   * @return std::string Topic type
   */
  std::string getTopicType(const std::string& topic_name) const;

  /**
   * @brief Process a message from a topic
   * 
   * @param topic_name Topic name
   * @param message_data Raw message data
   * @param values Output parameter for extracted field values
   * @return true if processing was successful
   * @return false if processing failed
   */
  bool processMessage(const std::string& topic_name, 
                     const rclcpp::SerializedMessage& message_data,
                     Ros2Introspection::RenamedValues& values);

private:
  rclcpp::Node* node_;
  std::map<std::string, std::string> topic_types_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};

} // namespace ros2_introspection 