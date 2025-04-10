#include <ros2_introspection/topic_introspection.hpp>
#include <rclcpp/serialization.hpp>

namespace ros2_introspection
{

TopicIntrospector::TopicIntrospector()
: node_(nullptr)
{
}

bool TopicIntrospector::initialize(rclcpp::Node* node)
{
  if (!node) {
    return false;
  }
  
  node_ = node;
  return true;
}

bool TopicIntrospector::addTopic(const std::string& topic_name, const std::string& topic_type)
{
  try {
    if (!node_) {
      return false;
    }
    
    topic_types_[topic_name] = topic_type;
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to add topic %s [%s]: %s", 
               topic_name.c_str(), topic_type.c_str(), e.what());
    return false;
  }
}

std::vector<std::string> TopicIntrospector::getTopics() const
{
  std::vector<std::string> topics;
  for (const auto& entry : topic_types_) {
    topics.push_back(entry.first);
  }
  return topics;
}

std::string TopicIntrospector::getTopicType(const std::string& topic_name) const
{
  auto it = topic_types_.find(topic_name);
  if (it != topic_types_.end()) {
    return it->second;
  }
  return "";
}

bool TopicIntrospector::processMessage(
  const std::string& topic_name, 
  const rclcpp::SerializedMessage& message_data,
  Ros2Introspection::RenamedValues& values)
{
  try {
    if (!node_) {
      return false;
    }
    
    // 메시지 타입이 등록되어 있는지 확인
    if (topic_types_.find(topic_name) == topic_types_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Topic %s not registered for introspection", 
                 topic_name.c_str());
      return false;
    }
    
    // 간단한 메시지 처리 (실제 파싱은 수행하지 않고 기본값만 추가)
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    values.push_back({topic_name + "/timestamp", static_cast<double>(clock.now().seconds())});
    values.push_back({topic_name + "/size", static_cast<double>(message_data.size())});
    
    return true;
  } catch (const std::exception& e) {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "Error processing message on topic %s: %s", 
                 topic_name.c_str(), e.what());
    }
    return false;
  }
}

} // namespace ros2_introspection 