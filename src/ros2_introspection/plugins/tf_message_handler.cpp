#include <ros2_introspection/tf_message_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

namespace ros2_introspection {
namespace plugins {

TFMessageHandler::TFMessageHandler()
  : node_(nullptr)
{
}

bool TFMessageHandler::supportsMessageType(const std::string& message_type) const
{
  // TF 메시지 타입 확인
  return (message_type == "tf2_msgs/msg/TFMessage" || 
          message_type == "tf2_msgs/TFMessage" ||
          message_type == "tf2_msgs::msg::TFMessage");
}

bool TFMessageHandler::initialize(
  rclcpp::Node* node,
  const std::string& topic,
  const std::string& /* message_type */)
{
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("tf_message_handler"), 
                "Null node passed to TFMessageHandler::initialize");
    return false;
  }
  
  node_ = node;
  RCLCPP_INFO(node_->get_logger(), 
             "Initialized TFMessageHandler for topic: %s", topic.c_str());
  return true;
}

bool TFMessageHandler::handleMessage(
  const std::string& topic,
  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
  Ros2Introspection::RenamedValues& values)
{
  if (!serialized_msg) {
    RCLCPP_ERROR(node_->get_logger(), "Received null serialized message");
    return false;
  }
  
  // 메시지 역직렬화
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
  tf2_msgs::msg::TFMessage tf_msg;
  
  try {
    serialization.deserialize_message(serialized_msg.get(), &tf_msg);
  } catch (const rclcpp::exceptions::RCLError& e) {
    RCLCPP_ERROR(node_->get_logger(), 
                "Failed to deserialize TF message: %s", e.what());
    return false;
  }
  
  // TF 메시지를 처리하여 RenamedValues로 변환
  values.clear();
  
  // 타임스탬프는 현재 시간 사용
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  
  // transforms 배열의 각 요소 처리
  for (size_t i = 0; i < tf_msg.transforms.size(); ++i) {
    const auto& transform = tf_msg.transforms[i];
    
    // Header
    values.push_back({topic + "/transforms." + std::to_string(i) + "/header/stamp/sec", 
                     static_cast<double>(transform.header.stamp.sec)});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/header/stamp/nanosec", 
                     static_cast<double>(transform.header.stamp.nanosec)});
    
    // 문자열 필드 처리 - 더 안전한 방식으로 수정
    // 접미사에 문자열 값을 직접 포함
    std::string frame_id_key = topic + "/transforms." + std::to_string(i) + "/header/frame_id";
    std::string child_frame_id_key = topic + "/transforms." + std::to_string(i) + "/child_frame_id";
    
    // frame_id와 child_frame_id는 고정값 -1로 대체
    values.push_back({frame_id_key, -1.0});
    values.push_back({child_frame_id_key, -1.0});
    
    // 별도 필드에 문자열 값 정보 저장 (출력용)
    values.push_back({frame_id_key + "_value", static_cast<double>(0)});
    values.push_back({child_frame_id_key + "_value", static_cast<double>(0)});
    
    // 별도 필드에 실제 문자열 저장
    values.push_back({frame_id_key + "_text:" + transform.header.frame_id, 1.0});
    values.push_back({child_frame_id_key + "_text:" + transform.child_frame_id, 1.0});
    
    // Transform - translation
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/x", 
                     transform.transform.translation.x});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/y", 
                     transform.transform.translation.y});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/z", 
                     transform.transform.translation.z});
    
    // Transform - rotation
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/x", 
                     transform.transform.rotation.x});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/y", 
                     transform.transform.rotation.y});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/z", 
                     transform.transform.rotation.z});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/w", 
                     transform.transform.rotation.w});
  }
  
  // 추가 정보: 변환 개수
  values.push_back({topic + "/transforms_count", static_cast<double>(tf_msg.transforms.size())});
  
  return true;
}

bool TFMessageHandler::createSubscription(
  rclcpp::Node* node,
  const std::string& topic,
  const std::string& /* message_type */,
  std::function<void(const std::string&, const Ros2Introspection::RenamedValues&)> callback)
{
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("tf_message_handler"), 
                "Null node passed to TFMessageHandler::createSubscription");
    return false;
  }
  
  node_ = node;
  user_callback_ = callback;
  
  // TF 메시지 구독 생성
  subscription_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    topic, rclcpp::QoS(10).best_effort(),
    [this, topic](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      this->tfCallback(topic, msg);
    });
  
  if (!subscription_) {
    RCLCPP_ERROR(node_->get_logger(), 
                "Failed to create subscription for TF topic: %s", topic.c_str());
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
             "Created TF subscription for topic: %s", topic.c_str());
  return true;
}

void TFMessageHandler::tfCallback(
  const std::string& topic, 
  const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  if (!user_callback_) {
    return;
  }
  
  Ros2Introspection::RenamedValues values;
  
  // transforms 배열의 각 요소 처리
  for (size_t i = 0; i < msg->transforms.size(); ++i) {
    const auto& transform = msg->transforms[i];
    
    // Header
    values.push_back({topic + "/transforms." + std::to_string(i) + "/header/stamp/sec", 
                     static_cast<double>(transform.header.stamp.sec)});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/header/stamp/nanosec", 
                     static_cast<double>(transform.header.stamp.nanosec)});
    
    // 문자열 필드 처리 - 더 안전한 방식으로 수정
    // 접미사에 문자열 값을 직접 포함
    std::string frame_id_key = topic + "/transforms." + std::to_string(i) + "/header/frame_id";
    std::string child_frame_id_key = topic + "/transforms." + std::to_string(i) + "/child_frame_id";
    
    // frame_id와 child_frame_id는 고정값 -1로 대체
    values.push_back({frame_id_key, -1.0});
    values.push_back({child_frame_id_key, -1.0});
    
    // 별도 필드에 문자열 값 정보 저장 (출력용)
    values.push_back({frame_id_key + "_value", static_cast<double>(0)});
    values.push_back({child_frame_id_key + "_value", static_cast<double>(0)});
    
    // 별도 필드에 실제 문자열 저장
    values.push_back({frame_id_key + "_text:" + transform.header.frame_id, 1.0});
    values.push_back({child_frame_id_key + "_text:" + transform.child_frame_id, 1.0});
    
    // Transform - translation
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/x", 
                     transform.transform.translation.x});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/y", 
                     transform.transform.translation.y});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/translation/z", 
                     transform.transform.translation.z});
    
    // Transform - rotation
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/x", 
                     transform.transform.rotation.x});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/y", 
                     transform.transform.rotation.y});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/z", 
                     transform.transform.rotation.z});
    values.push_back({topic + "/transforms." + std::to_string(i) + "/transform/rotation/w", 
                     transform.transform.rotation.w});
  }
  
  // 추가 정보: 변환 개수
  values.push_back({topic + "/transforms_count", static_cast<double>(msg->transforms.size())});
  
  // 사용자 콜백 호출
  user_callback_(topic, values);
}

// 플러그인 클래스 등록
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros2_introspection::plugins::TFMessageHandler, 
                       ros2_introspection::MessageHandlerPlugin)

} // namespace plugins
} // namespace ros2_introspection 