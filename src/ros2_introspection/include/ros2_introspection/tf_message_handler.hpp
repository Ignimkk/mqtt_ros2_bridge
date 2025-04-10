#pragma once

#include <ros2_introspection/message_handler_plugin.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>

namespace ros2_introspection {
namespace plugins {

/**
 * @brief TF 메시지 처리 플러그인
 * 
 * tf2_msgs/msg/TFMessage 메시지를 효율적으로 처리하는 플러그인입니다.
 */
class TFMessageHandler : public MessageHandlerPlugin {
public:
  TFMessageHandler();
  virtual ~TFMessageHandler() = default;
  
  /**
   * @brief 이 플러그인이 지원하는 메시지 타입 검사
   */
  bool supportsMessageType(const std::string& message_type) const override;
  
  /**
   * @brief 플러그인 초기화
   */
  bool initialize(
    rclcpp::Node* node,
    const std::string& topic,
    const std::string& message_type) override;
  
  /**
   * @brief 직렬화된 메시지 처리
   */
  bool handleMessage(
    const std::string& topic,
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
    Ros2Introspection::RenamedValues& values) override;
    
  /**
   * @brief 구독 생성 및 설정
   */
  bool createSubscription(
    rclcpp::Node* node,
    const std::string& topic,
    const std::string& message_type,
    std::function<void(const std::string&, const Ros2Introspection::RenamedValues&)> callback) override;
    
  /**
   * @brief 플러그인 이름 반환
   */
  std::string getName() const override {
    return "TFMessageHandler";
  }

private:
  // TF 메시지 콜백
  void tfCallback(
    const std::string& topic, 
    const tf2_msgs::msg::TFMessage::SharedPtr msg);
  
  // 내부 콜백 함수
  std::function<void(const std::string&, const Ros2Introspection::RenamedValues&)> user_callback_;
  
  // 구독자
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  
  // 노드 참조
  rclcpp::Node* node_;
  
  // 문자열 캐싱을 위한 멤버 변수
  std::unordered_map<std::string, size_t> string_to_id_;
  std::vector<std::string> id_to_string_;
  
  // 문자열을 캐시에 추가하고 ID 반환
  size_t addStringToCache(const std::string& str) {
    auto it = string_to_id_.find(str);
    if (it != string_to_id_.end()) {
      return it->second;
    }
    
    size_t id = id_to_string_.size();
    string_to_id_[str] = id;
    id_to_string_.push_back(str);
    return id;
  }
  
  // ID로부터 문자열 조회
  std::string getStringFromCache(size_t id) const {
    if (id < id_to_string_.size()) {
      return id_to_string_[id];
    }
    return "";
  }
};

} // namespace plugins
} // namespace ros2_introspection 