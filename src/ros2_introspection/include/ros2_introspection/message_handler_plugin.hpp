#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <ros2_introspection/ros2_introspection.hpp>
#include <class_loader/class_loader.hpp>

namespace ros2_introspection {

/**
 * @brief 메시지 핸들러 플러그인 인터페이스
 * 
 * 각 메시지 타입별 특화된 처리를 위한 플러그인 인터페이스입니다.
 * 새로운 메시지 타입을 지원하려면 이 인터페이스를 구현하세요.
 */
class MessageHandlerPlugin {
public:
  virtual ~MessageHandlerPlugin() = default;
  
  /**
   * @brief 플러그인이 지원하는 메시지 타입 검사
   * 
   * @param message_type 메시지 타입 문자열 (예: "tf2_msgs/msg/TFMessage")
   * @return true 이 플러그인이 해당 메시지 타입을 지원하는 경우
   * @return false 이 플러그인이 해당 메시지 타입을 지원하지 않는 경우
   */
  virtual bool supportsMessageType(const std::string& message_type) const = 0;
  
  /**
   * @brief 플러그인 초기화
   * 
   * @param node 플러그인이 사용할 ROS 노드
   * @param topic 처리할 토픽 이름
   * @param message_type 메시지 타입
   * @return true 초기화 성공
   * @return false 초기화 실패
   */
  virtual bool initialize(
    rclcpp::Node* node,
    const std::string& topic,
    const std::string& message_type) = 0;
  
  /**
   * @brief 직렬화된 메시지 처리
   * 
   * @param topic 토픽 이름
   * @param serialized_msg 직렬화된 메시지
   * @param values 출력: 처리된 값을 저장할 컨테이너
   * @return true 메시지 처리 성공
   * @return false 메시지 처리 실패
   */
  virtual bool handleMessage(
    const std::string& topic,
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg,
    Ros2Introspection::RenamedValues& values) = 0;
    
  /**
   * @brief 구독 생성 및 설정
   * 
   * @param node 구독을 생성할 ROS 노드
   * @param topic 구독할 토픽 이름
   * @param message_type 메시지 타입
   * @param callback 메시지 수신 시 호출할 콜백 함수
   * @return true 구독 설정 성공
   * @return false 구독 설정 실패
   */
  virtual bool createSubscription(
    rclcpp::Node* node,
    const std::string& topic,
    const std::string& message_type,
    std::function<void(const std::string&, const Ros2Introspection::RenamedValues&)> callback) = 0;
    
  /**
   * @brief 플러그인 이름 반환
   * 
   * @return 플러그인 이름
   */
  virtual std::string getName() const = 0;
};

using MessageHandlerPluginPtr = std::shared_ptr<MessageHandlerPlugin>;

} // namespace ros2_introspection 