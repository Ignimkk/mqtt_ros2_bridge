#pragma once

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <ros2_introspection/message_handler_plugin.hpp>
#include <pluginlib/class_loader.hpp>

namespace ros2_introspection {

/**
 * @brief 메시지 핸들러 플러그인을 로드하고 관리하는 클래스
 */
class PluginLoader {
public:
  /**
   * @brief 생성자
   */
  PluginLoader();
  
  /**
   * @brief 소멸자
   */
  ~PluginLoader();
  
  /**
   * @brief 가능한 모든 플러그인 로드
   * 
   * @return true 하나 이상의 플러그인 로드 성공
   * @return false 모든 플러그인 로드 실패
   */
  bool loadPlugins();
  
  /**
   * @brief 지정된 메시지 타입을 지원하는 플러그인 가져오기
   * 
   * @param message_type 메시지 타입 (예: "tf2_msgs/msg/TFMessage")
   * @return MessageHandlerPluginPtr 플러그인 인스턴스 (지원하는 플러그인이 없으면 nullptr)
   */
  MessageHandlerPluginPtr getPluginForMessageType(const std::string& message_type);
  
  /**
   * @brief 로드된 모든 플러그인 목록 가져오기
   * 
   * @return const std::vector<MessageHandlerPluginPtr>& 플러그인 목록
   */
  const std::vector<MessageHandlerPluginPtr>& getLoadedPlugins() const;
  
  /**
   * @brief 메시지 타입과 연결된 플러그인 검색
   * 
   * @param message_type 메시지 타입
   * @return true 해당 메시지 타입에 대한 플러그인이 있음
   * @return false 해당 메시지 타입에 대한 플러그인이 없음
   */
  bool hasPluginFor(const std::string& message_type) const;

private:
  // 플러그인 로더
  std::unique_ptr<pluginlib::ClassLoader<MessageHandlerPlugin>> class_loader_;
  
  // 로드된 모든 플러그인 인스턴스
  std::vector<MessageHandlerPluginPtr> plugins_;
  
  // 메시지 타입별 플러그인 맵핑
  std::map<std::string, MessageHandlerPluginPtr> type_to_plugin_map_;
};

} // namespace ros2_introspection 