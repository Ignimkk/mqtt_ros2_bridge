#pragma once

#include "rclcpp/logger.hpp"
#include <ros2_introspection/ros2_introspection.hpp>
#include <string>
#include <map>

class MessageFormatter
{
public:
  virtual ~MessageFormatter() = default;
  
  virtual void formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values) = 0;
    
  virtual void formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts) = 0;
};

// 콘솔 출력 포맷터
class ConsoleMessageFormatter : public MessageFormatter
{
public:
  void formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values) override;
    
  void formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts) override;
};

// JSON 출력 포맷터
class JsonMessageFormatter : public MessageFormatter
{
public:
  void formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values) override;
    
  void formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts) override;
};

// CSV 출력 포맷터
class CsvMessageFormatter : public MessageFormatter
{
public:
  void formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values) override;
    
  void formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts) override;
};
