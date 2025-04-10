#include <ros2_introspection/message_formatter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

// 콘솔 출력 포맷터
void ConsoleMessageFormatter::formatSingleMessage(
  const rclcpp::Logger& logger,
  const std::string& topic,
  const Ros2Introspection::RenamedValues& values)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(6);
  
  ss << "Message from topic [" << topic << "], fields: " << values.size() << std::endl;
  
  for (const auto& [name, value] : values) {
    ss << "  " << name << ": " << value << std::endl;
  }
  
  RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

void ConsoleMessageFormatter::formatAllMessages(
  const rclcpp::Logger& logger,
  const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
  const std::map<std::string, size_t>& message_counts)
{
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  double now = clock.now().seconds();
  
  std::stringstream ss;
  ss << std::fixed << std::setprecision(6);
  
  ss << "Current status at time " << now << ":" << std::endl;
  ss << "Total topics: " << messages.size() << std::endl;
  
  for (const auto& [topic, values] : messages) {
    auto count_it = message_counts.find(topic);
    size_t count = (count_it != message_counts.end()) ? count_it->second : 0;
    
    ss << "Topic [" << topic << "], message count: " << count << ", fields: " << values.size() << std::endl;
    
    for (const auto& [name, value] : values) {
      ss << "  " << name << ": " << value << std::endl;
    }
  }
  
  RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

// JSON 출력 포맷터
void JsonMessageFormatter::formatSingleMessage(
  const rclcpp::Logger& logger,
  const std::string& topic,
  const Ros2Introspection::RenamedValues& values)
{
  try {
    nlohmann::json json_data;
    json_data["topic"] = topic;
    json_data["timestamp"] = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    
    // 데이터 필드 생성
    nlohmann::json& data = json_data["data"];
    
    // 문자열 필드 처리 관련 변수
    std::map<std::string, std::string> string_fields;
    
    // 먼저 문자열 필드가 있는지 확인
    for (const auto& value_pair : values) {
      const auto& key = value_pair.first;
      
      // "_text:" 접미사가 있는 필드는 문자열 값을 나타냄
      std::size_t text_pos = key.find("_text:");
      if (text_pos != std::string::npos) {
        std::string base_key = key.substr(0, text_pos);
        std::string text_value = key.substr(text_pos + 6); // "_text:" 이후의 문자열
        string_fields[base_key] = text_value;
      }
    }
    
    // 모든 값을 JSON 오브젝트에 추가
    for (const auto& value_pair : values) {
      const auto& key = value_pair.first;
      const auto& value = value_pair.second;
      
      // 문자열 관련 특수 필드는 건너뜀
      if (key.find("_text:") != std::string::npos || 
          key.find("_value") != std::string::npos) {
        continue;
      }
      
      // 문자열 필드인지 확인
      auto it = string_fields.find(key);
      if (it != string_fields.end()) {
        // 문자열 필드이면 실제 문자열 값으로 대체
        data[key] = it->second;
      } else {
        // 일반 필드는 그대로 추가
        data[key] = value;
      }
    }
    
    // JSON 메시지 출력
    std::string json_str = json_data.dump(2);  // 2 space indentation
    RCLCPP_INFO(logger, "%s", json_str.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error formatting JSON for topic %s: %s", topic.c_str(), e.what());
  }
}

void JsonMessageFormatter::formatAllMessages(
  const rclcpp::Logger& logger,
  const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
  const std::map<std::string, size_t>& message_counts)
{
  try {
    json j;
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    j["timestamp"] = clock.now().seconds();
    
    // 토픽 목록 추가
    json topics_array = json::array();
    
    for (const auto& [topic, values] : messages) {
      if (values.empty()) {
        continue; // 값이 없는 경우 건너뜀
      }
      
      json topic_obj;
      topic_obj["name"] = topic;
      
      auto count_it = message_counts.find(topic);
      size_t count = (count_it != message_counts.end()) ? count_it->second : 0;
      topic_obj["count"] = count;
      
      // 문자열 필드 처리를 위한 맵
      std::map<std::string, std::string> string_fields;
      
      // 먼저 문자열 필드가 있는지 확인
      for (const auto& value_pair : values) {
        const auto& key = value_pair.first;
        
        // "_text:" 접미사가 있는 필드는 문자열 값을 나타냄
        std::size_t text_pos = key.find("_text:");
        if (text_pos != std::string::npos) {
          std::string base_key = key.substr(0, text_pos);
          std::string text_value = key.substr(text_pos + 6); // "_text:" 이후의 문자열
          string_fields[base_key] = text_value;
        }
      }
      
      // 데이터 필드 추가
      json data;
      for (const auto& [name, value] : values) {
        try {
          // 문자열 관련 특수 필드는 건너뜀
          if (name.find("_text:") != std::string::npos || 
              name.find("_value") != std::string::npos) {
            continue;
          }
          
          // 문자열 필드인지 확인
          auto it = string_fields.find(name);
          if (it != string_fields.end()) {
            // 문자열 필드이면 실제 문자열 값으로 대체
            data[name] = it->second;
          } else {
            // 일반 필드는 그대로 추가
            data[name] = value;
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(logger, "Error adding field %s in topic %s: %s", 
                     name.c_str(), topic.c_str(), e.what());
        }
      }
      
      topic_obj["data"] = data;
      topics_array.push_back(topic_obj);
    }
    
    j["topics"] = topics_array;
    
    RCLCPP_INFO(logger, "%s", j.dump(2).c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error formatting JSON for all messages: %s", e.what());
  }
}

// CSV 출력 포맷터
void CsvMessageFormatter::formatSingleMessage(
  const rclcpp::Logger& logger,
  const std::string& topic,
  const Ros2Introspection::RenamedValues& values)
{
  std::stringstream ss;
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  double now = clock.now().seconds();
  
  ss << "timestamp,topic,field,value" << std::endl;
  
  for (const auto& [name, value] : values) {
    ss << now << "," << topic << "," << name << "," << value << std::endl;
  }
  
  RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

void CsvMessageFormatter::formatAllMessages(
  const rclcpp::Logger& logger,
  const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
  const std::map<std::string, size_t>& message_counts)
{
  std::stringstream ss;
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  double now = clock.now().seconds();
  
  ss << "timestamp,topic,field,value,message_count" << std::endl;
  
  for (const auto& [topic, values] : messages) {
    auto count_it = message_counts.find(topic);
    size_t count = (count_it != message_counts.end()) ? count_it->second : 0;
    
    for (const auto& [name, value] : values) {
      ss << now << "," << topic << "," << name << "," << value << "," << count << std::endl;
    }
  }
  
  RCLCPP_INFO(logger, "%s", ss.str().c_str());
}
