#ifndef ROS2_INTROSPECTION_MESSAGE_FORMATTER_HPP
#define ROS2_INTROSPECTION_MESSAGE_FORMATTER_HPP

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <set>
#include <nlohmann/json.hpp>
#include "rclcpp/logger.hpp"
#include <ros2_introspection/ros2_introspection.hpp>
#include <mosquitto.h>

namespace ros2_introspection
{

// MQTT 설정을 저장하는 구조체
struct MqttConfig {
    bool enabled = false;
    std::string host = "localhost";
    int port = 1883;
    std::string client_id = "ros2_introspection";
    std::map<std::string, std::string> topic_mapping;  // ROS 토픽 -> MQTT 토픽 매핑
    int qos = 0;
    bool retain = false;
};

class MessageFormatter 
{
public:
    struct FormatOptions {
        FormatOptions() 
            : pretty_print(true)
            , indent_level(2)
            , compact_arrays(true)
            , show_timestamps(true)
            , group_similar_fields(true)
            , structured_format(true)  // 구조화된 형식 사용 여부
        {}

        bool pretty_print;
        int indent_level;
        bool compact_arrays;
        bool show_timestamps;
        bool group_similar_fields;  // 유사한 필드들을 그룹화
        bool structured_format;     // 구조화된 형식 사용 여부
    };

    // 새로운 인터페이스
    static std::string formatMessage(
        const std::string& topic_name,
        const std::string& json_data,
        const FormatOptions& options = FormatOptions());

    // 이전 인터페이스와의 호환성을 위한 메서드들
    virtual ~MessageFormatter() = default;
    
    virtual void formatSingleMessage(
        const rclcpp::Logger& logger,
        const std::string& topic,
        const Ros2Introspection::RenamedValues& values);
        
    virtual void formatAllMessages(
        const rclcpp::Logger& logger,
        const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
        const std::map<std::string, size_t>& message_counts);
    
    // MQTT 관련 메서드
    void initMqtt(const MqttConfig& config);
    void publishToMqtt(const std::string& topic, const nlohmann::json& data);
    void closeMqtt();
    bool isMqttEnabled() const { return mqtt_config_.enabled; }
    const MqttConfig& getMqttConfig() const { return mqtt_config_; }

protected:
    // 필드 그룹 정보를 저장하는 구조체
    struct FieldGroup {
        FieldGroup() = default;
        FieldGroup(const std::string& bn, const std::string& fp)
            : base_name(bn), full_prefix(fp) {}

        std::string base_name;           // 기본 필드 이름 (예: "position", "velocity")
        std::string full_prefix;         // 전체 경로 (예: "/joint_states/position")
        std::vector<double> values;      // 값들의 배열
        std::map<std::string, std::string> metadata;  // 추가 메타데이터
    };

    static std::string formatFields(
        const nlohmann::json& data,
        const FormatOptions& options);

    // 구조화된 형식으로 변환하는 새로운 메서드
    static nlohmann::json convertToStructured(const nlohmann::json& flat_data);
    
    // 새로운 구조화된 포맷 메서드
    static std::string formatStructured(
        const nlohmann::json& structured_data,
        const FormatOptions& options);

    // JSON을 YAML 스타일로 변환하는 메서드
    static std::string formatJsonAsYaml(
        const nlohmann::json& json_data,
        int depth,
        const FormatOptions& options);

    static std::vector<FieldGroup> groupSimilarFields(
        const nlohmann::json& data);

    static std::string formatFieldGroup(
        const FieldGroup& group,
        const FormatOptions& options);

    static std::string formatValue(
        const nlohmann::json& value,
        const FormatOptions& options);

    static std::string indent(int level);
    static std::string formatNumber(double value);
    static std::string getBaseName(const std::string& full_path);
    static std::string getPrefix(const std::string& full_path);

    // 유틸리티 함수: RenamedValues를 JSON으로 변환
    static nlohmann::json renamedValuesToJson(const Ros2Introspection::RenamedValues& values);
    
private:
    // MQTT 관련 멤버 변수
    MqttConfig mqtt_config_;
    struct mosquitto* mosq_ = nullptr;
    bool mqtt_connected_ = false;
};

// 기존 코드와의 호환성을 위한 포맷터 클래스들
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

class JsonMessageFormatter : public MessageFormatter
{
public:
    JsonMessageFormatter() = default;
    ~JsonMessageFormatter() override;
    
    void formatSingleMessage(
        const rclcpp::Logger& logger,
        const std::string& topic,
        const Ros2Introspection::RenamedValues& values) override;
        
    void formatAllMessages(
        const rclcpp::Logger& logger,
        const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
        const std::map<std::string, size_t>& message_counts) override;
};

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

} // namespace ros2_introspection

#endif // ROS2_INTROSPECTION_MESSAGE_FORMATTER_HPP
