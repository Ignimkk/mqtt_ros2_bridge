#include "ros2_introspection/message_formatter.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <regex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <fstream>

namespace ros2_introspection
{

// MQTT 관련 메서드 구현
void MessageFormatter::initMqtt(const MqttConfig& config)
{
    mqtt_config_ = config;
    
    if (!mqtt_config_.enabled) {
        return;
    }
    
    // libmosquitto 초기화
    mosquitto_lib_init();
    
    // 클라이언트 인스턴스 생성
    mosq_ = mosquitto_new(mqtt_config_.client_id.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("Failed to create MQTT client instance");
    }
    
    // 서버 연결
    int rc = mosquitto_connect(mosq_, mqtt_config_.host.c_str(), mqtt_config_.port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::string error_msg = "Failed to connect to MQTT broker: " + std::string(mosquitto_strerror(rc));
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        throw std::runtime_error(error_msg);
    }
    
    // 백그라운드 처리 시작
    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::string error_msg = "Failed to start MQTT loop: " + std::string(mosquitto_strerror(rc));
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        throw std::runtime_error(error_msg);
    }
    
    mqtt_connected_ = true;
}

void MessageFormatter::publishToMqtt(const std::string& topic, const nlohmann::json& data)
{
    if (!mqtt_config_.enabled || !mosq_ || !mqtt_connected_) {
        return;
    }
    
    // ROS 토픽에 해당하는 MQTT 토픽 찾기
    auto it = mqtt_config_.topic_mapping.find(topic);
    if (it == mqtt_config_.topic_mapping.end()) {
        // 매핑이 없으면 발행하지 않음
        return;
    }
    
    // JSON 데이터를 문자열로 변환
    std::string payload = data.dump();
    
    // MQTT로 메시지 발행
    int rc = mosquitto_publish(mosq_, nullptr, it->second.c_str(), payload.size(), 
                              payload.c_str(), mqtt_config_.qos, mqtt_config_.retain);
    
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("message_formatter"), 
                    "Failed to publish to MQTT topic %s: %s", 
                    it->second.c_str(), mosquitto_strerror(rc));
    }
}

void MessageFormatter::closeMqtt()
{
    if (mosq_ && mqtt_connected_) {
        // 연결 종료
        mosquitto_disconnect(mosq_);
        mosquitto_loop_stop(mosq_, true);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mqtt_connected_ = false;
    }
    
    // 라이브러리 정리
    mosquitto_lib_cleanup();
}

std::string MessageFormatter::formatMessage(
    const std::string& topic_name,
    const std::string& json_data,
    const FormatOptions& options)
{
    try {
        auto json = nlohmann::json::parse(json_data);
        
        // Check if this is a dynamic_introspection message
        if (json.contains("data")) {
            json = json["data"];
        }

        std::stringstream ss;
        ss << "Topic: " << topic_name << "\n";
        
        if (options.show_timestamps && json.contains("header/stamp/sec")) {
            double sec = json["header/stamp/sec"].get<double>();
            double nsec = json["header/stamp/nanosec"].get<double>();
            ss << "Timestamp: " << sec + nsec * 1e-9 << "\n";
        }

        // 구조화된 형식으로 출력할지 여부 결정
        if (options.structured_format) {
            auto structured_data = convertToStructured(json);
            ss << formatStructured(structured_data, options);
        } else {
            ss << formatFields(json, options);
        }
        
        return ss.str();
    } catch (const std::exception& e) {
        return "Error formatting message: " + std::string(e.what());
    }
}

void MessageFormatter::formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values)
{
    try {
        nlohmann::json json_data = renamedValuesToJson(values);
        std::string json_str = json_data.dump();
        
        // 기본 포맷 옵션으로 메시지 포맷
        FormatOptions options;
        std::string formatted = formatMessage(topic, json_str, options);
        
        // 로거를 통해 출력
        RCLCPP_INFO(logger, "%s", formatted.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error in formatSingleMessage: %s", e.what());
    }
}

void MessageFormatter::formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts)
{
    try {
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        double now = clock.now().seconds();
        
        std::stringstream ss;
        ss << "Current status at time " << now << ":\n";
        ss << "Total topics: " << messages.size() << "\n\n";
        
        for (const auto& [topic, values] : messages) {
            auto count_it = message_counts.find(topic);
            size_t count = (count_it != message_counts.end()) ? count_it->second : 0;
            
            // 각 토픽에 대한 정보 추가
            ss << "Topic: " << topic << "\n";
            ss << "Message count: " << count << "\n";
            
            // 값을 JSON으로 변환 후 포맷
            nlohmann::json json_data = renamedValuesToJson(values);
            std::string json_str = json_data.dump();
            
            // 기본 포맷 옵션으로 메시지 포맷 (타이틀 제외)
            FormatOptions options;
            std::string formatted = formatFields(json_data, options);
            ss << formatted << "\n";
        }
        
        // 로거를 통해 출력
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error in formatAllMessages: %s", e.what());
    }
}

nlohmann::json MessageFormatter::renamedValuesToJson(const Ros2Introspection::RenamedValues& values)
{
    nlohmann::json json_data;
    
    for (const auto& [name, value] : values) {
        // 문자열 필드 처리 (특수 패턴이 있는 경우)
        size_t text_pos = name.find("_text:");
        if (text_pos != std::string::npos) {
            std::string base_key = name.substr(0, text_pos);
            std::string text_value = name.substr(text_pos + 6);  // "_text:" 이후
            json_data[base_key] = text_value;
        } else if (name.find("_value") == std::string::npos) {  // "_value" 패턴은 건너뜀
            // 일반 필드
            json_data[name] = value;
        }
    }
    
    return json_data;
}

std::string MessageFormatter::formatFields(
    const nlohmann::json& data,
    const FormatOptions& options)
{
    if (!options.group_similar_fields) {
        // 그룹화하지 않고 일반적인 출력
        std::stringstream ss;
        ss << "Data:\n";
        for (auto& [key, value] : data.items()) {
            ss << indent(options.indent_level) << key << ": "
               << formatValue(value, options) << "\n";
        }
        return ss.str();
    }

    // 유사한 필드들을 그룹화
    auto groups = groupSimilarFields(data);
    
    std::stringstream ss;
    std::string current_prefix = "";
    
    for (const auto& group : groups) {
        std::string prefix = getPrefix(group.full_prefix);
        
        // 새로운 섹션 시작
        if (prefix != current_prefix) {
            if (!current_prefix.empty()) {
                ss << "\n";
            }
            ss << prefix << ":\n";
            current_prefix = prefix;
        }
        
        ss << formatFieldGroup(group, options);
    }
    
    return ss.str();
}

std::vector<MessageFormatter::FieldGroup> MessageFormatter::groupSimilarFields(
    const nlohmann::json& data)
{
    std::map<std::string, FieldGroup> groups;
    
    // 정규식 패턴: 배열 인덱스를 찾기 위한 것
    std::regex array_pattern(R"(^(.+)\.(\d+)$)");
    
    for (auto& [key, value] : data.items()) {
        std::smatch matches;
        if (std::regex_match(key, matches, array_pattern)) {
            // 배열 요소 처리
            std::string base_key = matches[1].str();
            int index = std::stoi(matches[2].str());
            
            // 그룹이 없으면 새로 생성
            if (groups.find(base_key) == groups.end()) {
                groups.emplace(base_key, FieldGroup(getBaseName(base_key), base_key));
            }
            
            // 벡터 크기 확장이 필요한 경우
            if (static_cast<size_t>(index) >= groups[base_key].values.size()) {
                groups[base_key].values.resize(index + 1, 0.0);
            }
            
            // 값 저장
            if (value.is_number()) {
                groups[base_key].values[index] = value.get<double>();
            }
        } else {
            // 일반 필드 처리
            if (value.is_string()) {
                std::string prefix = getPrefix(key);
                std::string base_name = getBaseName(key);
                
                // 메타데이터로 저장
                if (groups.find(prefix) == groups.end()) {
                    groups.emplace(prefix, FieldGroup(base_name, prefix));
                }
                groups[prefix].metadata[base_name] = value.get<std::string>();
            }
        }
    }
    
    // 맵을 벡터로 변환
    std::vector<FieldGroup> result;
    result.reserve(groups.size());
    for (const auto& [_, group] : groups) {
        result.push_back(group);
    }
    
    // 그룹 정렬
    std::sort(result.begin(), result.end(),
        [](const FieldGroup& a, const FieldGroup& b) {
            return a.full_prefix < b.full_prefix;
        });
    
    return result;
}

std::string MessageFormatter::formatFieldGroup(
    const FieldGroup& group,
    const FormatOptions& options)
{
    std::stringstream ss;
    
    // 배열 값이 있는 경우
    if (!group.values.empty()) {
        ss << indent(options.indent_level) 
           << getBaseName(group.full_prefix) << ": [";
        
        for (size_t i = 0; i < group.values.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << formatNumber(group.values[i]);
        }
        ss << "]\n";
    }
    
    // 메타데이터가 있는 경우
    for (const auto& [key, value] : group.metadata) {
        ss << indent(options.indent_level) << key << ": " << value << "\n";
    }
    
    return ss.str();
}

std::string MessageFormatter::formatValue(
    const nlohmann::json& value,
    const FormatOptions& options)
{
    (void)options; // 사용하지 않는 매개변수 경고 제거

    if (value.is_array()) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < value.size(); ++i) {
            if (i > 0) ss << ", ";
            if (value[i].is_number()) {
                ss << formatNumber(value[i].get<double>());
            } else {
                ss << value[i].dump();
            }
        }
        ss << "]";
        return ss.str();
    } else if (value.is_number()) {
        return formatNumber(value.get<double>());
    } else {
        return value.dump();
    }
}

std::string MessageFormatter::indent(int level)
{
    return std::string(level, ' ');
}

std::string MessageFormatter::formatNumber(double value)
{
    std::stringstream ss;
    if (std::abs(value) < 1e-3 || std::abs(value) > 1e3) {
        ss << std::scientific << std::setprecision(3) << value;
    } else {
        ss << std::fixed << std::setprecision(3) << value;
    }
    return ss.str();
}

std::string MessageFormatter::getBaseName(const std::string& full_path)
{
    size_t pos = full_path.find_last_of('/');
    return (pos == std::string::npos) ? full_path : full_path.substr(pos + 1);
}

std::string MessageFormatter::getPrefix(const std::string& full_path)
{
    size_t pos = full_path.find_last_of('/');
    return (pos == std::string::npos) ? "" : full_path.substr(0, pos);
}

// ConsoleMessageFormatter 구현
void ConsoleMessageFormatter::formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values)
{
    // 부모 클래스의 메서드 호출
    MessageFormatter::formatSingleMessage(logger, topic, values);
}

void ConsoleMessageFormatter::formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts)
{
    // 부모 클래스의 메서드 호출
    MessageFormatter::formatAllMessages(logger, messages, message_counts);
}

// JsonMessageFormatter 구현
JsonMessageFormatter::~JsonMessageFormatter()
{
    // MQTT 자원 정리
    closeMqtt();
}

void JsonMessageFormatter::formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values)
{
    try {
        nlohmann::json json_data = renamedValuesToJson(values);
        std::string json_str = json_data.dump();
        
        // 구조화된 포맷 옵션을 사용하는 포맷터로 메시지 포맷
        FormatOptions options;
        options.structured_format = true;  // 구조화된 형식 강제 적용
        std::string formatted = formatMessage(topic, json_str, options);
        
        // 로거를 통해 출력
        RCLCPP_INFO(logger, "%s", formatted.c_str());
        
        // 구조화된 데이터 생성
        auto structured_data = convertToStructured(json_data);
        
        // 테스트 목적으로 JSON 파일 저장
        try {
            // 토픽에서 파일명 생성 - '/'를 '_'로 대체
            std::string filename = topic;
            std::replace(filename.begin(), filename.end(), '/', '_');
            if (filename.empty() || filename[0] == '_') {
                filename = "topic" + filename;
            }
            filename += ".json";
            
            std::ofstream file(filename);
            if (file.is_open()) {
                file << structured_data.dump(2); // 2-space indentation for readability
                file.close();
                RCLCPP_INFO(logger, "Saved structured data to %s", filename.c_str());
            } else {
                RCLCPP_ERROR(logger, "Failed to open file %s for writing", filename.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Error saving JSON file: %s", e.what());
        }
        
        // MQTT로 발행
        if (isMqttEnabled()) {
            publishToMqtt(topic, structured_data);
        }
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
        nlohmann::json json_output;
        json_output["timestamp"] = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
        
        // 토픽 배열 생성
        nlohmann::json topics = nlohmann::json::array();
        
        for (const auto& [topic, values] : messages) {
            nlohmann::json topic_obj;
            topic_obj["name"] = topic;
            
            auto count_it = message_counts.find(topic);
            topic_obj["count"] = (count_it != message_counts.end()) ? count_it->second : 0;
            
            // 데이터 필드 추가
            topic_obj["data"] = renamedValuesToJson(values);
            
            topics.push_back(topic_obj);
        }
        
        json_output["topics"] = topics;
        
        // JSON 문자열로 변환 후 출력
        std::string json_str = json_output.dump(2);  // 2 space indentation
        RCLCPP_INFO(logger, "%s", json_str.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error formatting JSON for all messages: %s", e.what());
    }
}

// CsvMessageFormatter 구현
void CsvMessageFormatter::formatSingleMessage(
    const rclcpp::Logger& logger,
    const std::string& topic,
    const Ros2Introspection::RenamedValues& values)
{
    try {
        std::stringstream ss;
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        double now = clock.now().seconds();
        
        ss << "timestamp,topic,field,value" << std::endl;
        
        for (const auto& [name, value] : values) {
            ss << now << "," << topic << "," << name << "," << value << std::endl;
        }
        
        RCLCPP_INFO(logger, "%s", ss.str().c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error formatting CSV for topic %s: %s", topic.c_str(), e.what());
    }
}

void CsvMessageFormatter::formatAllMessages(
    const rclcpp::Logger& logger,
    const std::map<std::string, Ros2Introspection::RenamedValues>& messages,
    const std::map<std::string, size_t>& message_counts)
{
    try {
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
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error formatting CSV for all messages: %s", e.what());
    }
}

nlohmann::json MessageFormatter::convertToStructured(const nlohmann::json& flat_data)
{
    nlohmann::json result;
    
    // /tf/transforms.0/... -> transforms[0]...
    // /joint_states/position.0 -> position[0]
    std::map<std::string, std::map<int, nlohmann::json>> arrays;
    std::regex array_pattern(R"(^(.+)\.(\d+)\/(.+)$)");
    std::regex simple_array_pattern(R"(^(.+)\.(\d+)$)");
    
    // 먼저 기본 네임스페이스를 식별 (예: joint_states/, tf/)
    std::set<std::string> namespaces;
    
    for (const auto& [key, value] : flat_data.items()) {
        std::string path = key;
        if (path[0] == '/') {
            path = path.substr(1);  // 맨 앞의 / 제거
        }
        
        // 첫 번째 '/' 기준으로 네임스페이스 추출
        size_t slash_pos = path.find('/');
        if (slash_pos != std::string::npos) {
            namespaces.insert(path.substr(0, slash_pos));
        }
    }
    
    // 각 키를 네임스페이스 기준으로 올바른 위치에 배치
    for (const auto& [key, value] : flat_data.items()) {
        std::string path = key;
        if (path[0] == '/') {
            path = path.substr(1);  // 맨 앞의 / 제거
        }
        
        // 네임스페이스 확인
        std::string current_ns = "";
        size_t ns_length = 0;
        
        for (const auto& ns : namespaces) {
            if (path.find(ns + "/") == 0 && ns.length() > ns_length) {
                current_ns = ns;
                ns_length = ns.length();
            }
        }
        
        // 네임스페이스가 있으면 경로에서 제거하고 계층 구조로 변환
        std::string adjusted_path;
        if (!current_ns.empty()) {
            adjusted_path = path.substr(current_ns.length() + 1); // +1 for the trailing '/'
        } else {
            adjusted_path = path;
        }
        
        std::smatch matches;
        // 배열 내 객체 처리 (예: transforms.0/header/frame_id)
        if (std::regex_match(adjusted_path, matches, array_pattern)) {
            std::string array_name = matches[1].str();
            int index = std::stoi(matches[2].str());
            std::string field_path = matches[3].str();
            
            // JSON 경로를 점(.)으로 분리
            std::vector<std::string> path_parts;
            std::string current;
            for (char c : field_path) {
                if (c == '/') {
                    if (!current.empty()) {
                        path_parts.push_back(current);
                        current.clear();
                    }
                } else {
                    current += c;
                }
            }
            if (!current.empty()) {
                path_parts.push_back(current);
            }
            
            // 네임스페이스가 있으면 해당 네임스페이스 내에 배열 생성
            nlohmann::json* target;
            if (!current_ns.empty()) {
                if (!result.contains(current_ns)) {
                    result[current_ns] = nlohmann::json::object();
                }
                target = &arrays[current_ns + "/" + array_name][index];
            } else {
                target = &arrays[array_name][index];
            }
            
            // 중첩된 객체 생성
            nlohmann::json* current_obj = target;
            for (size_t i = 0; i < path_parts.size() - 1; ++i) {
                if (!current_obj->contains(path_parts[i])) {
                    (*current_obj)[path_parts[i]] = nlohmann::json::object();
                }
                current_obj = &(*current_obj)[path_parts[i]];
            }
            
            // 최종 값 할당
            (*current_obj)[path_parts.back()] = value;
        }
        // 단순 배열 처리 (예: position.0)
        else if (std::regex_match(adjusted_path, matches, simple_array_pattern)) {
            std::string array_name = matches[1].str();
            int index = std::stoi(matches[2].str());
            
            // 네임스페이스가 있으면 해당 네임스페이스 내에 배열 저장
            std::string full_array_name;
            if (!current_ns.empty()) {
                if (!result.contains(current_ns)) {
                    result[current_ns] = nlohmann::json::object();
                }
                full_array_name = current_ns + "/" + array_name;
            } else {
                full_array_name = array_name;
            }
            
            if (arrays.find(full_array_name) == arrays.end()) {
                arrays[full_array_name] = std::map<int, nlohmann::json>();
            }
            
            arrays[full_array_name][index] = value;
        }
        // 일반 필드
        else {
            std::vector<std::string> path_parts;
            std::string current;
            
            // 네임스페이스가 있는 경우 먼저 해당 네임스페이스 객체 생성
            nlohmann::json* target = &result;
            if (!current_ns.empty()) {
                if (!result.contains(current_ns)) {
                    result[current_ns] = nlohmann::json::object();
                }
                target = &result[current_ns];
            }
            
            for (char c : adjusted_path) {
                if (c == '/') {
                    if (!current.empty()) {
                        path_parts.push_back(current);
                        current.clear();
                    }
                } else {
                    current += c;
                }
            }
            if (!current.empty()) {
                path_parts.push_back(current);
            }
            
            // 중첩된 객체 생성
            nlohmann::json* current_obj = target;
            for (size_t i = 0; i < path_parts.size() - 1; ++i) {
                if (!current_obj->contains(path_parts[i])) {
                    (*current_obj)[path_parts[i]] = nlohmann::json::object();
                }
                current_obj = &(*current_obj)[path_parts[i]];
            }
            
            // 최종 값 할당
            if (!path_parts.empty()) {
                (*current_obj)[path_parts.back()] = value;
            }
        }
    }
    
    // 배열 처리
    for (const auto& [array_name, indices] : arrays) {
        // 네임스페이스와 실제 배열 이름 분리
        std::string ns = "";
        std::string actual_array_name = array_name;
        
        size_t slash_pos = array_name.find('/');
        if (slash_pos != std::string::npos) {
            ns = array_name.substr(0, slash_pos);
            actual_array_name = array_name.substr(slash_pos + 1);
        }
        
        // 배열 내 객체들이 있는 경우 (transforms 등)
        if (!indices.empty() && indices.begin()->second.is_object()) {
            nlohmann::json array = nlohmann::json::array();
            // 인덱스 순서대로 정렬
            for (const auto& [idx, obj] : indices) {
                // idx가 array.size()보다 크면 그 사이에 null을 채움
                while (array.size() < static_cast<size_t>(idx)) {
                    array.push_back(nullptr);
                }
                array.push_back(obj);
            }
            
            // 타겟 객체 결정
            if (!ns.empty()) {
                // 중첩 경로 처리 (예: actual_array_name이 여러 레벨을 가질 수 있음)
                std::vector<std::string> path_parts;
                std::string current;
                
                for (char c : actual_array_name) {
                    if (c == '/') {
                        if (!current.empty()) {
                            path_parts.push_back(current);
                            current.clear();
                        }
                    } else {
                        current += c;
                    }
                }
                if (!current.empty()) {
                    path_parts.push_back(current);
                }
                
                // 중첩 객체 생성
                nlohmann::json* current_obj = &result[ns];
                for (size_t i = 0; i < path_parts.size() - 1; ++i) {
                    if (!current_obj->contains(path_parts[i])) {
                        (*current_obj)[path_parts[i]] = nlohmann::json::object();
                    }
                    current_obj = &(*current_obj)[path_parts[i]];
                }
                
                if (!path_parts.empty()) {
                    (*current_obj)[path_parts.back()] = array;
                }
            } else {
                // 중첩 경로 처리
                std::vector<std::string> path_parts;
                std::string current;
                
                for (char c : actual_array_name) {
                    if (c == '/') {
                        if (!current.empty()) {
                            path_parts.push_back(current);
                            current.clear();
                        }
                    } else {
                        current += c;
                    }
                }
                if (!current.empty()) {
                    path_parts.push_back(current);
                }
                
                nlohmann::json* current_obj = &result;
                for (size_t i = 0; i < path_parts.size() - 1; ++i) {
                    if (!current_obj->contains(path_parts[i])) {
                        (*current_obj)[path_parts[i]] = nlohmann::json::object();
                    }
                    current_obj = &(*current_obj)[path_parts[i]];
                }
                
                if (!path_parts.empty()) {
                    (*current_obj)[path_parts.back()] = array;
                }
            }
        }
        // 단순 배열인 경우 (position, velocity 등)
        else {
            nlohmann::json array = nlohmann::json::array();
            // 인덱스 순서대로 정렬
            for (const auto& [idx, value] : indices) {
                // idx가 array.size()보다 크면 그 사이에 null을 채움
                while (array.size() < static_cast<size_t>(idx)) {
                    array.push_back(nullptr);
                }
                array.push_back(value);
            }
            
            // 타겟 객체 결정
            if (!ns.empty()) {
                result[ns][actual_array_name] = array;
            } else {
                result[actual_array_name] = array;
            }
        }
    }
    
    return result;
}

std::string MessageFormatter::formatStructured(
    const nlohmann::json& structured_data,
    const FormatOptions& options)
{
    std::stringstream ss;
    
    // 모든 토픽을 동일한 방식으로 처리하는 일반 함수로 구현
    return formatJsonAsYaml(structured_data, 0, options);
}

// 일반적인 JSON -> YAML 스타일 변환기
std::string MessageFormatter::formatJsonAsYaml(
    const nlohmann::json& json_data,
    int depth,
    const FormatOptions& options)
{
    std::stringstream ss;
    
    if (json_data.is_object()) {
        // 객체는 각 키-값 쌍을 순회하며 출력
        bool first = true;
        for (auto it = json_data.begin(); it != json_data.end(); ++it) {
            // 첫 항목이 아니고 depth가 0이면 한 줄 추가
            if (!first && depth == 0) {
                ss << "\n";
            }
            first = false;
            
            const std::string& key = it.key();
            const auto& value = it.value();
            
            // 들여쓰기 적용
            ss << indent(depth * options.indent_level);
            
            // 키 출력
            ss << key << ":";
            
            // 값이 객체나 배열이면 줄바꿈 후 재귀 호출
            if (value.is_object()) {
                ss << "\n" << formatJsonAsYaml(value, depth + 1, options);
            }
            else if (value.is_array()) {
                // 배열의 요소가 객체인 경우 특별 처리 (YAML 리스트 스타일)
                if (!value.empty() && value[0].is_object()) {
                    ss << "\n";
                    for (size_t i = 0; i < value.size(); ++i) {
                        if (value[i].is_null()) continue;
                        ss << indent(depth * options.indent_level) << "- ";
                        ss << formatJsonAsYaml(value[i], depth + 1, options);
                    }
                }
                // 단순 값 배열은 한 줄에 출력
                else {
                    ss << " [";
                    for (size_t i = 0; i < value.size(); ++i) {
                        if (i > 0) ss << ", ";
                        if (value[i].is_number()) {
                            ss << formatNumber(value[i].get<double>());
                        } else if (value[i].is_string()) {
                            ss << value[i].get<std::string>();
                        } else {
                            ss << value[i].dump();
                        }
                    }
                    ss << "]\n";
                }
            }
            // 단일 값은 같은 줄에 출력
            else {
                ss << " ";
                if (value.is_number()) {
                    ss << formatNumber(value.get<double>()) << "\n";
                } else if (value.is_string()) {
                    ss << value.get<std::string>() << "\n";
                } else {
                    ss << value.dump() << "\n";
                }
            }
        }
    }
    else if (json_data.is_array()) {
        // 배열은 줄바꿈 후 각 요소를 출력
        for (size_t i = 0; i < json_data.size(); ++i) {
            ss << indent(depth * options.indent_level) << "- ";
            if (json_data[i].is_object() || json_data[i].is_array()) {
                ss << "\n" << formatJsonAsYaml(json_data[i], depth + 1, options);
            } else if (json_data[i].is_number()) {
                ss << formatNumber(json_data[i].get<double>()) << "\n";
            } else if (json_data[i].is_string()) {
                ss << json_data[i].get<std::string>() << "\n";
            } else {
                ss << json_data[i].dump() << "\n";
            }
        }
    }
    // 기본 값은 그대로 출력
    else {
        if (json_data.is_number()) {
            ss << formatNumber(json_data.get<double>()) << "\n";
        } else if (json_data.is_string()) {
            ss << json_data.get<std::string>() << "\n";
        } else {
            ss << json_data.dump() << "\n";
        }
    }
    
    return ss.str();
}

} // namespace ros2_introspection
