#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include <ros2_introspection/ros2_introspection.hpp>
#include <ros2_introspection/message_formatter.hpp>
#include <ros2_introspection/plugin_loader.hpp>
#include <ros2_introspection/message_handler_plugin.hpp>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <future>
#include <chrono>
#include <algorithm>
#include <functional>
// #include <ros2_introspection/topic_introspection.hpp>  // 이 행 주석 처리
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <pluginlib/class_loader.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <utility> // for std::pair

using namespace std::chrono_literals;

// 네임스페이스 사용 선언 추가
using ros2_introspection::MessageFormatter;
using ros2_introspection::ConsoleMessageFormatter;
using ros2_introspection::JsonMessageFormatter;
using ros2_introspection::CsvMessageFormatter;

class DynamicIntrospectionNode : public rclcpp::Node
{
public:
  DynamicIntrospectionNode(const rclcpp::NodeOptions& options)
  : Node("dynamic_introspection_node", options)
  {
    // 파라미터 가져오기
    auto config_file = this->declare_parameter("config_file", "");
    
    std::vector<std::string> topics;
    std::vector<std::string> types;
    std::string output_format = "console";
    double update_rate = 1.0;
    bool use_plugins = false;
    int max_array_size = 100;
    int timeout_ms = 1000;
    
    // 설정 파일이 지정되었으면 해당 파일에서 설정 로드
    if (!config_file.empty()) {
      RCLCPP_INFO(this->get_logger(), "Loading configuration from file: %s", config_file.c_str());
      try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Parse topics and types from the YAML structure
        if (config["topics"]) {
          for (const auto& topic : config["topics"]) {
            topics.push_back(topic["name"].as<std::string>());
            types.push_back(topic["type"].as<std::string>());
          }
        }
        
        // Load other settings
        if (config["output_format"]) {
          output_format = config["output_format"].as<std::string>();
        }
        
        if (config["update_rate"]) {
          update_rate = config["update_rate"].as<double>();
        }
        
        if (config["use_plugins"]) {
          use_plugins = config["use_plugins"].as<bool>();
        }
        
        if (config["max_array_size"]) {
          max_array_size = config["max_array_size"].as<int>();
        }
        
        if (config["timeout_ms"]) {
          timeout_ms = config["timeout_ms"].as<int>();
        }
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load configuration: %s", e.what());
        throw std::runtime_error("Failed to load configuration file");
      }
    } else {
      // Load from parameters if no config file specified
      topics = this->declare_parameter("topics", std::vector<std::string>{});
      types = this->declare_parameter("types", std::vector<std::string>{});
      output_format = this->declare_parameter("output_format", output_format);
      update_rate = this->declare_parameter("update_rate", update_rate);
      use_plugins = this->declare_parameter("use_plugins", use_plugins);
      max_array_size = this->declare_parameter("max_array_size", max_array_size);
      timeout_ms = this->declare_parameter("timeout_ms", timeout_ms);
    }
    
    // 토픽 개수와 타입 개수 확인
    if (topics.size() != types.size()) {
      std::string error_msg = "Number of topics (" + std::to_string(topics.size()) + 
                             ") does not match number of types (" + std::to_string(types.size()) + ")";
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }
    
    // 파서 초기화
    parser_ = std::make_shared<Ros2Introspection::Parser>();
    if (!parser_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize parser");
      return;
    }
    parser_->setMaxArrayPolicy(Ros2Introspection::Parser::KEEP_LARGE_ARRAYS);
    
    // 포맷터 초기화
    try {
      if (output_format == "json") {
        formatter_ = std::make_shared<JsonMessageFormatter>();
      } else if (output_format == "csv") {
        formatter_ = std::make_shared<CsvMessageFormatter>();
      } else {
        formatter_ = std::make_shared<ConsoleMessageFormatter>();
      }
      
      if (!formatter_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize formatter");
        return;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error initializing formatter: %s", e.what());
      return;
    }
    
    // 플러그인 로더 초기화
    if (use_plugins) {
      try {
        plugin_loader_ = std::make_shared<ros2_introspection::PluginLoader>();
        if (!plugin_loader_) {
          RCLCPP_ERROR(this->get_logger(), "Failed to initialize plugin loader");
          use_plugins = false;
        } else if (!plugin_loader_->loadPlugins()) {
          RCLCPP_WARN(this->get_logger(), "Failed to load message handler plugins, falling back to default handlers");
          use_plugins = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "Loaded message handler plugins");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing plugin loader: %s", e.what());
        use_plugins = false;
      }
    }
    
    // 모든 토픽에 대한 구독자 생성
    try {
      setupSubscriptions(topics, types);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error setting up subscriptions: %s", e.what());
      return;
    }
    
    // 파라미터 업데이트 콜백
    param_sub_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return this->onParamUpdate(parameters);
      });
    
    // 정기적인 상태 출력 타이머
    if (update_rate > 0) {
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/update_rate),
        std::bind(&DynamicIntrospectionNode::publishStatus, this));
    }
    
    // Update parameters
    topics_ = topics;
    types_ = types;
    output_format_ = output_format;
    update_rate_ = update_rate;
    use_plugins_ = use_plugins;
    max_array_size_ = max_array_size;
    timeout_ms_ = timeout_ms;
    
    RCLCPP_INFO(this->get_logger(), "Dynamic introspection node started with %zu topics", topics_.size());
  }

private:
  void setupSubscriptions(const std::vector<std::string>& topics, const std::vector<std::string>& types)
  {
    for (size_t i = 0; i < topics.size(); ++i) {
      try {
        bool handled = false;
        
        // 플러그인 사용이 활성화된 경우, 적절한 플러그인을 찾아 사용
        if (use_plugins_ && plugin_loader_) {
          try {
            auto plugin = plugin_loader_->getPluginForMessageType(types[i]);
            if (plugin) {
              RCLCPP_INFO(this->get_logger(), "Using plugin %s for topic %s [%s]", 
                        plugin->getName().c_str(), topics[i].c_str(), types[i].c_str());
              
              // 플러그인 초기화
              if (plugin->initialize(this, topics[i], types[i])) {
                // 플러그인을 통한 구독 설정
                if (plugin->createSubscription(
                    this, topics[i], types[i],
                    [this, topic = topics[i]](const std::string& msg_topic, 
                                              const Ros2Introspection::RenamedValues& values) {
                      // 최신 메시지 저장
                      try {
                        std::lock_guard<std::mutex> lock(message_mutex_);
                        latest_messages_[msg_topic] = values;
                        message_count_[msg_topic]++;
                      } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Error storing plugin message: %s", e.what());
                      }
                      
                      // 즉시 출력 모드인 경우 바로 출력
                      if (output_format_ != "periodic") {
                        try {
                          formatter_->formatSingleMessage(this->get_logger(), msg_topic, values);
                        } catch (const std::exception& e) {
                          RCLCPP_ERROR(this->get_logger(), "Error formatting plugin message: %s", e.what());
                        }
                      }
                    })) {
                  // 플러그인으로 처리됨
                  plugin_instances_[topics[i]] = plugin;
                  handled = true;
                  continue;
                }
              }
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error setting up plugin for topic %s: %s", 
                         topics[i].c_str(), e.what());
          }
        }
        
        // 플러그인으로 처리되지 않은 경우, 기본 처리 방식 사용
        if (!handled) {
          // 메시지 타입 등록
          try {
            parser_->registerMessageType(topics[i], types[i]);
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to register message type %s: %s", 
                        types[i].c_str(), e.what());
            continue;
          }
          
          // 제네릭 구독자 생성 - 특별히 TF 메시지는 회피 처리
          try {
            // TF 메시지 처리는 안전성을 위해 특별한 방식으로 처리
            if (types[i] == "tf2_msgs/msg/TFMessage" && 
               (topics[i] == "/tf" || topics[i] == "/tf_static")) {
              RCLCPP_INFO(this->get_logger(), "Creating special subscription for TF topic: %s", topics[i].c_str());
              auto subscription = this->create_generic_subscription(
                topics[i], 
                types[i],
                rclcpp::QoS(10).best_effort(),
                [this, topic = topics[i]](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                  this->tfMessageCallbackSafe(topic, msg);
                });
              subscriptions_.push_back(subscription);
            } else {
              // 일반 메시지 처리
              auto subscription = this->create_generic_subscription(
                topics[i], 
                types[i],
                rclcpp::QoS(10).best_effort(),
                [this, topic = topics[i]](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                  this->messageCallback(topic, msg);
                });
              subscriptions_.push_back(subscription);
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create subscription for topic %s: %s", 
                        topics[i].c_str(), e.what());
            continue;
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s [%s]", 
                   topics[i].c_str(), types[i].c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to %s: %s", 
                    topics[i].c_str(), e.what());
      }
    }
  }

  // 안전한 메시지 처리를 위한 타임아웃 함수
  template<typename Func>
  bool runWithTimeout(Func func, int timeoutMs) {
    try {
      std::future<bool> future = std::async(std::launch::async, func);
      
      // 타임아웃 내에 작업이 완료되는지 확인
      auto status = future.wait_for(std::chrono::milliseconds(timeoutMs));
      if (status == std::future_status::timeout) {
        RCLCPP_WARN(this->get_logger(), "Operation timed out after %d ms", timeoutMs);
        return false;
      }
      
      return future.get();  // 실제 반환값 가져오기
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in async operation: %s", e.what());
      return false;
    }
  }
  
  // TF 메시지 처리를 위한 특수 안전 콜백
  void tfMessageCallbackSafe(const std::string& topic, 
                           std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
    if (!serialized_msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null TF message on topic %s", topic.c_str());
      return;
    }
    
    // TF 메시지의 안전 처리를 위한 예외 처리
    try {
      // 플러그인을 사용하여 TF 메시지 처리
      if (use_plugins_ && plugin_loader_) {
        auto plugin = plugin_loader_->getPluginForMessageType("tf2_msgs/msg/TFMessage");
        if (plugin) {
          Ros2Introspection::RenamedValues renamed;
          if (plugin->handleMessage(topic, serialized_msg, renamed)) {
            // 최신 메시지 저장
            {
              std::lock_guard<std::mutex> lock(message_mutex_);
              latest_messages_[topic] = renamed;
              message_count_[topic]++;
            }

            // 즉시 출력 모드인 경우 바로 출력
            if (output_format_ != "periodic") {
              formatter_->formatSingleMessage(this->get_logger(), topic, renamed);
            }
            return;
          }
        }
      }

      // 플러그인 사용 실패 시 기본 처리
      // 메시지 버퍼 안전한 복사
      const auto& rcl_msg = serialized_msg->get_rcl_serialized_message();
      if (!rcl_msg.buffer || rcl_msg.buffer_length == 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid TF message buffer on topic %s", topic.c_str());
        return;
      }
      
      // TF 메시지 처리
      Ros2Introspection::RenamedValues renamed;
      
      // Deserialize TF message
      try {
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
        tf2_msgs::msg::TFMessage tf_msg;
        serialization.deserialize_message(serialized_msg.get(), &tf_msg);
        
        // TF 메시지 내용을 직접 처리
        for (size_t i = 0; i < tf_msg.transforms.size(); ++i) {
          const auto& transform = tf_msg.transforms[i];
          
          // Header
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/header/stamp/sec",
            static_cast<double>(transform.header.stamp.sec)));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/header/stamp/nanosec",
            static_cast<double>(transform.header.stamp.nanosec)));
          
          // frame_id와 child_frame_id 정보
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/header/frame_id",
            -1.0));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/child_frame_id",
            -1.0));
          
          // Transform - translation
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/translation/x",
            transform.transform.translation.x));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/translation/y",
            transform.transform.translation.y));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/translation/z",
            transform.transform.translation.z));
          
          // Transform - rotation
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/rotation/x",
            transform.transform.rotation.x));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/rotation/y",
            transform.transform.rotation.y));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/rotation/z",
            transform.transform.rotation.z));
          renamed.push_back(std::make_pair(
            topic + "/transforms." + std::to_string(i) + "/transform/rotation/w",
            transform.transform.rotation.w));
        }
        
        // Transform 개수 추가
        renamed.push_back(std::make_pair(
          topic + "/transforms_count",
          static_cast<double>(tf_msg.transforms.size())));
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error deserializing TF message: %s", e.what());
        
        // 에러 발생 시 최소한의 정보만 포함
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        renamed.push_back(std::make_pair(topic + "/timestamp", static_cast<double>(clock.now().seconds())));
        renamed.push_back(std::make_pair(topic + "/processed", 1.0));
        renamed.push_back(std::make_pair(topic + "/error", 1.0));
      }
      
      // 최신 메시지 저장
      {
        std::lock_guard<std::mutex> lock(message_mutex_);
        latest_messages_[topic] = renamed;
        message_count_[topic]++;
      }
      
      // 즉시 출력 모드인 경우 바로 출력
      if (output_format_ != "periodic") {
        formatter_->formatSingleMessage(this->get_logger(), topic, renamed);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing TF message on topic %s: %s", 
                  topic.c_str(), e.what());
    }
  }
  
  // 일반 메시지 처리 콜백
  void messageCallback(const std::string& topic, 
                      std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
  {
    if (!serialized_msg) {
      RCLCPP_ERROR(this->get_logger(), "Received null message on topic %s", topic.c_str());
      return;
    }

    try {
      // 메시지 버퍼 복사
      const auto& rcl_msg = serialized_msg->get_rcl_serialized_message();
      if (!rcl_msg.buffer || rcl_msg.buffer_length == 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid message buffer on topic %s", topic.c_str());
        return;
      }
      
      std::vector<uint8_t> msg_buffer(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
      
      // 메시지 배열 설정
      rcutils_uint8_array_t msg_array;
      msg_array.allocator = rcutils_get_default_allocator();
      msg_array.buffer = msg_buffer.data();
      msg_array.buffer_length = msg_buffer.size();
      msg_array.buffer_capacity = msg_buffer.size();
      
      Ros2Introspection::FlatMessage flat_msg;
      bool deserializeSuccess = false;
      
      // 일반 메시지 처리
      try {
        deserializeSuccess = parser_->deserializeIntoFlatMessage(topic, &msg_array, &flat_msg, max_array_size_);
        if (!deserializeSuccess) {
          RCLCPP_WARN(this->get_logger(), "Failed to fully deserialize message on topic %s, some data may be missing", topic.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during message deserialization for topic %s: %s", 
                    topic.c_str(), e.what());
        return;
      }
      
      // 평면화된 메시지를 이름-값 쌍으로 변환
      Ros2Introspection::RenamedValues renamed;
      
      try {
        Ros2Introspection::ConvertFlatMessageToRenamedValues(flat_msg, renamed);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting flat message for topic %s: %s", 
                    topic.c_str(), e.what());
        return;
      }
      
      // renamed가 비어있지 않은 경우에만 처리
      if (!renamed.empty()) {
        // 최신 메시지 저장
        try {
          std::lock_guard<std::mutex> lock(message_mutex_);
          latest_messages_[topic] = renamed;
          message_count_[topic]++;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error storing message: %s", e.what());
          return;
        }
        
        // 즉시 출력 모드인 경우 바로 출력
        if (output_format_ != "periodic") {
          try {
            formatter_->formatSingleMessage(this->get_logger(), topic, renamed);
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error formatting message for topic %s: %s", 
                        topic.c_str(), e.what());
          }
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Empty message content for topic %s", topic.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing message on topic %s: %s", 
                  topic.c_str(), e.what());
    }
  }
  
  void publishStatus()
  {
    try {
      std::lock_guard<std::mutex> lock(message_mutex_);
      if (!latest_messages_.empty()) {
        try {
          formatter_->formatAllMessages(this->get_logger(), latest_messages_, message_count_);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error formatting messages: %s", e.what());
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in publishStatus: %s", e.what());
    }
  }
  
  rcl_interfaces::msg::SetParametersResult 
  onParamUpdate(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    bool need_resubscribe = false;
    
    for (const auto& param : parameters) {
      if (param.get_name() == "topics" || param.get_name() == "types") {
        need_resubscribe = true;
      } else if (param.get_name() == "output_format") {
        std::string output_format = param.as_string();
        try {
          if (output_format == "json") {
            formatter_ = std::make_shared<ros2_introspection::JsonMessageFormatter>();
          } else if (output_format == "csv") {
            formatter_ = std::make_shared<ros2_introspection::CsvMessageFormatter>();
          } else {
            formatter_ = std::make_shared<ros2_introspection::ConsoleMessageFormatter>();
          }
          output_format_ = output_format;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error updating formatter: %s", e.what());
          result.successful = false;
          result.reason = "Error updating formatter: " + std::string(e.what());
          return result;
        }
      } else if (param.get_name() == "update_rate") {
        double update_rate = param.as_double();
        try {
          if (update_rate > 0) {
            timer_ = this->create_wall_timer(
              std::chrono::duration<double>(1.0/update_rate),
              std::bind(&DynamicIntrospectionNode::publishStatus, this));
          } else if (timer_) {
            timer_.reset();
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Error updating timer: %s", e.what());
          result.successful = false;
          result.reason = std::string("Error updating timer: ") + e.what();
          return result;
        }
      } else if (param.get_name() == "max_array_size") {
        max_array_size_ = param.as_int();
      } else if (param.get_name() == "timeout_ms") {
        timeout_ms_ = param.as_int();
      } else if (param.get_name() == "use_plugins") {
        if (use_plugins_ != param.as_bool()) {
          use_plugins_ = param.as_bool();
          need_resubscribe = true;
        }
      }
    }
    
    // 구독 재설정이 필요한 경우
    if (need_resubscribe) {
      try {
        topics_ = get_parameter("topics").as_string_array();
        types_ = get_parameter("types").as_string_array();
        
        if (topics_.size() != types_.size()) {
          RCLCPP_ERROR(this->get_logger(), "Number of topics must match number of types");
          result.successful = false;
          result.reason = "Number of topics must match number of types";
          return result;
        }
        
        // 기존 구독 해제
        subscriptions_.clear();
        plugin_instances_.clear();
        
        // 새 구독 설정
        setupSubscriptions(topics_, types_);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error resubscribing: %s", e.what());
        result.successful = false;
        result.reason = std::string("Error resubscribing: ") + e.what();
        return result;
      }
    }
    
    return result;
  }
  
  // 멤버 변수
  std::shared_ptr<Ros2Introspection::Parser> parser_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::shared_ptr<ros2_introspection::MessageFormatter> formatter_;
  std::vector<std::string> topics_;
  std::vector<std::string> types_;
  std::string output_format_;
  double update_rate_;
  std::mutex message_mutex_;
  std::map<std::string, Ros2Introspection::RenamedValues> latest_messages_;
  std::map<std::string, size_t> message_count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_sub_;
  int max_array_size_;
  int timeout_ms_;
  bool use_plugins_;
  
  // 플러그인 관련 멤버
  std::shared_ptr<ros2_introspection::PluginLoader> plugin_loader_;
  std::map<std::string, ros2_introspection::MessageHandlerPluginPtr> plugin_instances_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicIntrospectionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
