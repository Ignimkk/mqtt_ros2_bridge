#include <ros2_introspection/plugin_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>

namespace ros2_introspection {

PluginLoader::PluginLoader()
  : class_loader_(nullptr) 
{
  // pluginlib 로그 레벨 설정
  RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "Initializing plugin loader");
}

PluginLoader::~PluginLoader() 
{
  plugins_.clear();
  if (class_loader_) {
    try {
      class_loader_.reset();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                  "Error destroying class loader: %s", e.what());
    }
  }
}

bool PluginLoader::loadPlugins() 
{
  try {
    // 현재 실행 파일 경로 출력
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string executable_path = std::string(result, (count > 0) ? count : 0);
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "Current executable path: %s", executable_path.c_str());
    
    // ROS2 패키지 경로 찾기 시도
    const char* ros_distro = std::getenv("ROS_DISTRO");
    const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "ROS_DISTRO: %s", ros_distro ? ros_distro : "unset");
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "AMENT_PREFIX_PATH: %s", ament_prefix_path ? ament_prefix_path : "unset");
    
    // plugins.xml 파일 경로 확인
    std::string package_share_dir = ament_prefix_path ? std::string(ament_prefix_path) : "";
    if (!package_share_dir.empty()) {
      size_t pos = package_share_dir.find(':');
      if (pos != std::string::npos) {
        package_share_dir = package_share_dir.substr(0, pos);
      }
      package_share_dir += "/share/ros2_introspection";
      RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
                 "Looking for plugins.xml in: %s", package_share_dir.c_str());
      
      // 디렉토리 내용 표시
      if (std::filesystem::exists(package_share_dir)) {
        RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "Directory contents:");
        for (const auto& entry : std::filesystem::directory_iterator(package_share_dir)) {
          RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "  %s", entry.path().c_str());
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ros2_introspection"), 
                   "Directory does not exist: %s", package_share_dir.c_str());
      }
    }
    
    // 라이브러리 파일 위치 확인
    std::string lib_path = package_share_dir;
    lib_path = lib_path.substr(0, lib_path.find("/share"));
    lib_path += "/lib";
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "Checking library directory: %s", lib_path.c_str());
    
    if (std::filesystem::exists(lib_path)) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "Library directory contents:");
      for (const auto& entry : std::filesystem::directory_iterator(lib_path)) {
        RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "  %s", entry.path().c_str());
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ros2_introspection"), 
                 "Library directory does not exist: %s", lib_path.c_str());
    }
    
    // 실제 플러그인 로더 생성
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "Creating class loader for MessageHandlerPlugin");

    // 플러그인 XML 파일이 있는지 직접 확인
    std::string xml_path = package_share_dir + "/plugins.xml";
    if (std::filesystem::exists(xml_path)) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
                 "Plugin XML file exists: %s", xml_path.c_str());
      
      // 파일 내용 읽어보기
      std::ifstream xml_file(xml_path);
      if (xml_file.is_open()) {
        std::string line;
        RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "Plugin XML contents:");
        while (std::getline(xml_file, line)) {
          RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), "  %s", line.c_str());
        }
        xml_file.close();
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ros2_introspection"), 
                 "Plugin XML file not found: %s", xml_path.c_str());
    }

    // 로그 레벨 설정 (더 자세한 정보를 위해)
    // RCUTILS_SET_ENV("PLUGINLIB_VERBOSITY", "4");
    setenv("PLUGINLIB_VERBOSITY", "4", 1); // 표준 POSIX 함수로 대체

    class_loader_ = std::make_unique<pluginlib::ClassLoader<MessageHandlerPlugin>>(
      "ros2_introspection", "ros2_introspection::MessageHandlerPlugin");
    
    if (!class_loader_) {
      RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                  "Failed to create ClassLoader");
      return false;
    }
    
    // 사용 가능한 플러그인 목록 표시
    std::vector<std::string> classes = class_loader_->getDeclaredClasses();
    RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
               "Found %zu message handler plugins", classes.size());
    
    for (const auto& class_name : classes) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
                 "Loading plugin: %s", class_name.c_str());
      
      try {
        MessageHandlerPluginPtr plugin = class_loader_->createSharedInstance(class_name);
        if (plugin) {
          plugins_.push_back(plugin);
          RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
                     "Successfully loaded plugin: %s", class_name.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                      "Failed to create plugin instance: %s", class_name.c_str());
        }
      } catch (const pluginlib::PluginlibException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                    "Failed to load plugin %s: %s", class_name.c_str(), e.what());
      }
    }
    
    return !plugins_.empty();
  } catch (const pluginlib::PluginlibException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                "Failed to load plugins: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_introspection"), 
                "Exception in loadPlugins: %s", e.what());
    return false;
  }
}

MessageHandlerPluginPtr PluginLoader::getPluginForMessageType(const std::string& message_type) 
{
  for (auto& plugin : plugins_) {
    if (plugin->supportsMessageType(message_type)) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
                 "Found plugin for message type %s: %s", 
                 message_type.c_str(), plugin->getName().c_str());
      return plugin;
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ros2_introspection"), 
             "No plugin found for message type: %s", message_type.c_str());
  return nullptr;
}

const std::vector<MessageHandlerPluginPtr>& PluginLoader::getLoadedPlugins() const
{
  return plugins_;
}

bool PluginLoader::hasPluginFor(const std::string& message_type) const
{
  // 모든 로드된 플러그인에서 해당 메시지 타입을 지원하는지 확인
  for (const auto& plugin : plugins_) {
    if (plugin->supportsMessageType(message_type)) {
      return true;
    }
  }
  
  return false;
}

} // namespace ros2_introspection 