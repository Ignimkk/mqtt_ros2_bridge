#include "rclcpp/rclcpp.hpp"
#include <ros2_introspection/ros2_introspection.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

class IntrospectionNode : public rclcpp::Node
{
public:
  IntrospectionNode() : Node("camera_info_introspection")
  {
    parser_ = std::make_shared<Ros2Introspection::Parser>();

    topic_name_ = "/camera/color/camera_info";
    const std::string type_name = "sensor_msgs/msg/CameraInfo";

    try {
      parser_->registerMessageType(topic_name_, type_name);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to register message type: %s", e.what());
      return;
    }

    sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic_name_,
        rclcpp::QoS(10).best_effort(),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
          rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serialization;
          rclcpp::SerializedMessage serialized_msg;
          serialization.serialize_message(msg.get(), &serialized_msg);

          rcutils_uint8_array_t msg_array;
          msg_array.allocator = rcutils_get_default_allocator();
          msg_array.buffer = serialized_msg.get_rcl_serialized_message().buffer;
          msg_array.buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
          msg_array.buffer_capacity = serialized_msg.get_rcl_serialized_message().buffer_capacity;

          Ros2Introspection::FlatMessage flat_msg;
          try {
            parser_->deserializeIntoFlatMessage(topic_name_, &msg_array, &flat_msg, 100);
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message: %s", e.what());
            return;
          }

          Ros2Introspection::RenamedValues renamed;
          Ros2Introspection::ConvertFlatMessageToRenamedValues(flat_msg, renamed);

          std::cout << "\n[Flattened Message from topic: " << topic_name_ << "]\n";
          for (const auto &entry : renamed) {
            std::cout << "  " << entry.first << " = " << entry.second << "\n";
          }
        });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_;
  std::shared_ptr<Ros2Introspection::Parser> parser_;
  std::string topic_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntrospectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
