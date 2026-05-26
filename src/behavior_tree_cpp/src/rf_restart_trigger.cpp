#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace std::chrono_literals;

class RfRestartTrigger : public rclcpp::Node
{
public:
  RfRestartTrigger()
  : Node("rf_restart_trigger")
  {
    topic_ = declare_parameter<std::string>("topic", "/rf");
    channel_ = declare_parameter<int>("channel", 9);
    channel_count_ = declare_parameter<int>("channel_count", 14);
    active_value_ = declare_parameter<int>("active_value", 2000);
    inactive_value_ = declare_parameter<int>("inactive_value", 1000);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    active_count_ = declare_parameter<int>("active_count", 20);
    publish_inactive_before_ = declare_parameter<bool>("publish_inactive_before", true);
    publish_inactive_after_ = declare_parameter<bool>("publish_inactive_after", true);

    channel_count_ = std::max(channel_count_, 1);
    channel_ = std::clamp(channel_, 0, channel_count_ - 1);
    active_value_ = std::clamp(active_value_, 0, 65535);
    inactive_value_ = std::clamp(inactive_value_, 0, 65535);
    rate_hz_ = std::max(rate_hz_, 0.1);
    active_count_ = std::max(active_count_, 1);

    pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>(topic_, 10);
  }

  int run()
  {
    RCLCPP_WARN(
      get_logger(),
      "Publishing RF restart trigger: topic=%s channel=%d active=%d inactive=%d count=%d",
      topic_.c_str(), channel_, active_value_, inactive_value_, active_count_);

    rclcpp::sleep_for(500ms);

    if (publish_inactive_before_)
    {
      publish_value(inactive_value_, 5);
    }

    publish_value(active_value_, active_count_);

    if (publish_inactive_after_)
    {
      publish_value(inactive_value_, 5);
    }

    RCLCPP_WARN(get_logger(), "RF restart trigger publish complete.");
    return 0;
  }

private:
  void publish_value(int value, int count)
  {
    rclcpp::WallRate rate(rate_hz_);
    for (int i = 0; rclcpp::ok() && i < count; ++i)
    {
      std_msgs::msg::UInt16MultiArray msg;
      msg.data.assign(static_cast<std::size_t>(channel_count_),
                      static_cast<std::uint16_t>(inactive_value_));
      msg.data[static_cast<std::size_t>(channel_)] = static_cast<std::uint16_t>(value);
      pub_->publish(msg);
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }

  std::string topic_;
  int channel_{9};
  int channel_count_{14};
  int active_value_{2000};
  int inactive_value_{1000};
  double rate_hz_{10.0};
  int active_count_{20};
  bool publish_inactive_before_{true};
  bool publish_inactive_after_{true};

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RfRestartTrigger>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
