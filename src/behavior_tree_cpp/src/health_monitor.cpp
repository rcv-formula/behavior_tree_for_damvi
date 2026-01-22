#include <unordered_map>
#include <string>
#include <optional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class HealthMonitor : public rclcpp::Node
{
public:
  HealthMonitor() : Node("health_monitor")
  {
    imu_topic_   = declare_parameter<std::string>("IMU_TOPIC", "/imu/data");
    lidar_topic_ = declare_parameter<std::string>("LIDAR_TOPIC", "/scan");
    vesc_topic_  = declare_parameter<std::string>("VESC_TOPIC", "/commands/motor/speed");

    timeout_["imu"]   = declare_parameter<double>("TIMEOUT_IMU", 0.5);
    timeout_["lidar"] = declare_parameter<double>("TIMEOUT_LIDAR", 0.5);
    timeout_["vesc"]  = declare_parameter<double>("TIMEOUT_VESC", 1.0);

    grace_sec_ = declare_parameter<double>("GRACE_SEC", 3.0);
    debounce_sec_ = declare_parameter<double>("DEBOUNCE_SEC", 0.3);

    t0_ = now_s();
    double n = now_s();
    last_["imu"] = n;
    last_["lidar"] = n;
    last_["vesc"] = n;

    create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10, [this](sensor_msgs::msg::Imu::SharedPtr){ mark("imu"); });

    create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic_, 10, [this](sensor_msgs::msg::LaserScan::SharedPtr){ mark("lidar"); });

    create_subscription<std_msgs::msg::Float64>(
      vesc_topic_, 10, [this](std_msgs::msg::Float64::SharedPtr){ mark("vesc"); });

    pub_ok_ = create_publisher<std_msgs::msg::Bool>("/system/critical_ok", 1);
    pub_rs_ = create_publisher<std_msgs::msg::String>("/system/critical_reason", 1);

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this](){ tick(); });
  }

private:
  double now_s() const { return this->now().seconds(); }
  void mark(const std::string& k) { last_[k] = now_s(); }

  bool all_ok(double n)
  {
    for (const auto& kv : timeout_)
    {
      const std::string& k = kv.first;
      double to = kv.second;
      if (n - last_[k] > to)
      {
        reason_ = "timeout:" + k;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason_.c_str());
        return false;
      }
    }
    reason_ = "ok";
    return true;
  }

  void tick()
  {
    double n = now_s();

    if (n - t0_ < grace_sec_)
    {
      std_msgs::msg::Bool ok; ok.data = true;
      std_msgs::msg::String rs; rs.data = "grace";
      pub_ok_->publish(ok);
      pub_rs_->publish(rs);
      return;
    }

    bool ok = all_ok(n);
    if (ok)
    {
      fail_since_.reset();
      std_msgs::msg::Bool b; b.data = true;
      std_msgs::msg::String s; s.data = "ok";
      pub_ok_->publish(b); // 수정 필요
      pub_rs_->publish(s);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "In this tick, sensors are ok. Now go to next tick");
      return;
    }

    if (!fail_since_) fail_since_ = n;
    bool hard_fail = (n - *fail_since_) >= debounce_sec_;

    std_msgs::msg::Bool b; b.data = !hard_fail;
    std_msgs::msg::String s; s.data = hard_fail ? reason_ : "debouncing";
    b.data = true;
    pub_ok_->publish(b); // 수정 필요
    pub_rs_->publish(s);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "In this tick, sensors are down. Now go to next tick");
  }

  std::string imu_topic_, lidar_topic_, vesc_topic_;
  std::unordered_map<std::string, double> timeout_;
  std::unordered_map<std::string, double> last_;

  double t0_{0.0};
  double grace_sec_{3.0};
  double debounce_sec_{0.3};

  std::optional<double> fail_since_;
  std::string reason_{"startup"};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ok_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
