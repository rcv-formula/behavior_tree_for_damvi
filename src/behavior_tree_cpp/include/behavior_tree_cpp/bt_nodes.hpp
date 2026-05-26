#pragma once

#include <cmath>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <mutex>
#include <optional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

namespace behavior_tree_cpp_pkg
{

// ---- small helpers to keep python-like log strings ----
inline std::string py_str_bool(bool b) { return b ? "True" : "False"; }

inline std::string py_str_double(double v)
{
  // python str() 느낌으로 너무 길어지지 않게
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return std::string(buf);
}

// ---- shared storage (like blackboard + node internal state) ----
struct SharedData
{
  std::mutex mtx;

  // critical_ok (health monitor)
  bool   critical_ok{false};
  double critical_t{0.0};

  // ego odom
  bool   ego_ok{false};
  double ego_t{0.0};
  double ego_x{0.0};
  double ego_y{0.0};
  double ego_yaw{0.0};
  double ego_vx{0.0};
  double ego_vy{0.0};

  // dynamic obstacle odom
  bool   dyn_ok{false};
  double dyn_t{0.0};
  double dyn_x{0.0};
  double dyn_y{0.0};
  double dyn_vx{0.0};
  double dyn_vy{0.0};

  // static obstacle point
  bool   st_ok{false};
  double st_t{0.0};
  double st_x{0.0};
  double st_y{0.0};

  // global path (for static-path distance)
  nav_msgs::msg::Path::SharedPtr global_path{nullptr};

  // local path (for SelectPath)
  nav_msgs::msg::Path::SharedPtr local_path{nullptr};
  std::pair<int32_t, uint32_t> local_stamp{0, 0};

  // latest BT obstacle decision (for SelectPath + RViz status)
  bool obstacle_ready{false};
  bool latest_dynamic_obstacle{false};
  bool latest_static_obstacle{false};
  double latest_dynamic_distance{100.0};
  double latest_static_distance{100.0};
  int latest_obstacle_mode{1};
};

// ---- math utilities ----
double yaw_from_quat(const geometry_msgs::msg::Quaternion& q);

std::optional<double> point_to_path_min_dist(double px, double py,
                                             const nav_msgs::msg::Path::SharedPtr& path_msg);

bool is_in_front_180(double rel_x, double rel_y, double ego_yaw, double half_angle_deg = 100.0);

// -----------------------
// CondCriticalOK (subscribes /system/critical_ok + /system/critical_reason)
// -----------------------
class CondCriticalOK : public BT::SyncActionNode
{
public:
  CondCriticalOK(const std::string& name, const BT::NodeConfiguration& config,
                 const rclcpp::Node::SharedPtr& node,
                 const std::shared_ptr<SharedData>& shared);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  double now() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SharedData> shared_;

  double fresh_{0.5};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ok_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_reason_;
};

// -----------------------
// EmergencyStop
// -----------------------
class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string& name, const BT::NodeConfiguration& config,
                const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool succeed_{false};
};

// -----------------------
// CartographerRestart
// - Watches mapped RF channel values and restarts cartographer launch once per switch edge.
// -----------------------
class CartographerRestartNode : public BT::SyncActionNode
{
public:
  CartographerRestartNode(const std::string& name, const BT::NodeConfiguration& config,
                          const rclcpp::Node::SharedPtr& node);
  ~CartographerRestartNode() override;

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  struct RestartConfig
  {
    double stop_delay_sec{2.0};
    std::string stop_command;
    std::string launch_command;
    std::string launch_log_path;
  };

  double now() const;
  void refresh_params();
  void on_rf_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
  void request_restart(std::uint16_t rf_value);
  void restart_worker(std::uint16_t rf_value, RestartConfig config);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_rf_;

  std::mutex mtx_;
  bool restart_requested_{false};
  bool trigger_latched_{false};
  double last_restart_t_{0.0};
  std::uint16_t requested_rf_value_{0};

  std::atomic_bool restart_in_progress_{false};
  std::thread restart_thread_;

  bool enabled_{false};
  std::string rf_topic_{"/rf"};
  int rf_channel_{9};
  int rf_min_{1501};
  int rf_max_{65535};
  double cooldown_sec_{10.0};
  RestartConfig restart_config_;
};

// -----------------------
// CheckObstacle (your Python CheckObstacle behaviour)
// - Outputs: dynamic_obstacle, static_obstacle, dynamic_distance, static_distance, prioritize_dynamic_flag, obstacle_mode
// -----------------------
class CheckObstacleNode : public BT::SyncActionNode
{
public:
  CheckObstacleNode(const std::string& name, const BT::NodeConfiguration& config,
                    const rclcpp::Node::SharedPtr& node,
                    const std::shared_ptr<SharedData>& shared);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<bool>("dynamic_obstacle"),
	      BT::OutputPort<bool>("static_obstacle"),
	      BT::OutputPort<double>("dynamic_distance"),
	      BT::OutputPort<double>("static_distance"),
	      BT::OutputPort<bool>("prioritize_dynamic_flag"),
	      BT::OutputPort<int>("obstacle_mode"),
	    };
	  }

  BT::NodeStatus tick() override;

private:
  double now() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SharedData> shared_;

	  // params (same semantics as python)
	  double thresh_m_{15.0};
	  double static_thresh_m_{10.0};
	  double estop_thresh_m_{0.5}; // kept for parity, not used directly (python comments)
	  double fresh_{0.15};
	  double half_angle_deg_{100.0};
		  double dynamic_min_speed_mps_{0.009};
	  double static_path_enter_m_{0.6};
	  double static_path_hold_m_{1.0};
	  double dynamic_static_overlap_m_{1.0};

  // state
  bool st_flag_memory_{false};
  nav_msgs::msg::Path::SharedPtr global_path_msg_{nullptr};

	  // ros i/o
	  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publish_flag_;
	  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publish_mode_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ego_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dyn_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_static_;
};

// -----------------------
// SelectPath (your Python SelectPath behaviour)
// - Inputs: dynamic_distance, static_distance
// - Output: mode flag from local planner (0=BASE, 1=AVOID, 2=ACC)
// -----------------------
class SelectPathNode : public BT::SyncActionNode
{
public:
  SelectPathNode(const std::string& name, const BT::NodeConfiguration& config,
                 const rclcpp::Node::SharedPtr& node,
                 const std::shared_ptr<SharedData>& shared);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("dynamic_distance"),
      BT::InputPort<double>("static_distance"),
      BT::OutputPort<double>("overtake_flag"),
    };
  }

  BT::NodeStatus tick() override;

private:
  nav_msgs::msg::Path path_scaler(const nav_msgs::msg::Path& path_msg, double divide);
  void publish_decision_marker(const std::string& frame_id);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<SharedData> shared_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_;

  std::pair<int32_t, uint32_t> local_last_stamp_{0, 0};
  std::optional<nav_msgs::msg::Path> emergency_scaled_path_;
  std::optional<nav_msgs::msg::Path> acc_scaled_path_;
};

}  // namespace behavior_tree_cpp_pkg
