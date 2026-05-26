#include "behavior_tree_cpp/bt_nodes.hpp"

namespace behavior_tree_cpp_pkg
{

namespace
{

const char* bt_decision_name(bool ready, bool dyn_flag, bool st_flag)
{
  if (!ready) return "WAITING";
  if (dyn_flag && st_flag) return "DYNAMIC + STATIC";
  if (dyn_flag) return "DYNAMIC";
  if (st_flag) return "STATIC";
  return "NO OBSTACLE";
}

void set_marker_color(visualization_msgs::msg::Marker& marker,
                      bool ready,
                      bool dyn_flag,
                      bool st_flag)
{
  marker.color.a = 1.0;

  if (!ready)
  {
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    return;
  }

  if (dyn_flag && st_flag)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    return;
  }

  if (dyn_flag)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;
    return;
  }

  if (st_flag)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    return;
  }

  marker.color.r = 0.05;
  marker.color.g = 0.9;
  marker.color.b = 0.15;
}

std::string shell_quote(const std::string& value)
{
  std::string out = "'";
  for (const char ch : value)
  {
    if (ch == '\'')
    {
      out += "'\\''";
    }
    else
    {
      out += ch;
    }
  }
  out += "'";
  return out;
}

}  // namespace

double yaw_from_quat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  );
}

std::optional<double> point_to_path_min_dist(double px, double py,
                                             const nav_msgs::msg::Path::SharedPtr& path_msg)
{
  if (!path_msg) return std::nullopt;
  double dmin = std::numeric_limits<double>::infinity();
  for (const auto& ps : path_msg->poses)
  {
    const double gx = ps.pose.position.x;
    const double gy = ps.pose.position.y;
    const double d = std::hypot(px - gx, py - gy);
    if (d < dmin) dmin = d;
  }
  return dmin;
}

bool is_in_front_180(double rel_x, double rel_y, double ego_yaw, double half_angle_deg)
{
  const double dist = std::hypot(rel_x, rel_y);
  if (dist == 0.0) return true;

  const double fx = std::cos(ego_yaw);
  const double fy = std::sin(ego_yaw);
  const double ox = rel_x / dist;
  const double oy = rel_y / dist;

  const double dot = fx * ox + fy * oy;
  const double cos_limit = std::cos(half_angle_deg * M_PI / 180.0);
  return dot >= cos_limit;
}

// -----------------------
// CondCriticalOK
// -----------------------
CondCriticalOK::CondCriticalOK(const std::string& name, const BT::NodeConfiguration& config,
                               const rclcpp::Node::SharedPtr& node,
                               const std::shared_ptr<SharedData>& shared)
: BT::SyncActionNode(name, config), node_(node), shared_(shared)
{
  node_->declare_parameter<bool>("bypass_critical", false); // 테스트용으로 센서 상태 ok라고 보고 진행하기.

  fresh_ = node_->declare_parameter<double>("critical_fresh", 0.5);

  sub_ok_ = node_->create_subscription<std_msgs::msg::Bool>(
    "system/critical_ok", 1,
    [this](std_msgs::msg::Bool::SharedPtr msg)
    {
      {
        std::lock_guard<std::mutex> lk(shared_->mtx);
        shared_->critical_ok = bool(msg->data);
        shared_->critical_t = now();
      }
      std::string s = std::string("I got the message, critical_ok is ") + py_str_bool(bool(msg->data));
      RCLCPP_WARN(node_->get_logger(), "%s", s.c_str());
    });

  sub_reason_ = node_->create_subscription<std_msgs::msg::String>(
    "system/critical_reason", 1,
    [this](std_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_WARN(node_->get_logger(), "%s", msg->data.c_str());
    });
}

double CondCriticalOK::now() const
{
  return node_->now().seconds();
}

BT::NodeStatus CondCriticalOK::tick()
{
  bool bypass = false;
  node_->get_parameter("bypass_critical", bypass);

  if (bypass)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "CondCriticalOK bypassed (test mode)");
    return BT::NodeStatus::SUCCESS;
  }

  const double now_t = now();
  bool ok;
  double t;
  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    ok = shared_->critical_ok;
    t  = shared_->critical_t;
  }

  if (t == 0.0 || (now_t - t) > fresh_)
  {
    RCLCPP_WARN(node_->get_logger(), "/system/critical_ok can't subscribable. CondCriticalOK Failure.");
    return BT::NodeStatus::FAILURE;
  }

  if (ok)
  {
    RCLCPP_INFO(node_->get_logger(), "IMU,VESC,LiDAR working.");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

// -----------------------
// EmergencyStop
// -----------------------
EmergencyStop::EmergencyStop(const std::string& name, const BT::NodeConfiguration& config,
                             const rclcpp::Node::SharedPtr& node)
: BT::SyncActionNode(name, config), node_(node)
{
  succeed_ = node_->declare_parameter<bool>("estop_succeed", false);
}

BT::NodeStatus EmergencyStop::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Estop SUCCESS");
  return succeed_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// -----------------------
// CartographerRestartNode
// -----------------------
CartographerRestartNode::CartographerRestartNode(const std::string& name,
                                                 const BT::NodeConfiguration& config,
                                                 const rclcpp::Node::SharedPtr& node)
: BT::SyncActionNode(name, config), node_(node)
{
  enabled_ = node_->declare_parameter<bool>("cartographer_restart_enabled", false);
  rf_topic_ = node_->declare_parameter<std::string>("cartographer_restart_rf_topic", "/rf");
  rf_channel_ = node_->declare_parameter<int>("cartographer_restart_rf_channel", 9);
  rf_min_ = node_->declare_parameter<int>("cartographer_restart_rf_min", 1501);
  rf_max_ = node_->declare_parameter<int>("cartographer_restart_rf_max", 65535);
  cooldown_sec_ = node_->declare_parameter<double>("cartographer_restart_cooldown_sec", 10.0);
  restart_config_.stop_delay_sec =
    node_->declare_parameter<double>("cartographer_restart_stop_delay_sec", 2.0);
  restart_config_.stop_command = node_->declare_parameter<std::string>(
    "cartographer_stop_command",
    "pkill -SIGINT -f 'ros2 launch cartographer_ros Damvi_carto_pure_wheel_launch.py' || true");
  restart_config_.launch_command = node_->declare_parameter<std::string>(
    "cartographer_launch_command",
    "cd /home/symoon/Desktop/F1/Local_SLAM_Complete/good/SLAM_main-local_loss_wheel && "
    "source install/setup.bash && "
    "ros2 launch cartographer_ros Damvi_carto_pure_wheel_launch.py");
  restart_config_.launch_log_path =
    node_->declare_parameter<std::string>("cartographer_launch_log_path",
                                          "/tmp/cartographer_restart.log");

  sub_rf_ = node_->create_subscription<std_msgs::msg::UInt16MultiArray>(
    rf_topic_, 10,
    [this](std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
      on_rf_msg(msg);
    });

  RCLCPP_INFO(node_->get_logger(),
              "CartographerRestart watching RF topic '%s' (enabled=%s, channel=%d, range=[%d,%d])",
              rf_topic_.c_str(), enabled_ ? "true" : "false", rf_channel_, rf_min_, rf_max_);
}

CartographerRestartNode::~CartographerRestartNode()
{
  if (restart_thread_.joinable())
  {
    restart_thread_.join();
  }
}

double CartographerRestartNode::now() const
{
  return node_->now().seconds();
}

void CartographerRestartNode::refresh_params()
{
  node_->get_parameter("cartographer_restart_enabled", enabled_);
  node_->get_parameter("cartographer_restart_rf_channel", rf_channel_);
  node_->get_parameter("cartographer_restart_rf_min", rf_min_);
  node_->get_parameter("cartographer_restart_rf_max", rf_max_);
  node_->get_parameter("cartographer_restart_cooldown_sec", cooldown_sec_);
  node_->get_parameter("cartographer_restart_stop_delay_sec", restart_config_.stop_delay_sec);
  node_->get_parameter("cartographer_stop_command", restart_config_.stop_command);
  node_->get_parameter("cartographer_launch_command", restart_config_.launch_command);
  node_->get_parameter("cartographer_launch_log_path", restart_config_.launch_log_path);

  if (rf_min_ > rf_max_)
  {
    std::swap(rf_min_, rf_max_);
  }
  if (rf_min_ < 0) rf_min_ = 0;
  if (rf_max_ < 0) rf_max_ = 0;
  if (rf_min_ > 65535) rf_min_ = 65535;
  if (rf_max_ > 65535) rf_max_ = 65535;
}

void CartographerRestartNode::on_rf_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
  refresh_params();

  if (!enabled_)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    trigger_latched_ = false;
    return;
  }

  if (rf_channel_ < 0)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "cartographer_restart_enabled is true, but cartographer_restart_rf_channel is not set.");
    return;
  }

  const auto channel = static_cast<std::size_t>(rf_channel_);
  if (channel >= msg->data.size())
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "RF restart channel %d is out of range. /rf has %zu channels.",
      rf_channel_, msg->data.size());
    return;
  }

  const std::uint16_t value = msg->data[channel];
  const bool trigger_active = value >= static_cast<std::uint16_t>(rf_min_) &&
                              value <= static_cast<std::uint16_t>(rf_max_);

  bool should_request = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (trigger_active && !trigger_latched_)
    {
      trigger_latched_ = true;
      should_request = true;
    }
    else if (!trigger_active)
    {
      trigger_latched_ = false;
    }
  }

  if (should_request)
  {
    request_restart(value);
  }
}

void CartographerRestartNode::request_restart(std::uint16_t rf_value)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (restart_in_progress_.load())
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Cartographer restart request ignored because a restart is already running.");
    return;
  }

  const double now_t = now();
  if (last_restart_t_ != 0.0 && (now_t - last_restart_t_) < cooldown_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Cartographer restart request ignored by cooldown: %.2fs remaining.",
      cooldown_sec_ - (now_t - last_restart_t_));
    return;
  }

  restart_requested_ = true;
  requested_rf_value_ = rf_value;
  last_restart_t_ = now_t;
  RCLCPP_WARN(node_->get_logger(),
              "Cartographer restart requested by RF channel %d value %u.",
              rf_channel_, static_cast<unsigned>(rf_value));
}

BT::NodeStatus CartographerRestartNode::tick()
{
  refresh_params();

  if (restart_thread_.joinable() && !restart_in_progress_.load())
  {
    restart_thread_.join();
  }

  bool start_restart = false;
  std::uint16_t rf_value = 0;
  RestartConfig config;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (restart_requested_ && !restart_in_progress_.load())
    {
      restart_requested_ = false;
      restart_in_progress_.store(true);
      rf_value = requested_rf_value_;
      config = restart_config_;
      start_restart = true;
    }
  }

  if (start_restart)
  {
    restart_thread_ = std::thread(
      [this, rf_value, config]()
      {
        restart_worker(rf_value, config);
      });
  }

  return BT::NodeStatus::SUCCESS;
}

void CartographerRestartNode::restart_worker(std::uint16_t rf_value, RestartConfig config)
{
  RCLCPP_WARN(node_->get_logger(),
              "Restarting cartographer launch now (RF value=%u).",
              static_cast<unsigned>(rf_value));

  if (!config.stop_command.empty())
  {
    const int stop_rc = std::system(config.stop_command.c_str());
    RCLCPP_INFO(node_->get_logger(), "cartographer_stop_command finished with rc=%d.", stop_rc);
  }

  if (config.stop_delay_sec > 0.0)
  {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(config.stop_delay_sec * 1000.0)));
  }

  if (config.launch_command.empty())
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "cartographer_launch_command is empty. Cartographer was stopped but not relaunched.");
    restart_in_progress_.store(false);
    return;
  }

  const std::string launch_inner =
    "(" + config.launch_command + ") >> " + shell_quote(config.launch_log_path) + " 2>&1 &";
  const std::string launch_shell = "bash -lc " + shell_quote(launch_inner);
  const int launch_rc = std::system(launch_shell.c_str());

  if (launch_rc == 0)
  {
    RCLCPP_WARN(node_->get_logger(),
                "cartographer launch restarted. Log: %s",
                config.launch_log_path.c_str());
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "cartographer_launch_command failed to start, rc=%d. Log: %s",
                 launch_rc, config.launch_log_path.c_str());
  }

  restart_in_progress_.store(false);
}

// -----------------------
// CheckObstacleNode
// -----------------------
CheckObstacleNode::CheckObstacleNode(const std::string& name, const BT::NodeConfiguration& config,
                                     const rclcpp::Node::SharedPtr& node,
                                     const std::shared_ptr<SharedData>& shared)
: BT::SyncActionNode(name, config), node_(node), shared_(shared)
{
	  thresh_m_ = node_->declare_parameter<double>("thresh_m", 15.0);
	  static_thresh_m_ = node_->declare_parameter<double>("static_thresh_m", 10.0);
	  estop_thresh_m_ = node_->declare_parameter<double>("estop_thresh_m", 0.5);
	  fresh_ = node_->declare_parameter<double>("fresh", 0.15);
	  half_angle_deg_ = node_->declare_parameter<double>("half_angle_deg", 100.0);
	  dynamic_min_speed_mps_ = node_->declare_parameter<double>("dynamic_min_speed_mps", 0.0);
	  static_path_enter_m_ = node_->declare_parameter<double>("static_path_enter_m", 0.6);
	  static_path_hold_m_ = node_->declare_parameter<double>("static_path_hold_m", 1.0);
	  dynamic_static_overlap_m_ = node_->declare_parameter<double>("dynamic_static_overlap_m", 1.0);
	  node_->declare_parameter<int>("obstacle_mode", 0);

	  publish_flag_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/obj_flag", 1);
	  publish_mode_ = node_->create_publisher<std_msgs::msg::Int32>("/obstacle_mode", 1);

  rclcpp::QoS path_qos(1);
  path_qos.reliable();

  sub_global_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/global_path", path_qos,
    [this](nav_msgs::msg::Path::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      shared_->global_path = msg;
      global_path_msg_ = msg;
    });

  sub_ego_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      const auto& p = msg->pose.pose.position;
      const auto& q = msg->pose.pose.orientation;
      const auto& v = msg->twist.twist.linear;

      shared_->ego_x = p.x;
      shared_->ego_y = p.y;
      shared_->ego_yaw = yaw_from_quat(q);
      shared_->ego_vx = v.x;
      shared_->ego_vy = v.y;
      shared_->ego_t  = now();
      shared_->ego_ok = true;
    });

  sub_dyn_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/dynamic_obstacle", 20,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      const auto& p = msg->pose.pose.position;
      const auto& v = msg->twist.twist.linear;

      shared_->dyn_x = p.x;
      shared_->dyn_y = p.y;
      shared_->dyn_vx = v.x;
      shared_->dyn_vy = v.y;
      shared_->dyn_t  = now();
      shared_->dyn_ok = true;
    });

  sub_static_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    "/static_obstacle", 10,
    [this](geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      shared_->st_x = msg->point.x;
      shared_->st_y = msg->point.y;
      shared_->st_t = now();
      shared_->st_ok = true;
    });
}

double CheckObstacleNode::now() const
{
  return node_->now().seconds();
}

BT::NodeStatus CheckObstacleNode::tick()
{
  // ---- local snapshot (no copying SharedData because it has mutex) ----
  bool ego_ok;
  double ego_t, ego_x, ego_y, ego_yaw;

  bool dyn_ok;
  double dyn_t, dyn_x, dyn_y, dyn_vx, dyn_vy;

  bool st_ok;
  double st_t, st_x, st_y;

  nav_msgs::msg::Path::SharedPtr global_path;

  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    ego_ok  = shared_->ego_ok;
    ego_t   = shared_->ego_t;
    ego_x   = shared_->ego_x;
    ego_y   = shared_->ego_y;
    ego_yaw = shared_->ego_yaw;

    dyn_ok = shared_->dyn_ok;
    dyn_t  = shared_->dyn_t;
    dyn_x  = shared_->dyn_x;
    dyn_y  = shared_->dyn_y;
    dyn_vx = shared_->dyn_vx;
    dyn_vy = shared_->dyn_vy;

    st_ok = shared_->st_ok;
    st_t  = shared_->st_t;
    st_x  = shared_->st_x;
    st_y  = shared_->st_y;

    global_path = shared_->global_path;
  }

  global_path_msg_ = global_path;

  const double now_t = now();

	  node_->get_parameter("thresh_m", thresh_m_);
	  node_->get_parameter("static_thresh_m", static_thresh_m_);
	  node_->get_parameter("fresh", fresh_);
	  node_->get_parameter("half_angle_deg", half_angle_deg_);
	  node_->get_parameter("dynamic_min_speed_mps", dynamic_min_speed_mps_);
	  node_->get_parameter("static_path_enter_m", static_path_enter_m_);
	  node_->get_parameter("static_path_hold_m", static_path_hold_m_);
	  node_->get_parameter("dynamic_static_overlap_m", dynamic_static_overlap_m_);

	  int selected_mode = 0;
	  node_->get_parameter("obstacle_mode", selected_mode);
	  if (selected_mode < 0 || selected_mode > 3)
	  {
	    RCLCPP_WARN_THROTTLE(
	      node_->get_logger(), *node_->get_clock(), 2000,
	      "Invalid obstacle_mode=%d. Use 0(auto), 1(no obstacle), 2(dynamic only), 3(dynamic+static). Falling back to auto.",
	      selected_mode);
	    selected_mode = 0;
	  }

  if (!ego_ok || ego_t == 0.0 || (now_t - ego_t) > fresh_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "/odom can't subscribable. CheckObstacle Failure.");
    return BT::NodeStatus::FAILURE;
  }

  bool dyn_flag = false;
  bool st_flag  = false;
  double dynamic_dist = 100.0;
  double static_dist  = 100.0;

  // ------------------------
  // Dynamic obstacle 판정
  // ------------------------
  const bool dyn_never = (!dyn_ok || dyn_t == 0.0);
  const bool dyn_stale = (dyn_ok && dyn_t != 0.0 && (now_t - dyn_t) > fresh_);
  if (dyn_never)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "No dynamic_obstacle detected yet.");
    dyn_flag = false;
    dynamic_dist = 100.0;
  }
  else if (dyn_stale){
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "/dynamic_obstacle stale: age=%.3fs (fresh = %.3fs).", (now_t-dyn_t),fresh_);
    dyn_flag = false;
    dynamic_dist = 100.0;
  }
  else
  {
    const double dx = dyn_x - ego_x;
    const double dy = dyn_y - ego_y;
    dynamic_dist = std::hypot(dx, dy);

    {
      std::string log = std::string("Dynamic obj :: ") + py_str_double(dynamic_dist) + " meters.";
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log.c_str());
    }

    const bool in_front = is_in_front_180(dx, dy, ego_yaw, half_angle_deg_);
    const bool close_enough = (dynamic_dist <= thresh_m_);
    const double dynamic_speed = std::hypot(dyn_vx, dyn_vy);
    const bool use_mode3_speed_gate = (selected_mode == 3 && dynamic_min_speed_mps_ > 0.0);
    const bool moving_enough = (!use_mode3_speed_gate || dynamic_speed >= dynamic_min_speed_mps_);

    if (in_front && close_enough && moving_enough)
    {
      dyn_flag = true;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "Dynamic VALID: dist=%.2fm, front180=%s, speed=%.2fm/s",
                    dynamic_dist, in_front ? "True" : "False", dynamic_speed);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "%s", buf);
    }
    else
    {
      if (!in_front)
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Dynamic detected but NOT in front 180deg.");
      }
      else if (!close_enough)
      {
        std::string log = std::string("Dynamic farther than ") + py_str_double(thresh_m_) + "m.";
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "%s", log.c_str());
      }
      else if (!moving_enough)
      {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Mode 3 dynamic suppressed by speed gate: speed=%.2fm/s < %.2fm/s.",
          dynamic_speed, dynamic_min_speed_mps_);
      }
    }
  }

  // ------------------------
  // Static obstacle 판정
  // ------------------------
  const bool st_never = (!st_ok || st_t == 0.0);
  const bool st_stale = (st_ok && st_t != 0.0 && (now_t - st_t) > fresh_);
  if (st_never){
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "No /static_obstacle detected yet.");
    st_flag = false;
    static_dist = 100.0;
  }
  else if (st_stale)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "/static_obstacle stale: age=%.3fs (fresh=%.3fs).", (now_t-st_t), fresh_);
    st_flag = false;
    static_dist = 100.0;
  }
  else
  {
    const double sx = st_x - ego_x;
    const double sy = st_y - ego_y;
    static_dist = std::hypot(sx, sy);

    {
      std::string log = std::string("Static obj :: ") + py_str_double(static_dist) + " meters.";
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", log.c_str());
    }

    const bool in_front = is_in_front_180(sx, sy, ego_yaw, half_angle_deg_);
	    const bool close_enough = (static_dist <= static_thresh_m_);

    auto path_d_opt = point_to_path_min_dist(st_x, st_y, global_path_msg_);
    double path_d = path_d_opt ? *path_d_opt : -1.0;

    bool close_to_path = false;
    if (path_d_opt)
    {
	      if (!st_flag_memory_ && path_d <= static_path_enter_m_) close_to_path = true;
	      else if (st_flag_memory_ && path_d <= static_path_hold_m_) close_to_path = true;
      else close_to_path = false;
    }
    else
    {
      close_to_path = false;
    }

    if (in_front && close_enough && close_to_path)
    {
      st_flag = true;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "Static VALID: dist=%.2fm, path_d=%.2fm",
                    static_dist, path_d_opt ? path_d : 0.0);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "%s", buf);
    }
    else
    {
      if (!in_front)
      {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Static detected but NOT in front 180deg.");
      }
      else if (!close_enough)
      {
	        std::string log = std::string("Static farther than ") + py_str_double(static_thresh_m_) + "m.";
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "%s", log.c_str());
      }
      else if (!close_to_path)
      {
        if (!path_d_opt)
        {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Static in front but local path not ready -> ignore static.");
        }
        else
        {
          char buf[256];
	          const double active_static_path_limit =
	            st_flag_memory_ ? static_path_hold_m_ : static_path_enter_m_;
	          std::snprintf(buf, sizeof(buf),
	                        "Static in front but not blocking path (dist to path=%.2fm >%.2fm)",
	                        path_d, active_static_path_limit);
	          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "%s", buf);
	        }
	      }
    }

    st_flag_memory_ = st_flag;
  }

	  const bool suppress_static_overlap = (selected_mode == 0 || selected_mode == 2);
	  if (suppress_static_overlap && dyn_flag && st_flag)
	  {
	    const double overlap_dist = std::hypot(dyn_x - st_x, dyn_y - st_y);
	    if (overlap_dist <= dynamic_static_overlap_m_)
	    {
	      st_flag = false;
	      static_dist = 100.0;
	      st_flag_memory_ = false;
	      RCLCPP_INFO_THROTTLE(
	        node_->get_logger(), *node_->get_clock(), 500,
	        "Static suppressed because it overlaps dynamic obstacle: overlap=%.2fm <= %.2fm",
	        overlap_dist, dynamic_static_overlap_m_);
	    }
	  }

	  if (selected_mode == 1)
	  {
	    dyn_flag = false;
	    st_flag = false;
	    dynamic_dist = 100.0;
	    static_dist = 100.0;
	    st_flag_memory_ = false;
	  }
	  else if (selected_mode == 2)
	  {
	    st_flag = false;
	    static_dist = 100.0;
	    st_flag_memory_ = false;
	  }

	  // obstacle_mode parameter:
	  // 0=auto, 1=no obstacle, 2=dynamic only, 3=dynamic+static.
	  // In auto mode, publish the mode inferred from the final flags.
	  int detected_mode = 1;
	  if (dyn_flag && !st_flag) detected_mode = 2;
	  else if (st_flag) detected_mode = 3;
	  const int obstacle_mode = (selected_mode == 0) ? detected_mode : selected_mode;

	  bool prioritize_dynamic_flag = false;

	  setOutput("dynamic_obstacle", dyn_flag);
	  setOutput("static_obstacle", st_flag);
	  setOutput("dynamic_distance", dynamic_dist);
	  setOutput("static_distance", static_dist);
	  setOutput("prioritize_dynamic_flag", prioritize_dynamic_flag);
	  setOutput("obstacle_mode", obstacle_mode);

  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    shared_->obstacle_ready = true;
    shared_->latest_dynamic_obstacle = dyn_flag;
    shared_->latest_static_obstacle = st_flag;
    shared_->latest_dynamic_distance = dynamic_dist;
    shared_->latest_static_distance = static_dist;
    shared_->latest_obstacle_mode = obstacle_mode;
  }

  geometry_msgs::msg::PointStamped msg;
	  msg.header.stamp = node_->get_clock()->now();  // Humble ok (implicit conversion)
	  msg.point.x = dyn_flag ? 1.0 : 0.0;
	  msg.point.y = st_flag ? 1.0 : 0.0;
	  msg.point.z = prioritize_dynamic_flag ? 1.0 : 0.0;
	  publish_flag_->publish(msg);

	  std_msgs::msg::Int32 mode_msg;
	  mode_msg.data = obstacle_mode;
	  publish_mode_->publish(mode_msg);

  // Python: 항상 FAILURE로 다음 노드(SelectPath)로 넘어가게 함
  return BT::NodeStatus::FAILURE;
}

// -----------------------
// SelectPathNode
// -----------------------
SelectPathNode::SelectPathNode(const std::string& name, const BT::NodeConfiguration& config,
                               const rclcpp::Node::SharedPtr& node,
                               const std::shared_ptr<SharedData>& shared)
: BT::SyncActionNode(name, config), node_(node), shared_(shared)
{
  pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("/selected_path", 1);
  pub_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("/bt_decision_marker", 1);

  // Python: create_subscription(Path, 'Path', self.cb_localpath, 1)
  // User: /Path 가 토픽 이름
  rclcpp::QoS qos(1);
  qos.reliable(); // local_path가 qos가 이제는 transient local이 아니라 그냥 reliable로갓네요.

  sub_local_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/Path", qos,
    [this](nav_msgs::msg::Path::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      shared_->local_path = msg;
      shared_->local_stamp = {msg->header.stamp.sec, msg->header.stamp.nanosec};
    });
}

nav_msgs::msg::Path SelectPathNode::path_scaler(const nav_msgs::msg::Path& path_msg, double divide)
{
  nav_msgs::msg::Path sp = path_msg;
  for (auto& ps : sp.poses)
  {
    ps.pose.position.z = ps.pose.position.z / divide;
  }
  return sp;
}

void SelectPathNode::publish_decision_marker(const std::string& frame_id)
{
  bool obstacle_ready = false;
  bool dyn_flag = false;
  bool st_flag = false;
  bool ego_ok = false;
  double ego_x = 0.0;
  double ego_y = 0.0;

  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    obstacle_ready = shared_->obstacle_ready;
    dyn_flag = shared_->latest_dynamic_obstacle;
    st_flag = shared_->latest_static_obstacle;
    ego_ok = shared_->ego_ok;
    ego_x = shared_->ego_x;
    ego_y = shared_->ego_y;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->get_clock()->now();
  marker.header.frame_id = frame_id.empty() ? "map" : frame_id;
  marker.ns = "behavior_tree";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = ego_ok ? ego_x : 0.0;
  marker.pose.position.y = ego_ok ? ego_y + 0.9 : 0.0;
  marker.pose.position.z = 1.2;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.45;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  set_marker_color(marker, obstacle_ready, dyn_flag, st_flag);
  marker.text = bt_decision_name(obstacle_ready, dyn_flag, st_flag);

  pub_marker_->publish(marker);
}

BT::NodeStatus SelectPathNode::tick()
{
  nav_msgs::msg::Path::SharedPtr lp;
  std::pair<int32_t, uint32_t> stamp;
  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    lp = shared_->local_path;
    stamp = shared_->local_stamp;
  }

  if (!lp)
  {
    publish_decision_marker("map");
    RCLCPP_WARN(node_->get_logger(), "/Path don't come yet.");
    return BT::NodeStatus::FAILURE;
  }

  constexpr double MODE_BASE = 0.0;
  constexpr double MODE_AVOID = 1.0;
  constexpr double MODE_ACC = 2.0;

  double overtake_flag = MODE_BASE;
  if (!lp->poses.empty())
  {
    overtake_flag = lp->poses[0].pose.orientation.z;
  }
  setOutput("overtake_flag", overtake_flag);

  if (stamp != local_last_stamp_)
  {
    local_last_stamp_ = stamp;
    emergency_scaled_path_ = path_scaler(*lp, 10.0);
  }

  double dynamic_distance = 100.0;
  double static_distance  = 100.0;
  {
    std::lock_guard<std::mutex> lk(shared_->mtx);
    dynamic_distance = shared_->latest_dynamic_distance;
    static_distance = shared_->latest_static_distance;
  }
  (void)getInput("dynamic_distance", dynamic_distance);
  (void)getInput("static_distance", static_distance);

  const bool emergency = (dynamic_distance <= 1.0 || static_distance <= 1.0);
  const bool obstacle_nearby = (dynamic_distance < 10.0 || static_distance < 10.0);
  publish_decision_marker(lp->header.frame_id);

  if (emergency)
  {
    if (emergency_scaled_path_)
    {
      pub_path_->publish(*emergency_scaled_path_);
      RCLCPP_INFO(
        node_->get_logger(),
        "Emergency path published! mode=%.0f dyn=%.2f static=%.2f",
        overtake_flag, dynamic_distance, static_distance);
      return BT::NodeStatus::SUCCESS;
    }

    pub_path_->publish(*lp);
    RCLCPP_INFO(node_->get_logger(), "Emergency인데 scaled path가 아직 없어 local path를 publish함");
    return BT::NodeStatus::SUCCESS;
  }

  pub_path_->publish(*lp);

  if (overtake_flag == MODE_ACC)
  {
    if (obstacle_nearby)
    {
      RCLCPP_INFO(node_->get_logger(), "ACC mode, obstacle nearby.");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "ACC mode, obstacle track cleared.");
    }
  }
  else if (overtake_flag == MODE_AVOID)
  {
    if (obstacle_nearby)
    {
      RCLCPP_INFO(node_->get_logger(), "Avoid mode, obstacle nearby.");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Avoid mode, obstacle track cleared.");
    }
  }
  else if (overtake_flag == MODE_BASE)
  {
    if (obstacle_nearby)
    {
      RCLCPP_INFO(node_->get_logger(), "Base mode, obstacle nearby.");
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Base mode, no obstacle nearby.");
    }
  }
  else
  {
    std::string log = std::string("unknown overtake_flag!!! ") + py_str_double(overtake_flag);
    if (obstacle_nearby)
    {
      log += " (obstacle nearby)";
    }
    else
    {
      log += " (no obstacle nearby)";
    }
    RCLCPP_WARN(node_->get_logger(), "%s", log.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree_cpp_pkg
