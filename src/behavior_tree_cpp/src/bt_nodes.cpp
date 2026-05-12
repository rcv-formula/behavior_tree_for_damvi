#include "behavior_tree_cpp/bt_nodes.hpp"

namespace behavior_tree_cpp_pkg
{

namespace
{

rclcpp::QoS reliable_qos(size_t depth)
{
  rclcpp::QoS qos(depth);
  qos.reliable();
  return qos;
}

rclcpp::QoS transient_reliable_qos(size_t depth)
{
  rclcpp::QoS qos(depth);
  qos.reliable().transient_local();
  return qos;
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
// CheckObstacleNode
// -----------------------
CheckObstacleNode::CheckObstacleNode(const std::string& name, const BT::NodeConfiguration& config,
                                     const rclcpp::Node::SharedPtr& node,
                                     const std::shared_ptr<SharedData>& shared)
: BT::SyncActionNode(name, config), node_(node), shared_(shared)
{
  thresh_m_ = node_->declare_parameter<double>("thresh_m", 15.0);
  estop_thresh_m_ = node_->declare_parameter<double>("estop_thresh_m", 0.5);
  fresh_ = node_->declare_parameter<double>("fresh", 0.3);
  half_angle_deg_ = node_->declare_parameter<double>("half_angle_deg", 100.0);

  publish_flag_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/obj_flag", 1);

  auto global_path_cb =
    [this](nav_msgs::msg::Path::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      shared_->global_path = msg;
      global_path_msg_ = msg;
    };

  sub_global_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/global_path", reliable_qos(1), global_path_cb);

  sub_global_path_transient_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/global_path", transient_reliable_qos(1), global_path_cb);

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
  double dyn_t, dyn_x, dyn_y;

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

    st_ok = shared_->st_ok;
    st_t  = shared_->st_t;
    st_x  = shared_->st_x;
    st_y  = shared_->st_y;

    global_path = shared_->global_path;
  }

  global_path_msg_ = global_path;

  const double now_t = now();

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

    if (in_front && close_enough)
    {
      dyn_flag = true;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "Dynamic VALID: dist=%.2fm, front180=%s",
                    dynamic_dist, in_front ? "True" : "False");
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
    const bool close_enough = (static_dist <= thresh_m_);

    auto path_d_opt = point_to_path_min_dist(st_x, st_y, global_path_msg_);
    double path_d = path_d_opt ? *path_d_opt : -1.0;

    bool close_to_path = false;
    if (path_d_opt)
    {
      if (!st_flag_memory_ && path_d <= 1.0) close_to_path = true;
      else if (st_flag_memory_ && path_d <= 2.0) close_to_path = true;
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
        std::string log = std::string("Static farther than ") + py_str_double(thresh_m_) + "m.";
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
          std::snprintf(buf, sizeof(buf),
                        "Static in front but not blocking path (dist to path=%.2fm >0.8m)",
                        path_d);
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "%s", buf);
        }
      }
    }

    st_flag_memory_ = st_flag;
  }

  // Python: prioritize_dynamic_flag 결국 False로 publish
  bool prioritize_dynamic_flag = false;

  setOutput("dynamic_obstacle", dyn_flag);
  setOutput("static_obstacle", st_flag);
  setOutput("dynamic_distance", dynamic_dist);
  setOutput("static_distance", static_dist);
  setOutput("prioritize_dynamic_flag", prioritize_dynamic_flag);

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->get_clock()->now();  // Humble ok (implicit conversion)
  msg.point.x = dyn_flag ? 1.0 : 0.0;
  msg.point.y = st_flag ? 1.0 : 0.0;
  msg.point.z = prioritize_dynamic_flag ? 1.0 : 0.0;
  publish_flag_->publish(msg);

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

  // Python: create_subscription(Path, 'Path', self.cb_localpath, 1)
  // User: /Path 가 토픽 이름
  auto local_path_cb =
    [this](nav_msgs::msg::Path::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(shared_->mtx);
      shared_->local_path = msg;
      shared_->local_stamp = {msg->header.stamp.sec, msg->header.stamp.nanosec};
    };

  sub_local_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/Path", reliable_qos(1), local_path_cb);

  sub_local_transient_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/Path", transient_reliable_qos(1), local_path_cb);
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
  (void)getInput("dynamic_distance", dynamic_distance);
  (void)getInput("static_distance", static_distance);

  const bool emergency = (dynamic_distance <= 1.0 || static_distance <= 1.0);
  const bool obstacle_nearby = (dynamic_distance < 10.0 || static_distance < 10.0);

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
