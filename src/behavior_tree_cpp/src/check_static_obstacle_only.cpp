#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

static double yaw_from_quat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  );
}

static std::optional<double> point_to_path_min_dist(
  double px, double py, const nav_msgs::msg::Path::SharedPtr& path_msg)
{
  if (!path_msg) return std::nullopt;
  if (path_msg->poses.empty()) return std::nullopt;

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

static bool is_in_front_180(double rel_x, double rel_y, double ego_yaw, double half_angle_deg)
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

class CheckStaticObstacleOnly : public rclcpp::Node
{
public:
  CheckStaticObstacleOnly()
  : Node("check_static_obstacle_only")
  {
    thresh_m_ = declare_parameter<double>("thresh_m", 15.0);
    fresh_ = declare_parameter<double>("fresh", 0.3);
    half_angle_deg_ = declare_parameter<double>("half_angle_deg", 100.0);
    hz_ = declare_parameter<double>("hz", 40.0);

    pub_flag_ = create_publisher<geometry_msgs::msg::PointStamped>("/obj_flag", 1);

    // global_path QoS: reliable + transient_local (당신이 쓰던 의도 그대로)
    rclcpp::QoS path_qos(1);
    path_qos.reliable().transient_local();

    sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
      "/global_path", path_qos,
      [this](nav_msgs::msg::Path::SharedPtr msg)
      {
        global_path_ = msg;
      });

    sub_ego_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg)
      {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        ego_x_ = p.x;
        ego_y_ = p.y;
        ego_yaw_ = yaw_from_quat(q);
        ego_t_ = now_s();
        ego_ok_ = true;
      });

    sub_static_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/static_obstacle", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg)
      {
        st_x_ = msg->point.x;
        st_y_ = msg->point.y;
        st_t_ = now_s();
        st_ok_ = true;
      });

    const double period = 1.0 / std::max(1e-6, hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&CheckStaticObstacleOnly::tick, this));

    RCLCPP_INFO(get_logger(), "CheckStaticObstacleOnly started.");
  }

private:
  double now_s() const
  {
    return this->now().seconds();
  }

  void tick()
  {
    const double now_t = now_s();

    // ego freshness
    if (!ego_ok_ || ego_t_ == 0.0 || (now_t - ego_t_) > fresh_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/odom stale -> skip");
      return;
    }

    bool st_flag = false;
    double static_dist = 100.0;

    // static freshness
    const bool st_stale = (!st_ok_ || st_t_ == 0.0 || (now_t - st_t_) > fresh_);
    if (st_stale)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/static_obstacle stale -> no static");
      st_flag_memory_ = false;
      st_flag = false;
      static_dist = 100.0;
    }
    else
    {
      const double sx = st_x_ - ego_x_;
      const double sy = st_y_ - ego_y_;
      static_dist = std::hypot(sx, sy);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Static obj :: %.3g meters.", static_dist);

      const bool in_front = is_in_front_180(sx, sy, ego_yaw_, half_angle_deg_);
      const bool close_enough = (static_dist <= thresh_m_);

      auto path_d_opt = point_to_path_min_dist(st_x_, st_y_, global_path_);
      bool close_to_path = false;
      if (path_d_opt)
      {
        const double path_d = *path_d_opt;
        // 당신이 쓰던 double-threshold (0.5/1.0)
        if (!st_flag_memory_ && path_d <= 0.5) close_to_path = true;
        else if (st_flag_memory_ && path_d <= 1.0) close_to_path = true;
      }

      if (in_front && close_enough && close_to_path)
      {
        st_flag = true;
        const double path_d = path_d_opt ? *path_d_opt : 0.0;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Static VALID: dist=%.2fm, path_d=%.2fm", static_dist, path_d);
      }
      else
      {
        if (!in_front)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Static detected but NOT in front 180deg.");
        }
        else if (!close_enough)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Static farther than %.3gm.", thresh_m_);
        }
        else if (!path_d_opt)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Static in front but global path와의 거리 계산 안됨 -> ignore.");
        }
        else
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Static in front but not blocking path (dist to path=%.2fm).", *path_d_opt);
        }
      }

      st_flag_memory_ = st_flag;
    }

    // publish obj_flag: x=0(dynamic 없음), y=static flag, z=0(prioritize_dynamic 없음)
    geometry_msgs::msg::PointStamped out;
    out.header.stamp = get_clock()->now();
    out.point.x = 0.0;
    out.point.y = st_flag ? 1.0 : 0.0;
    out.point.z = 0.0;
    pub_flag_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "obj_flag published.");
  }

  // params
  double thresh_m_{15.0};
  double fresh_{0.3};
  double half_angle_deg_{100.0};
  double hz_{40.0};

  // state (ego)
  bool ego_ok_{false};
  double ego_t_{0.0};
  double ego_x_{0.0};
  double ego_y_{0.0};
  double ego_yaw_{0.0};

  // state (static)
  bool st_ok_{false};
  double st_t_{0.0};
  double st_x_{0.0};
  double st_y_{0.0};
  bool st_flag_memory_{false};

  // data
  nav_msgs::msg::Path::SharedPtr global_path_{nullptr};

  // ros i/o
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_flag_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ego_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_static_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CheckStaticObstacleOnly>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
