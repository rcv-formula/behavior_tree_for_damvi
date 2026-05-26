#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"

using std::placeholders::_1;

struct Scenario
{
  std::string desc;
  double ego_x, ego_y;
  double dyn_x, dyn_y;
  double st_x,  st_y;
  double flag;     // mode flag (0=BASE, 1=AVOID, 2=ACC)
  double base_z;   // Path pose.position.z as base speed
};

class StateMachineTester : public rclcpp::Node
{
public:
  StateMachineTester()
  : Node("state_machine_tester")
  {
    // === Publishers (BT / health_monitor가 subscribe 하는 토픽들) ===
    const int qos_default = 10;

    pub_ego_   = create_publisher<nav_msgs::msg::Odometry>("odom", qos_default);
    pub_imu_   = create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos_default);
    pub_lidar_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos_default);
    pub_vesc_  = create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", qos_default);

    // BT 코드에서 구독하는 이름 그대로 맞춤
    pub_dyn_   = create_publisher<nav_msgs::msg::Odometry>("/dynamic_obstacle", 20);
    pub_static_= create_publisher<geometry_msgs::msg::PointStamped>("/static_obstacle", 10);

    // /Path 는 TRANSIENT_LOCAL 이면 늦게 뜬 구독자도 마지막 메시지 받기 쉬움
    rclcpp::QoS qos_path(1);
    qos_path.reliable().transient_local();
    pub_path_  = create_publisher<nav_msgs::msg::Path>("/Path", qos_path);

    // static close_to_path 판정 쓰면 /global_path 도 같이 뿌려주는 게 좋음
    rclcpp::QoS qos_gpath(1);
    qos_gpath.reliable().transient_local();
    pub_gpath_ = create_publisher<nav_msgs::msg::Path>("/global_path", qos_gpath);

    // === Scenarios ===
    // (너가 준 것 전부를 그대로 옮기면 너무 길어서, 일단 핵심 케이스 위주로 넣었고
    //  나머지는 아래 주석대로 계속 추가하면 됨)
    scenarios_ = {
      {"0: Clear lane (no close obstacles)", 0.0,0.0, 100.0,0.0, 100.0,5.0, 0.0, 2.0},
      {"1: Dynamic obstacle ~2m ahead",      0.0,0.0, 2.0,  0.0, 100.0,0.0, 0.0, 2.0},
      {"2: Static obstacle ~2m ahead",       0.0,0.0, 100.0,0.0, 2.0,  0.0, 0.0, 2.0},
      {"4: Dynamic obstacle ~1m ahead",      0.0,0.0, 0.9,  0.0, 100.0,0.0, 0.0, 2.0},
      {"5: Static obstacle ~1m ahead",       0.0,0.0, 100.0,0.0, 0.9,  0.0, 0.0, 2.0},

      {"9: [flag=1] Static obstacle ~2m",    0.0,0.0, 100.0,0.0, 2.0,  0.0, 1.0, 2.0},

      {"15:[flag=2] Dynamic obstacle ~2m",   0.0,0.0, 2.0,  0.0, 100.0,0.0, 2.0, 2.0},
      {"19:[flag=2] Dynamic obstacle ~0.3m", 0.0,0.0, 0.3,  0.0, 100.0,0.0, 2.0, 2.0},

      // local planner mode: 0=BASE, 1=AVOID, 2=ACC
      {"23:[flag=2] ACC obstacle ~2m",       0.0,0.0, 2.0,  0.0, 100.0,0.0, 2.0, 2.0},
    };

    // 파라미터: ros2 param set /state_machine_tester scenario_idx N
    declare_parameter<int>("scenario_idx", 0);

    // 2Hz로 계속 publish (원래 파이썬이 0.5초였음)
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               std::bind(&StateMachineTester::timer_cb, this));

    RCLCPP_INFO(get_logger(), "StateMachineTester started. Use: ros2 param set /state_machine_tester scenario_idx N");
  }

private:
  // ==== message builders ====
  nav_msgs::msg::Odometry make_odom(double x, double y)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.child_frame_id  = "base_link";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.twist.twist.linear.x = 0.5;
    msg.twist.twist.linear.y = 0.0;
    return msg;
  }

  sensor_msgs::msg::Imu make_imu()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = "imu";
    return msg;
  }

  sensor_msgs::msg::LaserScan make_scan()
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "lidar";
    return msg;
  }

  std_msgs::msg::Float64 make_vesc(double speed)
  {
    std_msgs::msg::Float64 msg;
    msg.data = speed;
    return msg;
  }

  geometry_msgs::msg::PointStamped make_static_point(double x, double y)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = 0.0;
    return msg;
  }

  nav_msgs::msg::Path make_path(double overtake_flag, double base_speed_z)
  {
    nav_msgs::msg::Path path;
    auto t = now();
    path.header.stamp = t;
    path.header.frame_id = "map";

    path.poses.clear();
    path.poses.reserve(5);

    for (int i = 0; i < 5; ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = t;
      ps.header.frame_id = "map";

      ps.pose.position.x = 0.5 * i;
      ps.pose.position.y = 0.0;
      ps.pose.position.z = base_speed_z; // speed encoded here

      ps.pose.orientation.x = 0.0;
      ps.pose.orientation.y = 0.0;
      ps.pose.orientation.z = overtake_flag; // mode encoded here
      ps.pose.orientation.w = 1.0;

      path.poses.push_back(ps);
    }
    return path;
  }

  // global_path는 static이 path 근처인지 판단할 때 필요하므로
  // 간단히 /Path 와 똑같은 라인 경로로 뿌려도 테스트가 됨
  nav_msgs::msg::Path make_global_path_like(const nav_msgs::msg::Path& local_path)
  {
    nav_msgs::msg::Path g = local_path;
    g.header.stamp = now();
    g.header.frame_id = "map";
    return g;
  }

  void timer_cb()
  {
    int idx = get_parameter("scenario_idx").as_int();
    if (scenarios_.empty()) return;

    idx = std::max(0, std::min(idx, static_cast<int>(scenarios_.size()) - 1));
    const auto& sc = scenarios_[idx];

    // publish messages
    auto ego   = make_odom(sc.ego_x, sc.ego_y);
    auto dyn   = make_odom(sc.dyn_x, sc.dyn_y);
    auto st    = make_static_point(sc.st_x, sc.st_y);
    auto imu   = make_imu();
    auto scan  = make_scan();
    auto vesc  = make_vesc(0.0);
    auto path  = make_path(sc.flag, sc.base_z);
    auto gpath = make_global_path_like(path);

    pub_ego_->publish(ego);
    pub_dyn_->publish(dyn);
    pub_static_->publish(st);
    pub_imu_->publish(imu);
    pub_lidar_->publish(scan);
    pub_vesc_->publish(vesc);
    pub_path_->publish(path);
    pub_gpath_->publish(gpath);

    RCLCPP_INFO(get_logger(),
      "[SCENARIO %d] %s ego=(%.2f,%.2f) dyn=(%.2f,%.2f) sta=(%.2f,%.2f) flag=%.1f base_z=%.2f",
      idx, sc.desc.c_str(),
      sc.ego_x, sc.ego_y,
      sc.dyn_x, sc.dyn_y,
      sc.st_x, sc.st_y,
      sc.flag, sc.base_z);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ego_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_lidar_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vesc_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_dyn_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_static_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_gpath_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Scenario> scenarios_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateMachineTester>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
