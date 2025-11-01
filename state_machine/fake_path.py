#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class SimplePathPublisher(Node):
    def __init__(self):
        super().__init__('simple_path_publisher')

        # QoS: Path/global_path 는 Latched(Transient Local), Reliable, KeepLast(1)
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # 기본 QoS: depth=10만 지정
        default_qos = QoSProfile(depth=10)

        # Publishers
        self.path_pub_    = self.create_publisher(Path, 'Path', latched_qos)
        self.global_pub_  = self.create_publisher(Path, 'global_path', latched_qos)
        self.odom_pub_    = self.create_publisher(Odometry, '/odom', default_qos)
        self.dynamic_pub_ = self.create_publisher(Odometry, '/dynamic_obj', default_qos)
        self.static_pub_  = self.create_publisher(PoseStamped, '/static_obj', default_qos)

        # 고정 경로 좌표 (Path/global_path)
        self.points = [
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
            (2.0, 2.0, 0.0)
        ]

        # Path orientation 값(요청대로 고정 0.0)
        self.orientation_value = 2.0

        # /odom은 항상 (0,0,0)
        self.odom_xyz = (0.0, 0.0, 0.0)

        # 거리 순환(단위 m): /odom과의 유클리드 거리
        self.distances = [0.5, 3.0, 8.0]
        self.dynamic_idx = 0
        self.static_idx  = 0
        self.dynamic_current_d = self.distances[self.dynamic_idx]
        self.static_current_d  = self.distances[self.static_idx]

        # global_path는 한 번만 퍼블리시
        self.publish_global_path()

        # === Timers ===
        # Path는 10Hz로 계속 퍼블리시
        self.path_timer       = self.create_timer(0.1, self.publish_path)
        # odom은 1Hz로 계속 퍼블리시
        self.odom_timer       = self.create_timer(1.0, self.publish_odom)

        # dynamic/static 퍼블리시 주기(연속 퍼블리시): 10Hz
        self.dynamic_pub_timer = self.create_timer(0.1, self.publish_dynamic_continuous)
        self.static_pub_timer  = self.create_timer(0.1, self.publish_static_continuous)

        # dynamic/static의 거리 변경 주기: 15초마다 다음 값으로 변경
        self.dynamic_change_timer = self.create_timer(15.0, self.update_dynamic_distance)
        self.static_change_timer  = self.create_timer(5.0, self.update_static_distance)

    # ===== Publishers =====
    def publish_global_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for p in self.points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.global_pub_.publish(msg)
        self.get_logger().info(f'Published global_path with {len(msg.poses)} points (once)')

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for p in self.points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
            pose.pose.orientation.z = self.orientation_value  # 고정 0.0
            msg.poses.append(pose)
	
	#msg.poses[0].pose.orientation.z = 2.0
	
        self.path_pub_.publish(msg)

    def publish_odom(self):
        x, y, z = self.odom_xyz
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub_.publish(odom)

    # ---- dynamic_obj: 연속 퍼블리시 (10Hz), 15초마다 거리만 변경 ----
    def publish_dynamic_continuous(self):
        d = self.dynamic_current_d
        ox, oy, oz = self.odom_xyz
        px, py, pz = ox + d, oy + 0.0, oz + 0.0  # +x 방향으로 d만큼

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'dynamic_link'
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.w = 1.0
        self.dynamic_pub_.publish(odom)

    def update_dynamic_distance(self):
        self.dynamic_idx = (self.dynamic_idx + 1) % len(self.distances)
        self.dynamic_current_d = self.distances[self.dynamic_idx]
        self.get_logger().info(f'[dynamic_obj] distance -> {self.dynamic_current_d} m')

    # ---- static_obj: 연속 퍼블리시 (10Hz), 5초마다 거리만 변경 ----
    def publish_static_continuous(self):
        d = self.static_current_d
        ox, oy, oz = self.odom_xyz
        px, py, pz = ox + d, oy + 0.0, oz + 0.0  # +x 방향으로 d만큼

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.w = 1.0
        self.static_pub_.publish(pose)

    def update_static_distance(self):
        self.static_idx = (self.static_idx + 1) % len(self.distances)
        self.static_current_d = self.distances[self.static_idx]
        self.get_logger().info(f'[static_obj] distance -> {self.static_current_d} m')


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

