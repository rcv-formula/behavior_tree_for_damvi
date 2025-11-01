#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, TwistWithCovariance, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

# ros2 param set /state_machine_tester scenario_idx 2
# 등 맨 뒤에 숫자를 이용해서 시나리오 전환 가능.
# 또 ros2 topic echo selected_path
# 에서 orientation.z 쪽 보면 속도 확인 가능. 정상 속도는 2로 해뒀음.

# === 실제 코드 import로 교체하세요 ===
# from your_pkg_name.your_file import CheckObstacle, SelectPath
from state_machine.behaviours2 import CheckObstacle, SelectPath

class StateMachineTester(Node):
    def __init__(self):
        super().__init__('state_machine_tester')

        # 실제 Behaviour 인스턴스
        self.check_obstacle_bt = CheckObstacle(self)
        self.select_path_bt    = SelectPath(self)

        # 우리가 publish할 토픽들 (테스트 노드 -> 실제 BT노드가 subscribe)
        qos_default = 10
        self.pub_ego    = self.create_publisher(Odometry,    'odom',         qos_default)
        self.pub_imu    = self.create_publisher(Imu,         '/imu/data',    qos_default)
        self.pub_lidar  = self.create_publisher(LaserScan,   '/scan',        qos_default)
        self.pub_vesc   = self.create_publisher(Float64,       '/commands/motor/speed', qos_default)
        self.pub_dyn    = self.create_publisher(Odometry,    '/dynamic_obj', qos_default)
        self.pub_static = self.create_publisher(PoseStamped, '/static_obj',  qos_default)

        qos_localpath = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_path   = self.create_publisher(Path,        'Path',         qos_localpath)

        # 시나리오들:
        # 각 인덱스마다 한 장면(ego/dyn/static 위치, overtake_flag, base_speed_z)
        # 설명:
        #   desc: 그냥 로깅용
        #   ego_xy:    (ego_x, ego_y)
        #   dyn_xy:    (dynamic obstacle x,y)
        #   sta_xy:    (static obstacle x,y)
        #   flag:      overtake_flag (0=global,1=static avoid,2=overtake,3=ACC)
        #   base_z:    Path상의 기본 속도(z). 이후 /10,/2 등으로 scale되는지 확인 가능
        self.scenarios = [
            # global 추종 모드(overtake_flag = 0)
            {
                "desc": "0: Clear lane (no close obstacles)",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (100.0, 5.0),
                "flag": 0.0,    # global 추종에서 유효 장애물 X
                "base_z": 2.0,
            },
            {
                "desc": "1: Dynamic obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 0.0,   # global 추종에서 dynamic만 2m => 유효장애물내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "2: Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 0.0,   # global 추종에서 static만 2m => 유효장애물 내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "3: Dynamic and Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 0.0,   # global 추종에서 장애물 모두 2m => 유효장애물 내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "4: Dynamic obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 0.0,   # global 추종에서 Dynamic만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "5: Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 0.0,   # global 추종에서 Static만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "6: Dynamic and Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 0.0,   # global 추종에서 둘다 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            # static 회피 모드(overtake_flag = 1)
            {
                "desc": "7: Clear lane (no close obstacles)",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (100.0, 5.0),
                "flag": 1.0,    # static 회피에서 유효 장애물 X
                "base_z": 2.0,
            },
            {
                "desc": "8: Dynamic obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 1.0,   # static 회피에서 dynamic만 2m => 유효장애물내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "9: Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 1.0,   # static 회피에서 static만 2m => 유효장애물 내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "10: Dynamic and Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 1.0,   # static 회피에서 장애물 모두 2m => 유효장애물 내인 global 추종
                "base_z": 2.0,
            },
            {
                "desc": "11: Dynamic obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 1.0,   # static 회피에서 Dynamic만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "12: Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 1.0,   # static 회피에서 Static만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "13: Dynamic and Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 1.0,   # static 회피에서 둘다 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            # Dynamic 추월 모드(overtake_flag = 2)
            {
                "desc": "14: Clear lane (no close obstacles)",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (100.0, 5.0),
                "flag": 2.0,    # Dynamic 추월에서 유효 장애물 X
                "base_z": 2.0,
            },
            {
                "desc": "15: Dynamic obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 dynamic만 2m => 유효장애물내인 Dynamic 추월
                "base_z": 2.0,
            },
            {
                "desc": "16: Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 static만 2m => 유효장애물 내인 Dynamic 추월
                "base_z": 2.0,
            },
            {
                "desc": "17: Dynamic and Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 장애물 모두 2m => 유효장애물 내인 Dynamic 추월
                "base_z": 2.0,
            },
            {
                "desc": "18: Dynamic obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 Dynamic만 1m이내 => 추월 유지!!(유효장애물 내인 Dynamic 추월)
                "base_z": 2.0,
            },
            {
                "desc": "19: Dynamic obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.3, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 Dynamic만 0.5m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "20: Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 Static만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "21: Dynamic and Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.2, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 2.0,   # Dynamic 추월에서 둘다 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            # ACC 모드(overtake_flag = 3)
            {
                "desc": "22: Clear lane (no close obstacles)",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (100.0, 5.0),
                "flag": 3.0,    # ACC 모드에서 유효 장애물 X
                "base_z": 2.0,
            },
            {
                "desc": "23: Dynamic obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 dynamic만 2m => 속도 절반으로 추종
                "base_z": 2.0,
            },
            {
                "desc": "24: Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 static만 2m => 정상속도 주행
                "base_z": 2.0,
            },
            {
                "desc": "25: Dynamic and Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (2.0, 0.0),
                "sta_xy": (2.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 장애물 모두 2m => 속도 절반으로 추종(사실 로컬플래너가 ACC모드로 하질 않겠지)
                "base_z": 2.0,
            },
            {
                "desc": "26: Dynamic obstacle ~5m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (5.0, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 dynamic만 5m,static은 없음 => 정상속도로 추종
                "base_z": 2.0,
            },
            {
                "desc": "27: Static obstacle ~2m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (5.0, 0.0),
                "sta_xy": (5.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 dynamic만 5m, static은 5m => 정상속도 주행
                "base_z": 2.0,
            },
            {
                "desc": "28: Dynamic obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (100.0, 0.0),
                "flag": 3.0,   # ACC 모드에서 Dynamic만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "29: Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (100.0, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 3.0,   # ACC 모드에서 Static만 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
            {
                "desc": "30: Dynamic and Static obstacle ~1m ahead",
                "ego_xy": (0.0, 0.0),
                "dyn_xy": (0.9, 0.0),
                "sta_xy": (0.9, 0.0),
                "flag": 3.0,   # ACC 모드에서 둘다 1m이내 => 비상정지(속도 1/10)
                "base_z": 2.0,
            },
        ]

        # 현재 어떤 시나리오를 재생 중인지: 파라미터로 노출
        # ros2 param set /state_machine_tester scenario_idx 3   처럼 바꿀 수 있음
        self.declare_parameter('scenario_idx', 0)

        # 주기적으로 현재 scenario_idx를 읽어서 그 장면을 publish + BT update
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2Hz

    #####################################################
    # 메시지 생성 유틸들
    #####################################################
    def make_odom_msg(self, x, y):
        msg = Odometry()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.child_frame_id  = "base_link"

        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        msg.twist = TwistWithCovariance()
        msg.twist.twist = Twist()
        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.linear.y = 0.0
        return msg

    def make_imu_msg(self):
        msg = Imu()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "imu"
        return msg
    
    def make_lidar_msg(self):
        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "lidar"
        return msg
    
    def make_vesc_msg(self, speed):
        msg = Float64()
        msg.data = speed
        return msg

    def make_pose_stamped(self, x, y):
        msg = PoseStamped()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "map"

        msg.pose = Pose()
        msg.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return msg

    def make_path_msg(self, overtake_flag, base_speed_z):
        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "map"

        poses_list = []
        for i in range(5):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = "map"

            # position.z 를 속도처럼 사용 (나중에 /10, /2로 줄어드는지 확인)
            ps.pose.position = Point(x=0.5 * i, y=0.0, z=base_speed_z)

            # orientation.z 에 overtake_flag를 encode (원래 코드 규칙)
            ps.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=float(overtake_flag),
                w=1.0
            )
            poses_list.append(ps)

        path_msg.poses = poses_list
        return path_msg

    #####################################################
    # 시나리오 1회 수행 (한 번의 tick)
    #####################################################
    def run_one_tick_with_scenario(self, sc_idx: int):
        # 범위 방어
        if sc_idx < 0:
            sc_idx = 0
        if sc_idx >= len(self.scenarios):
            sc_idx = len(self.scenarios) - 1

        sc = self.scenarios[sc_idx]

        ego_xy = sc["ego_xy"]
        # imu_xy = sc["imu_xy"] 바뀌는게 없으니까 imu는 케이스마다.
        dyn_xy = sc["dyn_xy"]
        sta_xy = sc["sta_xy"]
        flag   = sc["flag"]
        base_z = sc["base_z"]

        # 메시지 만들고 publish → 실제 BT 클래스들이 subscribe해서 내부 상태 갱신
        ego_msg = self.make_odom_msg(ego_xy[0], ego_xy[1])
        imu_msg = self.make_imu_msg()
        lidar_msg = self.make_lidar_msg()
        vesc_msg = self.make_vesc_msg(0.0)  # 속도 0 고정
        dyn_msg = self.make_odom_msg(dyn_xy[0], dyn_xy[1])
        sta_msg = self.make_pose_stamped(sta_xy[0], sta_xy[1])
        path_msg= self.make_path_msg(flag, base_z)

        self.pub_ego.publish(ego_msg)
        self.pub_imu.publish(imu_msg)
        self.pub_lidar.publish(lidar_msg)
        self.pub_vesc.publish(vesc_msg)
        self.pub_dyn.publish(dyn_msg)
        self.pub_static.publish(sta_msg)
        self.pub_path.publish(path_msg)

        # 로그
        self.get_logger().info(
            f"[SCENARIO {sc_idx}] {sc['desc']} "
            f"ego={ego_xy} dyn={dyn_xy} sta={sta_xy} flag={flag}, base_z={base_z}"
        )

        # py_trees에서는 normally tree.tick() 이지만
        # 지금은 Behaviour를 직접 update() 호출
        status_check  = self.check_obstacle_bt.update()
        status_select = self.select_path_bt.update()

        # Blackboard에서 거리 등 확인
        bb = self.check_obstacle_bt.bb
        self.get_logger().info(
            f"CheckObstacle Status={status_check} | "
            f"dynamic_dist={bb.dynamic_distance:.2f} "
            f"static_dist={bb.static_distance:.2f} "
            f"dyn_flag={bb.dynamic_obstacle} "
            f"sta_flag={bb.static_obstacle} "
            f"prio_dyn={self.check_obstacle_bt.prioritize_dynamic_flag}"
        )
        self.get_logger().info(
            f"SelectPath Status={status_select} | overtake_flag={self.select_path_bt.bb.overtake_flag}"
        )
        # selected_path / obj_flag 등은 ros2 topic echo 로 외부에서 확인

    #####################################################
    # timer: 매 틱마다 현재 scenario_idx 파라미터 값 읽어서 그 상황 실행
    #####################################################
    def timer_cb(self):
        sc_idx_param = self.get_parameter('scenario_idx').get_parameter_value().integer_value
        self.run_one_tick_with_scenario(sc_idx_param)


def main():
    rclpy.init()
    node = StateMachineTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()