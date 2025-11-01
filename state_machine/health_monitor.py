#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

# 토픽 이름은 환경에 맞게 수정하세요.
IMU_TOPIC   = '/imu/data'
LIDAR_TOPIC = '/scan'
VESC_TOPIC  = '/commands/motor/speed'

# 주파수에 맞춰 타임아웃 조절
TIMEOUTS_SEC = {'imu': 0.5, 'lidar': 0.5, 'vesc': 1.0} # 각각 Hz 맞춰서 수정!!
GRACE_SEC = 3.0        # 시작하고 3초까진 안와도 봐줌.
DEBOUNCE_SEC = 0.3     # 잠깐의 끊김은 인정해줌.

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        now = self._now()   # 현재 시각 초기화.
        self.last = {'imu': now, 'lidar': now, 'vesc': now} # 딕셔너리로 선언.
        self.fail_since = None  # 작동 안하기 시작한 시간
        self.reason = 'startup' # 초기 단계는 startup.

        # lambda는 여기 자리를 안쓴다는 뜻. 걍 토픽이 오면 저 mark 함수를 실행하라는 뜻으로 사용이 된다~
        self.create_subscription(Imu,       IMU_TOPIC,   lambda _: self._mark('imu'),   10) # mark함수(센서별 시간맞추기) 실행
        self.create_subscription(LaserScan, LIDAR_TOPIC, lambda _: self._mark('lidar'), 10) # 10은 큐인데 기본 코드에도 이렇게 사용하고 있어서..
        self.create_subscription(Float64,  VESC_TOPIC,  lambda _: self._mark('vesc'),  10)

        self.pub_ok = self.create_publisher(Bool,   '/system/critical_ok', 1)   # imu, lidar, vesc 세개가 없으면 아예 모든게 꺼져서 critical이라고 명명.
        self.pub_rs = self.create_publisher(String, '/system/critical_reason', 1)
        self.create_timer(0.1, self.tick)

    def _now(self): return self.get_clock().now().nanoseconds / 1e9
    def _mark(self, k): self.last[k] = self._now()  # 각 항목이 정상일때 그 확인 시간

    def _all_ok(self, now):
        for k, to in TIMEOUTS_SEC.items():  # 타임아웃 시간 넘은 경우 all_ok를 false로.
            if now - self.last[k] > to: # 각 항목별로 timeout sec보다 넘긴 경우.
                self.reason = f'timeout:{k}'    # 어떤게 타임아웃이다 라고 나오면서 false출력
                self.get_logger().warn(self.reason)
                # self.pub_rs.publish(String(self.reason))
                # self.get_logger().warn(str(self.last[k]))  굳이 시간 표시할 필요 없어. warn으로 하면.
                return False
        self.reason = 'ok'
        return True

    # /scan publisher는 있는데 센서가 꺼져있으면 보내질 않아서 센서가 켜져있을때만 콜백을 실행하게 됨. 이러면 굳이 이렇게 볼 필요 없이 imu, vesc 보듯이 콜백실행할때 마킹하도록 하면 됨. 
    # def cb_lidar(self, msg:LaserScan):
    #     # /scan 의 내용이 들어있지 않은경우 lidar가 꺼진 경우이다!
    #     self.get_logger().warn("lidar scan topic echo length : "+str(len(msg.ranges)))
    #     if len(msg.ranges) == 0:
    #         self.reason = 'lidar empty /scan topic, maybe lidar down.'
    #         self.get_logger().warn(self.reason)
    #         return
    #     else:
    #         self.reasen = 'lidar ok'
    #         self._mark('lidar') # 라이다 정상이라고 판단.
    #         return
        

    def tick(self):
        now = self._now()
        if now - self.t0 < GRACE_SEC:   # 시작 유예 시간 안에 있으면 봐줌.
            self.pub_ok.publish(Bool(data=True))
            self.pub_rs.publish(String(data='grace'))
            return

        ok = self._all_ok(now)  # 센서가 모두 잘 작동하는지 검사.(all_ok 함수 실행)
        if ok:  # 문제없이 모두 작동하는 경우
            self.fail_since = None
            self.pub_ok.publish(Bool(data=True))    # critical_ok를 True로 퍼블리쉬.
            self.pub_rs.publish(String(data='ok'))
            self.get_logger().warn("In this tick, sensors are ok. Now go to next tick")
        else:
            if self.fail_since is None: 
                self.fail_since = now
            hard_fail = (now - self.fail_since) >= DEBOUNCE_SEC
            self.pub_ok.publish(Bool(data=not hard_fail))
            self.pub_rs.publish(String(data=self.reason if hard_fail else 'debouncing'))
            self.get_logger().warn("In this tick, sensors are down. Now go to next tick")

def main():
    rclpy.init()
    rclpy.spin(HealthMonitor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
