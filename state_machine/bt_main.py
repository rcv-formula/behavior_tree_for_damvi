#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from py_trees.common import Status
from std_msgs.msg import Bool
from nav_msgs.msg import Path
# from ackermann_msgs.msg import AckermannDriveStamped

from state_machine.behaviours2 import CheckObstacle, SelectPath

# -----------------------
# 블랙보드(공유 상태)
# pytrees의 경우 굳이 topic을 publish, subscribe하지 않고 블랙보드를 사용하는데,
# 그 이유는 오히려 publish, subscribe하는게 시간이 더 걸린다는..
# 블랙보드는 딕셔너리같은 메모리 객체라서 O(1)이라고합니다.
# 그런데 publish,subscribe는 중간에 DDS 네트워크 계층, 직렬화/역직렬화가 들어가서 더 먹는다고 합니다.                
# -----------------------

# -----------------------
# BT 노드들
# -----------------------
class CondCriticalOK(py_trees.behaviour.Behaviour):    # 한 틱의 마지막에 imu, lidar, vesc 상태 확인하고 BB에 저장.
    """ health_monitor.py에서 퍼블리쉬하는 토픽인 /system/critical_ok 구독 전용 (항상 SUCCESS) """
    def __init__(self, node: Node):
        super().__init__("CondCriticalOK")
        self.node = node
        self.ok = False
        self.t = 0.0
        self.fresh = 0.5 # 0.5초안에 안오면 이상하다 판단.
        self.node.create_subscription(Bool, 'system/critical_ok', self.cb, 1)
        self.node.create_subscription(Bool, 'system/critical_reason', self.cb, 1)

    def now(self):  return self.node.get_clock().now().nanoseconds/1e9

    def cb(self, msg: Bool):
        self.ok = bool(msg.data)
        self.t = self.now()
        self.node.get_logger().warn("I got the message, critical_ok is "+str(self.ok))
    
    def cb_reason(self,msg:Bool):
        reason = msg.data
        self.node.get_logger().warn(reason)

    def update(self):
        now = self.now()
        if (self.t == 0.0 or (now-self.t) > self.fresh):
            self.node.get_logger().warn("/system/critical_ok can't subscribable. CondCriticalOK Failure.")
            # self.node.get_logger().warn(str(self.t))
            return Status.FAILURE
        if self.ok:
            self.node.get_logger().info("IMU,VESC,LiDAR working.")
            return Status.SUCCESS   # 트리구조에서 inverter로 사용하기 때문에 실제로는 FAILURE를 뽑게 됨.
        else:
            return Status.FAILURE
        # if not self.criticalok:
        #     self.node.get_logger().warn("One of imu, vesc, lidar doesn't working!!")
        #     # self.node.get_logger().warn("/odom can't subscribable. CheckObstacle Failure.")
        #     return Status.FAILURE
        # else:
        #     print("IMU,VESC,LiDAR working.")
        #     return Status.SUCCESS
    
# 일단 BB 관련 제외.
# class CondCriticalOK(py_trees.behaviour.Behaviour):     # 얘는 BB에서 받아옴. 속도 빠르겠지.
#     """ 임계 센서 모두 정상인지 판정 """
#     def update(self):
#         return Status.SUCCESS if BB.critical_ok else Status.FAILURE

# class CondObstacleByDistance(py_trees.behaviour.Behaviour):
#     """ /distance(Float64)가 thresh 이하 & 최신이면 SUCCESS"""
#     def __init__(self,node:Node,thresh_m=3.0):
#         super().__init__("Obstacle<="+str(thresh_m)+"m?")
#         self.node=node
#         self.thresh = float(thresh_m)
#         self.dist = None
#         self.t = 0.0
#         self.fresh = 0.5 # 0.5초 기간
#         self.node.create_subscription(Float64, 'distance', self.cb_dist, 10)	# distance publish 안하기로 했어. BB에 저장하는거로 하자.

#     def now(self) : return self.node.get_clock().now().nanoseconds/1e9

#     def cb_dist(self,msg):
#         self.dist = msg.data
#         self.t = self.now()
    
#     def update(self):
#         now = self.now()
#         self.node.get_logger().info("CondObstacleByDistance Node start")
#         if self.t == 0.0 or (now-self.t)>self.fresh:
#             self.node.get_logger().info("self t = 0.0 or now-self.t > self.fresh")
#             return Status.FAILURE
#         # 3m 이내이면 SUCCESS, 아니면 FAILURE 발동.
#         return Status.SUCCESS if (self.dist is not None and self.dist <= self.thresh) else Status.FAILURE


class CondLocalPathAvailable(py_trees.behaviour.Behaviour):
    """local path가 최근에 왔으면 SUCCESS"""
    def __init__(self, node:Node):
        super().__init__("LocalPath?")
        self.node=node
        self.t=0.0
        self.fresh=0.5
        self.node.create_subscription(Path,'local_path',self.cb,10)

    def now(self):  return self.node.get_clock().now().nanoseconds/1e9

    def cb(self,msg:Path):
        self.t = self.now()

    def update(self):
    	#self.node.get_logger().info("CondLocalPathAvailable Node start")
        return Status.SUCCESS if self.t != 0.0 and (self.now()-self.t)<=self.fresh else Status.FAILURE
    

class EmergencyStop(py_trees.behaviour.Behaviour):      # 프로그램을 꺼야하나 고민했는데 다시 연결할 수도 있지 않을까 싶어서 일단 속도를 0으로 만드는것으로 생각했습니다.
    """ 간단 E-stop: 속도 0 명령 전송 """
    def __init__(self, node: Node, name="E-Stop", succeed=False):
        super().__init__(name)
        self.node = node
        self.succeed = succeed
        
    def update(self):
        # 급정거
        self.node.get_logger().info("Estop SUCCESS")
        return Status.SUCCESS if self.succeed else Status.FAILURE

# -----------------------
# 트리 구성
# -----------------------
def build_tree(node: Node):
    # 가드: !CriticalOK -> EStop
    cond_ok = CondCriticalOK(node)
    estop_guard = EmergencyStop(node, name="E-Stop[Guard]", succeed=False)
    guard_seq = py_trees.composites.Selector(name="CriticalGuard", memory=False)
    guard_seq.add_children([cond_ok, estop_guard]) # Sequence 노드의 자식으로 설정.

    # 장애물 있을때 : LPP가 오면 LPP 사용, 아니면 GPP(slow)
    obstacle = CheckObstacle(node,thresh_m=7.0)
    # lpp_ready = CondLocalPathAvailable(node)

    # # Select Path에서 GPP랑 LPP 선택해서 mpc로 전달.
    # lpp_seq = py_trees.composites.Sequence(name="LPP 가능?",memory=False)
    # lpp_seq.add_children([lpp_ready, UseLPP(node)])

    # obstacle_branch = py_trees.composites.Selector(name="장애물 처리",memory=False)
    # obstacle_branch.add_children([lpp_seq, UseGPP(node,slow=True)])

    # 메인 : 장애물이면 위 분기, 아니면 GPP(normal)
    # obstacle_seq = py_trees.composites.Sequence(name="ObstacleSeq", memory=False)
    #estop_obs = EmergencyStop(node, name="E-Stop[Obstacle]") => SelectPath에서 비상정지까지 처리.
    # obstacle_seq.add_children([obstacle])

    # main_fb = py_trees.composites.Selector(name="Main",memory=False)
    # main_fb.add_children([obstacle_seq, UseGPP(node,slow=False)])  # 장애물 없으면 GPP

    main_fb = py_trees.composites.Selector(name="Main",memory=False)
    select_path = SelectPath(node)
    main_fb.add_children([obstacle, select_path])

    # # 루트 : (1) 가드 -> (2) 메인
    # obs_publisher = py_trees.decorators.SuccessIsRunning(name ="ObsPublisher",child=CheckObstacle(node))

    root = py_trees.composites.Sequence(name="Root",memory=False)
    root.add_children([guard_seq, main_fb])
    return py_trees.trees.BehaviourTree(root)

# -----------------------
# 실행 루프
# -----------------------
def main():
    rclpy.init()
    node = rclpy.create_node('bt_main')
    tree = build_tree(node)
    
    print(py_trees.display.ascii_tree(tree.root))
    py_trees.display.render_dot_tree(tree.root, name="bt_tree")

    # 10 Hz tick
    # rate = node.create_rate(10) # 10Hz
    # last_time = node.get_clock().now().nanoseconds/1e9
    # next_time = node.get_clock().now().nanoseconds/1e9
    try:
        while rclpy.ok():
            # print("New tick start!!!!")
            rclpy.spin_once(node, timeout_sec=0.025)  # 40hz 주기로 돌게 설정! (실제 확인 완료)
            tree.tick()
            # print("one tick finished!!!")

            # next_time = node.get_clock().now().nanoseconds/1e9
            # print("one tick time = ", next_time-last_time)
            # rate.sleep()
    finally:
        # print("finally destroy node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
