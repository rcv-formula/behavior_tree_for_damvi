#!/usr/bin/env python3
import math, rclpy
import py_trees
from py_trees.common import Access, Status
from py_trees.blackboard import Client as BBClient
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool, String, Float64 # state publish를 위해.
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from copy import deepcopy

# 장애물 감지 자체는 state machine에서 하지 않을것으로 생각됩니다.
# 장애물이 감지되었다는 결과를 토픽으로 받을 것으로 생각하고 있습니다.
# # === 장애물 감지 노드입니다. ===
class CheckObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, node, thresh_m=7.0):   # pytrees에서는 모든 노드가 내부적으로 이름을 가진대. 이름 설정.
        super().__init__("Obstacle <= "+str(thresh_m)+"m?")
        self.node = node

        # 제일 처음엔 초기화.
        self.ego_timestamp = 0.0    # 내 차
        self.ego_x = None
        self.ego_y = None
        self.dynamic_timestamp = 0.0    # 상대 차(dynamic)
        self.dynamic_x = None
        self.dynamic_y = None
        # self.dynamic_obstacle = False   # dynamic obstacle Flag
        self.static_timestamp = 0.0 # 정적 장애물 (반드시 1개)
        self.static_x = None
        self.static_y = None
        # self.static_obstacle = False
        self.thresh_m = thresh_m    # 유효장애물 인식 사거리
        self.estop_thresh_m = 0.5   # 비상정지 인식 사거리 (50cm)
        self.fresh = 15.0            # 신호 지연 기준 시간 ------------------------------------> 지금은 3으로 일부러 늘려놨는데 0.5로 설정해!! 실제에서는!!
        self.prioritize_dynamic_flag = True   # True(1.0) dynamic, False(0)이 static을 우선으로 설정.

        # BB에 저장.
        self.bb = BBClient(name = "CheckObstacle_writer")
        self.bb.register_key("dynamic_obstacle", access=Access.WRITE)
        self.bb.register_key("static_obstacle", access=Access.WRITE)
        self.bb.register_key("dynamic_distance", access=Access.WRITE)
        self.bb.register_key("static_distance", access=Access.WRITE)
        self.bb.register_key("overtake_flag", access=Access.READ)   # 추월OX에 따라서 비상정지 거리 조절하려고.

        self.bb.dynamic_obstacle = False
        self.bb.static_obstacle = False
        self.bb.dynamic_distance = 100  # 이렇게 초기화해놔야 scaled path가 생성안된채로 시작. 만약 0으로 두면 바로 estop뜸.
        self.bb.static_distance = 100

        self.node.create_subscription(Odometry, 'odom', self.cb_ego_odom, 10)
        self.node.create_subscription(Odometry, '/dynamic_obj', self.cb_dynamic_odom, 10)
        self.node.create_subscription(PoseStamped, '/static_obj', self.cb_static_odom, 10)
        self.publish_flag = self.node.create_publisher(PointStamped, '/obj_flag', 1)
    
    # 현재 시각 확인
    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9
    
    # 내 위치 callback 함수
    def cb_ego_odom(self,msg:Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ego_x = p.x
        self.ego_y = p.y
        self.ego_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        v = msg.twist.twist.linear
        self.ego_vel = (v.x, v.y)
        self.ego_timestamp = self.now()

    # 상대 차(dynamic) 위치 callback 함수
    def cb_dynamic_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.dynamic_x = p.x
        self.dynamic_y = p.y
        self.dynamic_vel = (v.x, v.y)
        self.dynamic_timestamp = self.now()  # 받은 시각 refresh

    # 정적 장애물 (static) 위치 callback 함수
    # 정적 장애물은 속도가 없겠지. 치고 가지 않는 이상.
    def cb_static_odom(self, msg: PoseStamped):
        p = msg.pose.position
        self.static_x = p.x
        self.static_y = p.y
        self.static_timestamp = self.now()

    def update(self):
        now = self.now()  
        if (self.ego_timestamp == 0.0 or (now - self.ego_timestamp) > self.fresh):
            self.node.get_logger().warn("/odom can't subscribable. CheckObstacle Failure.")
            return Status.FAILURE
        if (self.dynamic_timestamp == 0.0 or (now - self.dynamic_timestamp) > self.fresh):
            self.node.get_logger().warn("/dynamic_obj can't subscribable. CheckObstacle Failure.")
            return Status.FAILURE
        if (self.static_timestamp == 0.0 or (now - self.static_timestamp) > self.fresh):
            self.node.get_logger().warn("/static_obj can't subscribable. CheckObstacle Failure.")
            return Status.FAILURE
        
        dynamic_dx = self.dynamic_x - self.ego_x
        dynamic_dy = self.dynamic_y - self.ego_y
        dynamic_dist = math.hypot(dynamic_dx,dynamic_dy) # 동적장애물과의 거리

        static_dx = self.static_x - self.ego_x
        static_dy = self.static_y - self.ego_y
        static_dist = math.hypot(static_dx, static_dy)

        self.bb.dynamic_distance = dynamic_dist
        self.bb.static_distance = static_dist

        
    
        
        
        ## 동적장애물 거리
        # 비상정지의 경우에만 바로 SUCCESS를 뱉음. 그러면 다음 노드인 E-stop을 하게됨.
        if self.bb.dynamic_distance <= self.estop_thresh_m:
            self.bb.dynamic_obstacle = True
            self.node.get_logger().info("EStop : Dynamic obj : "+str(self.bb.dynamic_distance)+" meters.")            
            return Status.SUCCESS
        # 주변에 7m 내부에 장애물이 있으면 dynamic obstacle flag True로.
        elif self.bb.dynamic_distance <= self.thresh_m:
            self.bb.dynamic_obstacle = True
            self.node.get_logger().info("Dynamic obj : "+str(self.bb.dynamic_distance)+" meters.")
        else:
            self.bb.dynamic_obstacle = False
            self.node.get_logger().info("No Dynamic obj in "+str(self.thresh_m)+" meters.")

        ## 정적장애물 거리
        if self.bb.static_distance <= self.estop_thresh_m:
            self.bb.static_obstacle = True
            self.node.get_logger().info("Estop : Static obj : "+str(self.bb.static_distance)+" meters.")
            return Status.SUCCESS
        # 주변에 7m 내부에 장애물이 있으면 그 거리를 토픽으로 보내고, SUCCESS
        elif self.bb.static_distance <= self.thresh_m:
            self.bb.static_obstacle = True
            self.node.get_logger().info("Static obj : "+str(self.bb.static_distance)+" meters.")
        else:
            self.bb.static_obstacle = False
            self.node.get_logger().info("No Static obj in "+str(self.thresh_m)+" meters.")

        # dynamic, static 중에 우선순위 정하기
        if self.bb.dynamic_obstacle==False and self.bb.static_obstacle:
            self.prioritize_dynamic_flag = False
        else:   # 동적장애물이 유효사거리 내에 있다면 무조건 동적장애물을 우선으로 고려
            self.prioritize_dynamic_flag = True

        # dynamic 유효, static 유효, obj_flag 메시지에 담아서 송신(10Hz)
        msg = PointStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x = float(self.bb.dynamic_obstacle)  # 파이썬은 float은 무조건 float64
        msg.point.y = float(self.bb.static_obstacle)
        msg.point.z = float(self.prioritize_dynamic_flag)   # True(1.0)면 dynamic, False면 0.0 static.
        self.publish_flag.publish(msg)

        return Status.FAILURE   # FAILURE되면 seq째로 넘어가고 바로 selectpath로 감.
    
        

        # 이 아래는 내 차 기준 상대적 위치. 활용 예정.
        # cy = math.cos(-self.ego_yaw)
        # sy = math.cos(-self.ego_yaw)
        # x_rel = cy*dx - sy*dy   # 장애물의 내 차 기준 상대적 위치 앞(+), 뒤(-)
        # y_rel = sy*dx - cy*dy   # 장애물의 내 차 기준 상대적 위치 좌(-), 우(+)

        # # 거리 안보내고 바로 판단하기로 했음. 그래야 사이 딜레이가 안걸리니까
        # msg = Float64()
        # msg.data = dist
        # self.pub_dist.publish(msg)
        # self.node.get_logger().info("Distance topic published")
        # return Status.SUCCESS
        # Path가 3~4초 이후의 미래의 위치에서의 Path를 뽑는 방식이라,
        

# # === LPP 시도 === 로컬 플래너에서 패스를 다 주기로 해서 난 그냥 path를 골라 쓰기만 하면 돼서 LPP 함수를 그냥 없앴음.
# class UseLPP(py_trees.behaviour.Behaviour):
#     def __init__(self, node, name="Use LPP"):
#         super().__init__(name)
#         self.node = node
#         self.local_path_msg = None
#         qos = QoSProfile(
#             depth=1,
#             reliability=ReliabilityPolicy.RELIABLE,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL
#         )
#         self.pub_path = self.node.create_publisher(Path, 'path', qos)   # path로 로컬과 글로벌패스 다 통일
#         self.node.create_subscription(Path, 'local_path', self.cb_localpath, 10)
        
#     def cb_localpath(self,msg:Path):
#         self.local_path_msg = msg
        
#     def update(self):
#         # TODO: LPP 경로 수신 및 퍼블리시. 받은거 그대로 보냄.
#         if self.local_path_msg is None:
#             self.node.get_logger().warn("Local Path don't come yet.")
#             return Status.FAILURE
#         self.pub_path.publish(self.local_path_msg)
#         self.node.get_logger().info("Local Path Publishing")
#         return Status.SUCCESS   # Status.SUCCESS를 하는 이유는 BT에서 각 노드는 SUCCESS,RUNNING,FAILURE만을 반환하기 때문입니다.

# === Path 선택 === (이때 추월 경로 생성 안되거나 유효 장애물이 없는 경우는 일반으로 주행, 장애물이 근처에 오면 slow로 작동)
# slow의 경우는 z값을 내가 바꾸는 거로 했음. 1/10으로 일단 설정하자.
class SelectPath(py_trees.behaviour.Behaviour):
    def __init__(self, node, slow:bool=False,name="SelectPath"):    # slow일단 안씀 selectpath를 다시 usegpp이런식으로 나누면 그때 사용가능할듯-> 이렇게 수정해야 BT의 장점이 나와
        super().__init__(name)
        self.node = node
        # self.slow = slow    # 일단 느리게 가는거 설정 X(default는 일반속도)
        self.global_path_msg: Path | None=None
        self.local_path_msg: Path | None=None
        self.scaled_path: Path | None=None
        self.global_last_stamp = None
        self.local_last_stamp = None

        self.bb = BBClient(name="SelectPath")
        self.bb.register_key("dynamic_obstacle", access=Access.READ)
        self.bb.register_key("static_obstacle", access=Access.READ)
        self.bb.register_key(key="dynamic_distance", access=Access.READ)
        self.bb.register_key(key="static_distance", access=Access.READ)
        self.bb.register_key(key="overtake_flag", access=Access.WRITE)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.node.create_subscription(Path, 'global_path',self.cb_globalpath, qos)   # global path : ref_path에서 긴급하게 속도 늦출때만 사용. Path가 나오지 않는 경우를 대비해서.
        self.node.create_subscription(Path, 'Path', self.cb_localpath, qos)   # Path : global path와 기본은 같은데, 추월 가능 상황일 때 추월 가능하다고 알려줌.
        # 위 세 개를 custom message로 묶는게... 시간 동시에 하는게 쉽지 않은듯함.
        self.pub_path = self.node.create_publisher(Path, 'selected_path', qos)  # MPC에서 최종적으로 전달하는 path
        self.pub_state = self.node.create_publisher(String, 'state', 10)     # 현재 state를 publish하는것.
        

    def cb_globalpath(self,msg:Path):
        self.global_path_msg = msg
        print("callback global path")
        # dynamic_distance랑 static_distance가 안오는 상태에서는 이 둘이 한번도 저장이 안된 상태라면(장애물을 안봐서)
        if (self.bb.dynamic_obstacle == True and self.bb.dynamic_distance <= 1) or (self.bb.static_obstacle == True and self.bb.static_distance <= 1) and self.bb.overtake_flag == 0:
            # 2. stamp 바뀌었으면 1/10 복사본을 한번만 갱신
            stamp = (self.global_path_msg.header.stamp.sec, self.global_path_msg.header.stamp.nanosec)
            if stamp != self.global_last_stamp:
                self.global_last_stamp = stamp
                sp = deepcopy(self.global_path_msg)
                for ps in sp.poses:
                    ps.pose.position.z = ps.pose.position.z / 2.0
                self.scaled_path = sp
                # 느린 속도 path는 scaled path로 따로 저장. 원본 Path를 local_path_msg에 계속 저장중임.
    #     # 2. stamp 바뀌었으면 1/10 복사본을 한번만 갱신
    #     stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
    #     if stamp != self.last_stamp:
    #         self.last_stamp = stamp
    #         sp = deepcopy(msg)
    #         for ps in sp.poses:
    #             ps.pose.position.z = ps.pose.position.z / 10.0
    #         self.scaled_path = sp
    #         self.pub_path.publish(self.scaled_path)
    #         self.node.get_logger().info("slow global path published!")

    # def cb_distance(self, msg:Float64):
    #     self.distance = msg.data

    def cb_localpath(self,msg:Path):    # Local Path 생성 안될때 & 기존 global path 사용할때: Path에 orientation.x를 0으로 준대.
                                        # Local Path 생성 될때는 Path의 orientation.x를 1로 주는거고.
                                        # 그러면 global_path 자체를 쓸 필요가 없어진거네?
        self.local_path_msg = msg
        self.bb.overtake_flag = self.local_path_msg.poses[0].pose.orientation.z  # orientation의 x에 Path가 추월 경로인지 아닌지를 담아주기로 했음.
        # overtake_flag가 bool이 아니라 0:globalpath로 주행, 1:추월X인 localpath로 주행, 2:추월O인 localpath로 주행
        
        # 추월 X, 1m이내에서는 느리게 주행. 절반의 속도. 일단 생성은 계속 해두자. 필요할 때 바로 쓰도록. 차피 비상정지는 별개라.
        if (self.bb.dynamic_obstacle == True and self.bb.dynamic_distance <= 1) or (self.bb.static_obstacle == True and self.bb.static_distance <= 1) and self.bb.overtake_flag == 1:
            # 2. stamp 바뀌었으면 1/10 복사본을 한번만 갱신
            stamp = (self.local_path_msg.header.stamp.sec, self.local_path_msg.header.stamp.nanosec)
            if stamp != self.local_last_stamp:
                self.local_last_stamp = stamp
                sp = deepcopy(self.local_path_msg)
                for ps in sp.poses:
                    ps.pose.position.z = ps.pose.position.z / 2.0
                self.scaled_path = sp
                # 느린 속도 path는 scaled path로 따로 저장. 원본 Path를 local_path_msg에 계속 저장중임.
                
        


    # path publish를 얘가 안한대 Lpp에서 바로 가고 난 기준을 정하래. 추월유무를 정해야 그거에 맞게 Lpp를 짜주겟대
    def update(self):
        # TODO: GPP 경로 + 감속 주행 명령 퍼블리시
        self.node.get_logger().info("SelectPath Activate")
        if self.global_path_msg is None:
            self.node.get_logger().warn("/global_path don't come yet.")
            return Status.FAILURE
        if self.local_path_msg is None:
            self.node.get_logger().warn("/Path don't come yet.")
            return Status.FAILURE
        # 일단 비상'정지'는 CheckObstacle에서 처리했음. E-stop으로 가도록.
        # 비상정지의 thresh_m를 속도에 따라 조정해야할수도, state에 대해서도.
        # 추월O => 내가 가는 방향으로의, 30cm 거리.
        # 추월X => 내 속도 기준으로 바꾸면 좋은데, 일단은 50cm로.
        # ---------------------------------------------------------
        # 이제 비상정지 말고 그냥 느리게 가야하는 경우는?(estop_thresh_m랑 thresh_m의 중간. 1m정도에서는 느리게 가야하지 않나?)
        # 추월X에 유효장애물(추월할) 없으면 속도 느리게 할 필요 없음.
        # 근데 추월X인데 유효장애물 있다? 그러면 비상거리보다 크고 일정 거리 이내 일때는 속도를 늦춰서 진행(1/2정도로)
        
        if  self.bb.dynamic_distance <= 1 or self.bb.static_distance <= 1 and self.bb.overtake_flag!=2:   # SLOW의 경우
            self.pub_path.publish(self.scaled_path)
            self.node.get_logger().info("추월 X, slow path published! (추월 X, 1m 이내)")
        elif self.bb.dynamic_distance < 7 or self.bb.static_distance < 7 and self.bb.overtake_flag!=2:    # 추월X, SLOW 아닌 경우
            self.pub_path.publish(self.local_path_msg)
            self.node.get_logger().info("추월 X, normal speed Path published (추월 X)")
        elif self.bb.overtake_flag != 2:   # 유효사거리에 장애물도 없고 추월x 인 경우.
            self.pub_path.publish(self.local_path_msg)
            self.node.get_logger().info("추월X Path published")
        # 이 위 elif 두개를 합쳐도 될듯 거리를 안재고 그냥 overtake_flag로만.
        
        else:   # 추월O 인 경우.
            self.pub_path.publish(self.local_path_msg)
            self.node.get_logger().info("추월O Path published")
            # 그럼 지금은 추월O인 경우의 SLOW scaled path는 만들지 않는거네요?! => 네. 일단은 장애물 탐지쪽에서 비상정지합니다.
            # 그냥 가까우면 비상정지도 좋아보이지 않는데, 마인드는 50cm니까 그 이하로 추월하면 위험하다 로 가서 이렇게 진행함.
            # 이건 좀 그렇고 새로 만들자. 추월할 때는 추월하러 가는 경로 앞에 있다면 이런 식으로
        
        # st=String()
        # st.data = "Using LocalPath()"
        # self.pub_state.publish(st)
        return Status.SUCCESS
