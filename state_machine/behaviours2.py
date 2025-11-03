#!/usr/bin/env python3
import math, rclpy
import py_trees
from py_trees.common import Access, Status
from py_trees.blackboard import Client as BBClient
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from copy import deepcopy

def yaw_from_quat(q):
    # quaternion -> yaw
    return math.atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z)
    )

def point_to_path_min_dist(px, py, path_msg:Path):
    """
    static obstacle (px,py)가 global_path 상에 얼마나 가까운지(m).
    path_msg가 아직 None이면 None 리턴.
    """
    if path_msg is None:
        return None
    dmin = float("inf")
    for ps in path_msg.poses:
        gx = ps.pose.position.x
        gy = ps.pose.position.y
        d = math.hypot(px-gx, py-gy)
        if d < dmin:
            dmin = d
    return dmin

def is_in_front_180(rel_x, rel_y, ego_yaw, half_angle_deg:float=100.0)->bool:
    """
    ego_yaw 방향 기준으로 장애물이 앞쪽 200도 안인지 확인.
    앞쪽 반구 조건: cos(theta) >= 0  <=> 내 진행방향 단위벡터와 장애물 방향 단위벡터 내적 >= 0
    """
    dist = math.hypot(rel_x, rel_y)
    if dist == 0.0:
        return True  # 바로 내 위치라면 그냥 앞이라고 처리
    fx = math.cos(ego_yaw)
    fy = math.sin(ego_yaw)
    ox = rel_x/dist
    oy = rel_y/dist
    dot = fx*ox + fy*oy
    
    cos_limit = math.cos(math.radians(half_angle_deg))
    return dot >= cos_limit

# 장애물 감지 자체는 state machine에서 하지 않을것으로 생각됩니다.
# 장애물이 감지되었다는 결과를 토픽으로 받을 것으로 생각하고 있습니다.
# # === 장애물 감지 노드입니다. ===
class CheckObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, node, thresh_m=10.0):   # pytrees에서는 모든 노드가 내부적으로 이름을 가진대. 이름 설정.
        super().__init__("Obstacle <= "+str(thresh_m)+"m?")
        self.node = node

        # 제일 처음엔 초기화.
        self.ego_timestamp = 0.0    # 내 차
        self.ego_x = None
        self.ego_y = None
        self.ego_yaw = 0.0
        self.ego_vel = (0.0, 0.0)
        self.dynamic_timestamp = 0.0    # 상대 차(dynamic)
        self.dynamic_x = None
        self.dynamic_y = None
        self.dynamic_Vel = (0.0,0.0)
        # self.dynamic_obstacle = False   # dynamic obstacle Flag
        self.static_timestamp = 0.0 # 정적 장애물 (반드시 1개)
        self.static_x = None
        self.static_y = None
        # self.static_obstacle = False
        self.thresh_m = thresh_m    # 유효장애물 인식 사거리
        self.estop_thresh_m = 0.5   # 비상정지 인식 사거리 (50cm)
        self.fresh = 0.3            # 신호 지연 기준 시간 ------------------------------------> 지금은 3으로 일부러 늘려놨는데 0.5로 설정해!! 실제에서는!!
        self.prioritize_dynamic_flag = True   # True(1.0) dynamic, False(0)이 static을 우선으로 설정.
        self.local_path_msg: Path | None =None


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
        
        self.node.create_subscription(Path, '/Path', self.cb_path, 1)
        self.node.create_subscription(Odometry, 'odom', self.cb_ego_odom, 10)
        self.node.create_subscription(Odometry, '/dynamic_obstacle', self.cb_dynamic_odom, 20)
        self.node.create_subscription(PointStamped, '/static_obstacle', self.cb_static_odom, 10)

        self.publish_flag = self.node.create_publisher(PointStamped, '/obj_flag', 1)
    
    # 현재 시각 확인
    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9
    
    def cb_path(self, msg:Path):
        self.local_path_msg = msg

    # 내 위치 callback 함수
    def cb_ego_odom(self,msg:Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ego_x = p.x
        self.ego_y = p.y
        self.ego_yaw = yaw_from_quat(q)
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

        dynamic_dx = self.dynamic_x - self.ego_x
        dynamic_dy = self.dynamic_y - self.ego_y
        self.dynamic_dist = math.hypot(dynamic_dx,dynamic_dy) # 동적장애물과의 거리
        self.bb.dynamic_distance = self.dynamic_dist
        self.node.get_logger().info("Dynamic obj :: "+str(self.bb.dynamic_distance)+" meters.")

    # 정적 장애물 (static) 위치 callback 함수
    # 정적 장애물은 속도가 없겠지. 치고 가지 않는 이상.
    def cb_static_odom(self, msg: PointStamped):
        p = msg.point
        self.static_x = p.x
        self.static_y = p.y
        self.static_timestamp = self.now()

        static_dx = self.static_x - self.ego_x
        static_dy = self.static_y - self.ego_y
        self.static_dist = math.hypot(static_dx, static_dy)
        self.bb.static_distance = self.static_dist
        self.node.get_logger().info("Static obj :: "+str(self.bb.static_distance)+" meters.")

    def update(self):
        now = self.now()  
        if (self.ego_timestamp == 0.0 or (now - self.ego_timestamp) > self.fresh):
            self.node.get_logger().warn("/odom can't subscribable. CheckObstacle Failure.")
            return Status.FAILURE
        if (self.dynamic_timestamp == 0.0 or (now - self.dynamic_timestamp) > self.fresh):
            self.node.get_logger().warn("/dynamic_obstacle can't subscribable. CheckObstacle Failure.")
            self.bb.dynamic_obstacle = False
            self.bb.dynamic_distance = 100.0
            self.dynamic_dist = 100.0
            # return Status.FAILURE 실제로는 주석 지워라
        if (self.static_timestamp == 0.0 or (now - self.static_timestamp) > self.fresh):
            self.node.get_logger().warn("/static_obstacle can't subscribable. CheckObstacle Failure.")
            self.bb.static_obstacle = False
            self.bb.static_distance = 100.0
            self.static_dist = 100.0
            # return Status.FAILURE  실제로는 주석 지워라
        
        # 전방 180도, global path 인접 여부 확인해서 flag 결정
        dyn_flag = False
        st_flag = False
        
        # ------------------------
        # Dynamic obstacle 판정
        # ------------------------
        if (self.dynamic_x is not None and self.dynamic_y is not None and
            self.ego_x     is not None and self.ego_y     is not None):

            dx = self.dynamic_x - self.ego_x
            dy = self.dynamic_y - self.ego_y

            in_front = is_in_front_180(dx, dy, self.ego_yaw, half_angle_deg=100.0)          # ### ✅ 전방 200도
            close_enough = (self.dynamic_dist <= self.thresh_m)       # 거리 7m 이내

            # ### ✅ local_path 근처인지 확인 (1.0m 이내)
            path_d = point_to_path_min_dist(self.dynamic_x, self.dynamic_y, self.local_path_msg)
            if dyn_flag == False and (path_d is not None and path_d <= 1.0):
                close_to_path = True
            elif dyn_flag == True and (path_d is not None and path_d <= 1.2):
                close_to_path = True
            else:
                close_to_path = False 

            if in_front and close_enough and close_to_path:
                dyn_flag = True
                self.node.get_logger().info(
                    f"Dynamic VALID: dist={self.dynamic_dist:.2f}m, front180={in_front}"
                )
            else:
                if not in_front:
                    self.node.get_logger().info("Dynamic detected but NOT in front 180deg.")
                elif not close_enough:
                    self.node.get_logger().info(
                        f"Dynamic farther than {self.thresh_m}m."
                    )
                elif not close_to_path:
                    if path_d is None:
                        self.node.get_logger().info(
                            "Dynamic in front but local path not ready -> ignore static."
                        )
                    else:
                        self.node.get_logger().info(
                            f"Dynamic in front but not blocking path (dist to path={path_d:.2f}m >1.0m)"
                        )

        # ------------------------
        # Static obstacle 판정
        # ------------------------
        if (self.static_x is not None and self.static_y is not None and
            self.ego_x    is not None and self.ego_y    is not None):
            
            sx = self.static_x - self.ego_x
            sy = self.static_y - self.ego_y

            in_front = is_in_front_180(sx, sy, self.ego_yaw, half_angle_deg=100.0)          # ### ✅ 전방 200도
            close_enough = (self.static_dist <= self.thresh_m)        # 거리 7m 이내

            # ### ✅ local_path 근처인지 확인 (0.5m 이내)
            path_d = point_to_path_min_dist(self.static_x, self.static_y, self.local_path_msg)
            if st_flag == False and (path_d is not None and path_d <= 0.5):
                close_to_path = True
            elif st_flag == True and (path_d is not None and path_d <= 0.6):
                close_to_path = True
            else:
                close_to_path = False 
	
            if in_front and close_enough and close_to_path:
                st_flag = True
                self.node.get_logger().info(
                    f"Static VALID: dist={self.static_dist:.2f}m, path_d={path_d:.2f}m"
                )
            else:
                if not in_front:
                    self.node.get_logger().info("Static detected but NOT in front 180deg.")
                elif not close_enough:
                    self.node.get_logger().info(
                        f"Static farther than {self.thresh_m}m."
                    )
                elif not close_to_path:
                    if path_d is None:
                        self.node.get_logger().info(
                            "Static in front but local path not ready -> ignore static."
                        )
                    else:
                        self.node.get_logger().info(
                            f"Static in front but not blocking path (dist to path={path_d:.2f}m >0.8m)"
                        )

        # Blackboard 갱신
        self.bb.dynamic_obstacle = dyn_flag
        self.bb.static_obstacle  = st_flag

        # ## 동적장애물 거리
        # # 비상정지의 경우에만 바로 SUCCESS를 뱉음. 그러면 다음 노드인 E-stop을 하게됨.
        # # 실제로 E-Stop은 SelectPath에서 다 정의해뒀다. => 유효 사거리 안에 있을때 True로 표시만 해주면 됨.
        # # 주변에 7m 내부에 장애물이 있으면 dynamic obstacle flag True로.
        # if self.bb.dynamic_distance <= self.thresh_m:
        #     self.bb.dynamic_obstacle = True
        #     self.node.get_logger().info("Dynamic obj : "+str(self.bb.dynamic_distance)+" meters.")
        # else:
        #     self.bb.dynamic_obstacle = False
        #     self.node.get_logger().info("No Dynamic obj in "+str(self.thresh_m)+" meters.")

        # ## 정적장애물 거리
        # # 주변에 7m 내부에 장애물이 있으면 그 거리를 토픽으로 보내고, SUCCESS
        # if self.bb.static_distance <= self.thresh_m:
        #     self.bb.static_obstacle = True
        #     self.node.get_logger().info("Static obj : "+str(self.bb.static_distance)+" meters.")
        # else:
        #     self.bb.static_obstacle = False
        #     self.node.get_logger().info("No Static obj in "+str(self.thresh_m)+" meters.")

        # dynamic, static 중에 우선순위 정하기
        # if self.bb.dynamic_obstacle==False and self.bb.static_obstacle:
        #     self.prioritize_dynamic_flag = False
        # else:   # 동적장애물이 유효사거리 내에 있다면 무조건 동적장애물을 우선으로 고려
        #     self.prioritize_dynamic_flag = True
        #     self.node.get_logger().info("Prioritize Dynamic Obstacle")
        # 이거를 일단 static 회피가 가능한지 보기 위해서 우선순위를 다 static이 있다면 static으로 우선순위를 주도록함
        if self.bb.dynamic_obstacle==False and self.bb.static_obstacle:
            self.prioritize_dynamic_flag = False
        else:   # 동적장애물이 유효사거리 내에 있다면 무조건 동적장애물을 우선으로 고려
            self.prioritize_dynamic_flag = False
            # self.node.get_logger().info("Prioritize Dynamic Obstacle")

        # dynamic 유효, static 유효, obj_flag 메시지에 담아서 송신(10Hz)
        msg = PointStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x = float(self.bb.dynamic_obstacle)  # 파이썬은 float은 무조건 float64
        msg.point.y = float(self.bb.static_obstacle)
        msg.point.z = float(self.prioritize_dynamic_flag)   # True(1.0)면 dynamic, False면 0.0 static.
        self.publish_flag.publish(msg)

        return Status.FAILURE   # FAILURE되면 seq째로 넘어가고 바로 selectpath로 감.
    
        # 이 아래는 내 차 기준 상대적 위치.
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

# === Path 선택 === (이때 추월 경로 생성 안되거나 유효 장애물이 없는 경우는 일반으로 주행, 장애물이 근처에 오면 slow로 작동)
# slow의 경우는 z값을 내가 바꾸는 거로 했음. 1/10으로 일단 설정하자.
class SelectPath(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="SelectPath"):
        super().__init__(name)
        self.node = node
        self.local_path_msg: Path | None=None
        self.emergency_scaled_path: Path | None=None
        self.acc_scaled_path: Path | None=None
        self.local_last_stamp = None

        self.bb = BBClient(name="SelectPath")
        self.bb.register_key("dynamic_obstacle", access=Access.READ)
        self.bb.register_key("static_obstacle", access=Access.READ)
        self.bb.register_key(key="dynamic_distance", access=Access.READ)
        self.bb.register_key(key="static_distance", access=Access.READ)
        self.bb.register_key(key="overtake_flag", access=Access.WRITE)

        self.bb.overtake_flag = 0.0 # 일단 초기화

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.node.create_subscription(Path, 'Path', self.cb_localpath, 1)
        self.pub_path = self.node.create_publisher(Path, 'selected_path', 1)  # MPC에서 최종적으로 전달하는 path
        # 현재 state 를 토픽으로 publish 할까? info로 하지 말고.

    def path_scaler(self, path_msg:Path,divide:float) -> Path|None:
        if path_msg is None:
            return None
        sp = deepcopy(path_msg)
        for ps in sp.poses:
            ps.pose.position.z = ps.pose.position.z / float(divide)
        return sp

    def cb_localpath(self,msg:Path):
        self.local_path_msg = msg
        self.bb.overtake_flag = self.local_path_msg.poses[0].pose.orientation.z
        # overtake_flag가 bool이 아니라 0:globalpath로 주행, 1:static obs 회피, 2:Dynamic obs 추월, 3: Adaptive Cruise Control 
        # stamp 기준으로 경로가 바뀌었으면 1/10, 1/2 복사본을 한번만 갱신
        stamp = (self.local_path_msg.header.stamp.sec, self.local_path_msg.header.stamp.nanosec)
        if stamp != self.local_last_stamp:
            self.local_last_stamp = stamp
            self.emergency_scaled_path = self.path_scaler(msg,10.0)
            self.acc_scaled_path = self.path_scaler(msg, 2.0)

    def update(self):
        # self.node.get_logger().info("SelectPath Activate")
        if self.local_path_msg is None:
            self.node.get_logger().warn("/Path don't come yet.")
            return Status.FAILURE
        
        # Emergency mode
        if  (self.bb.dynamic_distance <= 1 or self.bb.static_distance <= 1) and self.bb.overtake_flag!=2:   # Emergency!!!
            if self.emergency_scaled_path is not None:
                self.pub_path.publish(self.emergency_scaled_path)
                self.node.get_logger().info("Emergency!! velocity/10 path published! (추월X, Emergency)")
                return Status.SUCCESS
            else:
                self.pub_path.publish(self.local_path_msg)
                self.node.get_logger().info("추월X, Emergency 인데 emergency_scaled_path가 아직 갱신이 안돼서 로컬 패스를 퍼블리쉬함!!")
                return Status.SUCCESS
        elif (self.bb.dynamic_distance <= 0.5 or self.bb.static_distance <= 1) and self.bb.overtake_flag == 2:	# Emergency!!! 추월모드에서는 동적장애물과의 거리가 0.5로 줄어듦. (double thresholding)
            if self.emergency_scaled_path is not None:
                self.pub_path.publish(self.emergency_scaled_path)
                self.node.get_logger().info("Emergency!! velocity/10 path published! (추월O, Emergency)")
                return Status.SUCCESS
            else:
                self.pub_path.publish(self.local_path_msg)
                self.node.get_logger().info("추월O, Emergency 인데 emergency_scaled_path가 아직 갱신이 안돼서 로컬 패스를 퍼블리쉬함!!")
                return Status.SUCCESS

        # Adaptive Cruise Control
        elif (self.bb.dynamic_distance < 2.5) and self.bb.overtake_flag==4:    # ACC할때 static distance는 고려할 필요 없음. 비상상황인 경우는 이미 위에서 정의해둠.
            if self.acc_scaled_path is not None:
                self.pub_path.publish(self.acc_scaled_path)
                self.node.get_logger().info("ACC mode, distance < 2.5m")
                return Status.SUCCESS
            else:
                self.pub_path.publish(self.local_path_msg)
                self.node.get_logger().info("ACC mode, distance < 2.5m인데 acc_scaled_path가 아직 갱신이 안돼서 로컬 패스를 퍼블리쉬")
                return Status.SUCCESS

        # 이 아래는 결국에 로컬 패스 주행이라 합쳐도 되는데 모드 체크를 위해서 이렇게 두개로 나눠놨습니다.
        elif self.bb.dynamic_distance < 10 or self.bb.static_distance < 10:
            self.pub_path.publish(self.local_path_msg)
            # 현재 모드 뭔지 체크용
            if self.bb.overtake_flag == 4:
                self.node.get_logger().info("ACC mode, 2.5m < distance < 10m")
            elif self.bb.overtake_flag == 0:
                self.node.get_logger().info("global path 추종, 거리 10m 이내.")
            elif self.bb.overtake_flag == 1:
                self.node.get_logger().info("Static 회피 모드, 거리 10m 이내.")
            elif self.bb.overtake_flag == 2:
                self.node.get_logger().info("Dynamic 추월 모드, 거리 10m 이내.")
            else:
                self.node.get_logger().warn(f"unknown overtake_flag!!! {self.bb.overtake_flag} (거리 10m 이내)")
            return Status.SUCCESS

        else:   # 주변에 장애물 없는 경우, Local path에서 0을 줄것. 쨋든 나는 로컬패스 주는대로 주행하면 됨. 아직은 품질 평가 X.
            self.pub_path.publish(self.local_path_msg)
            # 이건 현재 모드 체크용
            if self.bb.overtake_flag == 4:
                self.node.get_logger().info("ACC mode, 유효 장애물 X")
            elif self.bb.overtake_flag == 0:
                self.node.get_logger().info("global path 추종, 유효 장애물 X")
            elif self.bb.overtake_flag == 1:
                self.node.get_logger().info("Static 회피 모드, 유효 장애물 X")
            elif self.bb.overtake_flag == 2:
                self.node.get_logger().info("Dynamic 추월 모드, 유효 장애물 X")
            else:
                self.node.get_logger().warn(f"unknown overtake_flag!!! {self.bb.overtake_flag} (유효 장애물 X)")
            return Status.SUCCESS    
        # return Status.SUCCESS , 위에 각 케이스에 모두 써뒀음.
