import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan

# [2026-08-20] 초음파(sensor_msgs/Range) 안전장치를 붙였다가 완전히 걷어냈다 — 범퍼
# 네 모서리 4개는 이 환경 rclpy/rmw가 같은 메시지 타입을 한 노드에 2개 이상 구독하면
# 'Unable to convert call argument to Python object'로 깨지는 게 확인돼 1개(전방)로
# 줄였는데도, 그 예외가 한 번이라도 발생하면 이 노드의 콜백 처리 전체가 사실상
# 멈춰버리는(프로세스는 안 죽지만 위치 구독 콜백/도킹 전환이 다시는 안 불림) 훨씬 심각한
# 재현을 확인했다(2026-08-20). 안전장치 하나 때문에 미션 전체가 죽는 게 더 위험하다고
# 판단해 초음파 관련 코드를 전부 제거하고 urdf/xycar.urdf의 초음파 센서도 되돌렸다.
# 다시 시도한다면 이 환경의 rclpy/rmw 자체를 먼저 점검할 것 — 애플리케이션 코드
# 문제가 아니었다(최소 재현 스크립트로 격리 확인됨).

# navigate_to_pose 목표가 거부되는 경우가 있다 — AMCL이 /initialpose를 막 처리한 직후
# global_costmap이 "TF extrapolation into the past"로 잠깐(<1초) map->base_link 조회에
# 실패하는 스타트업 과도현상이 실측 확인됨(2026-08-19, Gazebo use_sim_time 환경에서
# /clock이 막 안정화되는 시점과 겹침). 이 순간의 거부는 시스템이 곧 회복되므로, 완전
# 실패로 처리하지 말고 잠깐 뒤 재전송한다.
GOAL_RETRY_MAX = 20
GOAL_RETRY_DELAY_SEC = 1.0


def yaw_to_quaternion(yaw: float):
    """2D yaw(rad) -> (x, y, z, w) quaternion."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ParkingNavigator(Node):

    def __init__(self):
        super().__init__('parking_navigator')

        self.declare_parameter('parking_zone', 'A')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('set_initial_pose', True)
        # /initialpose 구독자 연결을 기다리는 최대 시간이면서, 동시에 AMCL이 그 초기
        # pose를 실제로 처리했는지(/amcl_pose 수신) 확인하는 최대 대기시간으로도 재사용됨
        # (아래 publish_initial_pose_until_amcl_ready 참고, 1초 간격으로 재발행하며
        # 기다림). 45초는 상한선일 뿐 실제로는 AMCL이 처리하는 즉시 통과하므로, 정상
        # 상황(실차)에서 이 값을 키운다고 부팅이 느려지지 않는다. Gazebo+Nav2를 동시에
        # 돌리는 CPU 부하 큰 환경에서 DDS transient-local 전달이 20초 넘게 걸리는 경우도
        # 실측했다(2026-08-19).
        self.declare_parameter('initial_pose_wait_sec', 45.0)

        self.declare_parameter('start_x', 1.8)
        self.declare_parameter('start_y', 0.9)
        self.declare_parameter('start_yaw', 3.14)

        self.declare_parameter('zone_a_x', 0.0)
        self.declare_parameter('zone_a_y', 4.2)
        self.declare_parameter('zone_a_yaw', 0.0)

        self.declare_parameter('zone_b_x', 2.1)
        self.declare_parameter('zone_b_y', 3.3)
        self.declare_parameter('zone_b_yaw', -1.57)

        # ── 목표 근처 전용 "정밀 접근"(docking) — 2026-08-20 ──
        #   README §6-2/§6-4에 이미 적혀있던 문제: 주차 목표가 벽에서 0.5m 안쪽이라
        #   로봇 풋프린트(inscribed radius 0.363m) 기준 여유가 10cm 안팎뿐이라, Nav2의
        #   전역 NavFn 플래너가 tolerance=0.5 안에서도 유효 경로를 못 찾고
        #   (GridBased failed) behavior_server의 spin/backup 복구도 같은 이유로 실패하는
        #   무한루프에 빠졌다(실측 확인, run1/run4). 이건 costmap 파라미터를 더 튜닝해서
        #   풀 문제가 아니라 — 목표 자체가 costmap 기준으로는 "계획 불가능"한 지점이라는
        #   뜻이라, 그 구간만 costmap을 아예 거치지 않는 별도 로직으로 우회한다.
        #   목표까지 direct-line 거리가 final_approach_radius 이내로 들어오면
        #   navigate_to_pose 액션을 취소하고 AMCL pose 피드백만으로 직접 /cmd_vel을
        #   publish해 마무리한다(_start_docking()/_docking_control_loop() 참고). 이
        #   구간은 로봇이 이미 Nav2 경로를 따라 정상 주행해 도달한 지점이라 큰 장애물
        #   회피가 필요 없다는 전제 — 완전히 새로운 경로탐색이 필요한 상황(예: 처음부터
        #   막혀서 아예 접근을 못 한 경우)은 이 우회로 해결되지 않는다.
        # [2026-08-20] 1.0m이었다가, 실측 결과 로봇이 정확히 1.01m에서 Nav2가 경로를
        # 못 찾고 멈춰버리는 걸 확인함(도킹 반경에 살짝 못 미쳐 못 넘어감) — 이 근처는
        # 이미 목표 바로 앞 좁은 구간이라 Nav2가 어차피 못 뚫는 지점이므로, 도킹 전환을
        # 더 일찍 시켜서 그 구간 자체를 건너뛴다.
        # [2026-08-20] 1.5m이었다가, 실측 결과 "목표 근처 좁은 구역"(대략 x:0.5~1.5,
        # y:2.5~4, 벽까지 여유 0.22~0.30m대)이 1.5m 반경보다 넓게 걸쳐있어서 Nav2가
        # 그 구역 진입 전에 도킹으로 안 넘어가고 여전히 SmacPlannerHybrid+RPP로 그
        # 구역을 지나려다 "Starting point in lethal space"/충돌로 반복 실패하는 걸
        # 확인함. 이 구역 전체를 도킹(AMCL 직접제어, costmap 충돌검사 없음)이 커버하도록
        # 2.5m로 확장 — 대신 이 구간엔 정적 지도 외 다른 장애물이 없다는 전제.
        # [2026-08-20] 0.0(도킹 비활성화)으로 테스트해봤으나 Nav2 단독으로도 같은
        # 지점("Starting point in lethal space")에서 똑같이 막히는 걸 확인함 — 도킹
        # 모드가 문제였던 게 아니라 그 구역 자체가 둘 다에게 어려운 지점이었다.
        # 도킹이 없는 것보다는 있는 쪽이(불완전하더라도) 나으므로 2.5m로 원복.
        # [2026-08-22] 팀원 커밋과의 머지 충돌을 해소하며 SmacPlannerHybrid(Dubin)+
        # footprint margin 0.07m+inflation cost_scaling_factor=8.0 조합이 전부 반영된
        # "재정리된" nav2_params.yaml로도 도킹 진입 지점(약 (1.38,2.18), 목표까지
        # 2.45m)에서 여전히 정밀 접근이 막히는 걸 재확인함(run35). 이 조합으로
        # "도킹 없이 순수 Nav2"를 테스트해본 적은 없었으므로, 그 구간까지 Nav2가
        # 담당하도록 반경을 0.6m로 대폭 축소 — 도킹은 목표 바로 앞(벽까지 10cm 안팎이라
        # costmap이 원천적으로 계획 불가능한) 마지막 구간만 커버하게 한다.
        self.declare_parameter('final_approach_radius', 0.6)
        self.declare_parameter('final_approach_speed', 0.15)
        # nav2_params.yaml의 goal_checker(xy_goal_tolerance/yaw_goal_tolerance)와
        # 동일 값을 기본값으로 두되, 이 노드만 따로 튜닝할 수 있게 별도 파라미터로 뺌.
        self.declare_parameter('docking_xy_tolerance', 0.05)
        self.declare_parameter('docking_yaw_tolerance', 0.05)
        # [2026-08-20] final_approach_radius를 2.5m로 늘려 도킹이 더 긴 구간을
        # 커버해야 하므로, 그만큼 시간 여유도 같이 늘림(대략 2.5m/0.15m/s ≈ 17s +
        # 헤딩보정 여유).
        self.declare_parameter('docking_timeout_sec', 40.0)
        self.declare_parameter('docking_yaw_kp', 1.5)

        self._frame_id = self.get_parameter('goal_frame_id').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._result_future = None
        self._done = False
        self._success = False
        self._zone = None
        self._retry_count = 0

        # ── 정밀 접근(docking) 상태 ──
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb_amcl_pose, 10)
        # [2026-08-20] 도킹 중 좌/우 근접 장애물을 보고 반대쪽으로 미는 조향 바이어스용
        # — 이 노드엔 LaserScan 구독이 이거 하나뿐이라 초음파(Range) 때 겪은 "같은
        # 메시지 타입 2개 이상 구독 시 크래시" 문제와 무관하다(위 모듈 docstring 참고).
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self._scan_msg = None
        self._cur_pose = None   # (x, y, yaw), _cb_amcl_pose가 계속 갱신
        self._goal_pose = None  # (x, y, yaw), send_goal()이 채움
        self._docking_active = False
        self._docking_timer = None
        self._docking_deadline = None
        self._docking_progress_pose = None
        self._docking_progress_t = None
        self._docking_backoff_until = None
        # [2026-08-22] target_heading 모드 고정용 — _start_docking()에서 1회만 정해지고
        # _docking_control_loop() 내내 그대로 유지된다(아래 해당 위치 주석 참고).
        self._docking_use_fixed_heading = True
        # ── K턴(큰 헤딩오차 전용) 상태 — _docking_kturn_control() 참고 ──
        self._docking_maneuver_active = False
        self._docking_maneuver_dir_sign = 1.0
        self._docking_maneuver_phase = 1.0
        self._docking_maneuver_phase_until = None

    def zone_pose(self, zone: str):
        zone = zone.upper()
        if zone == 'A':
            prefix = 'zone_a'
        elif zone == 'B':
            prefix = 'zone_b'
        else:
            raise ValueError(f"알 수 없는 주차영역 '{zone}' (A 또는 B만 지원)")
        x = self.get_parameter(f'{prefix}_x').value
        y = self.get_parameter(f'{prefix}_y').value
        yaw = self.get_parameter(f'{prefix}_yaw').value
        return x, y, yaw

    def _cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._cur_pose = (p.position.x, p.position.y, quaternion_to_yaw(p.orientation))
        # [2026-08-20] 원래 _maybe_start_docking()을 별도 0.2Hz 타이머로 돌렸는데,
        # 드물게 발생하는 spin_once RuntimeError(위 _spin_once_safe 주석 참고)를 한 번
        # 겪은 뒤로 도킹 전환이 영영 안 일어나는 사고가 재현됐다(_maybe_start_docking을
        # 원래 별도 타이머로 돌렸는데, 그 예외 이후로는 구독 콜백에서 직접 불러도 마찬가지로
        # 멈춘 걸 실측 확인함 — 타이머만의 문제가 아니라 예외 발생 시점부터 이 노드의
        # 콜백 처리 전체가 사실상 죽는 것으로 보임, 2026-08-20). 근본 원인은 이
        # 환경 자체의 rclpy/rmw 제약(위 ULTRASONIC 관련 주석 삭제분 참고)이라 이
        # 노드 코드로 완전히 막을 수는 없고, 최소한 발생 빈도를 낮추기 위해 초음파
        # Range 구독(예외를 유발하던 원인)을 완전히 제거했다.
        self._maybe_start_docking()

    def _cb_scan(self, msg: LaserScan):
        self._scan_msg = msg

    def _lidar_steer_bias(self, direction: float) -> float:
        """도킹 이동 방향(direction: 전진>0/후진<0) 쪽 반원에서 좌/우 중 더 가까운
        장애물을 찾아 반대쪽으로 미는 조향 바이어스(angular.z에 더할 값, rad/s)를
        반환한다. 가까운 게 없으면 0.0. K턴/원호 제어 둘 다에서 공용으로 쓴다 —
        "카메라로 왼쪽 장애물 보이면 오른쪽으로 꺾기"와 같은 발상을 라이다로 구현한
        것(이 파이프라인엔 카메라가 없음, 라이다+AMCL만 사용)."""
        if self._scan_msg is None:
            return 0.0
        msg = self._scan_msg
        CHECK_RANGE_M = 0.5
        CHECK_HALF_ANGLE = math.radians(70)
        BIAS_GAIN = 3.0
        # 후진 중이면(차량 뒤쪽이 진행방향) 반대쪽 반원을 봐야 한다.
        base_angle = 0.0 if direction >= 0 else math.pi
        left_min = float('inf')
        right_min = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (0.05 < r < CHECK_RANGE_M):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            rel = normalize_angle(angle - base_angle)
            if abs(rel) > CHECK_HALF_ANGLE:
                continue
            if rel > 0:
                left_min = min(left_min, r)
            else:
                right_min = min(right_min, r)
        if left_min == float('inf') and right_min == float('inf'):
            return 0.0
        # 더 가까운 쪽에서 멀어지는 방향으로 미는 조향 — REP103 부호규약(양수=반시계=좌회전)
        # 기준, 왼쪽이 더 가까우면 오른쪽(음수)으로, 오른쪽이 더 가까우면 왼쪽(양수)으로.
        if left_min < right_min:
            return -BIAS_GAIN * max(0.0, CHECK_RANGE_M - left_min)
        return BIAS_GAIN * max(0.0, CHECK_RANGE_M - right_min)

    def publish_initial_pose(self):
        x = self.get_parameter('start_x').value
        y = self.get_parameter('start_y').value
        yaw = self.get_parameter('start_yaw').value
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # AMCL 기본 초기 공분산과 유사한 값 (x, y, yaw만 낮게)
        cov = [0.0] * 36
        cov[0] = 0.25    # x
        cov[7] = 0.25    # y
        cov[35] = 0.06853892326654787  # yaw
        msg.pose.covariance = cov

        self.get_logger().info(
            f'초기 pose 발행: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self._initial_pose_pub.publish(msg)

    def publish_initial_pose_until_amcl_ready(self, timeout_sec):
        """/initialpose를 한 번만 발행하고 바로 목표를 보내면 AMCL이 그 메시지를 처리하기
        전이라 navigate_to_pose가 거부될 수 있다. RELIABLE+TRANSIENT_LOCAL이라 QoS 자체는
        맞지만(/initialpose 구독자 연결까지는 확인됨), 실제 메시지 전달·처리가 언제
        끝나는지는 보장이 없다 — Gazebo 시뮬레이션처럼 CPU 부하가 큰 환경에서는 이
        지연이 15초를 넘기고, 심하면 한 번의 발행이 아예 유실되는 경우도 실측 확인함
        (2026-08-19). 그래서 단발 발행 대신 /amcl_pose가 실제로 올 때까지 주기적으로
        재발행한다(RViz의 "2D Pose Estimate"를 여러 번 누르는 것과 동일한 대응 —
        Nav2 커뮤니티에서 흔히 쓰이는 우회법). 타임아웃 안에 못 받아도 마지막 시도로
        그냥 진행한다(AMCL이 이미 다른 경로로 초기화됐을 수도 있으므로).

        [2026-08-20] 원래 여기서 /amcl_pose에 임시 구독을 따로 만들었었는데, __init__의
        _cb_amcl_pose(도킹용 self._cur_pose 갱신)가 이미 같은 토픽을 구독 중이라 같은
        노드 안에 /amcl_pose 구독이 두 개 동시에 존재하는 상태였다. 초음파를 추가한 뒤
        spin_once()가 'Unable to convert call argument to Python object'로 죽는 문제가
        생겨서 이 중복이 원인인가 의심해 없앴는데, 실측 결과 중복을 없애도 같은 문제가
        똑같이 재현됐다 — 즉 원인이 아니었다(진짜 원인은 초음파 Range 구독 자체, 위
        모듈 docstring 참고. 초음파는 이후 완전히 제거함). 다만 이 정리 자체(임시 구독
        대신 이미 있는 self._cur_pose 재사용)는 더 단순하고 구독도 하나 줄어드는
        정당한 개선이라 초음파 제거와 무관하게 그대로 유지한다."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        republish_period_sec = 1.0
        next_republish = self.get_clock().now().nanoseconds

        self.publish_initial_pose()
        while self._cur_pose is None and self.get_clock().now().nanoseconds < deadline:
            self._spin_once_safe(timeout_sec=0.2)
            if self._cur_pose is None and self.get_clock().now().nanoseconds >= next_republish:
                self.publish_initial_pose()
                next_republish = self.get_clock().now().nanoseconds + int(republish_period_sec * 1e9)

        received = self._cur_pose is not None
        if received:
            self.get_logger().info('AMCL이 초기 pose를 처리함(/amcl_pose 수신 확인)')
        else:
            self.get_logger().warn(
                f'/amcl_pose를 {timeout_sec:.0f}초 안에 못 받음 — 그래도 목표 전송 시도')
        return received

    def send_goal(self, zone: str):
        self._zone = zone
        x, y, yaw = self.zone_pose(zone)
        self._goal_pose = (x, y, yaw)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"주차영역 {zone.upper()} 목표 전송: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'남은 거리: {remaining:.2f} m', throttle_duration_sec=2.0)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self._retry_count < GOAL_RETRY_MAX:
                self._retry_count += 1
                self.get_logger().warn(
                    f'목표가 거부됨(재시도 {self._retry_count}/{GOAL_RETRY_MAX}) — '
                    f'{GOAL_RETRY_DELAY_SEC:.0f}초 뒤 재전송')
                timer = self.create_timer(GOAL_RETRY_DELAY_SEC, lambda: self._retry_send_goal(timer))
            else:
                self.get_logger().error(f'목표가 {GOAL_RETRY_MAX}번 모두 거부되었습니다.')
                self._done = True
                self._success = False
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)
        # 목표가 정상 수락된 이후부터 _cb_amcl_pose()가 매 pose 갱신마다
        # _maybe_start_docking()을 호출해 정밀 접근 반경 진입을 확인한다(별도
        # 타이머는 안 씀 — 위 _cb_amcl_pose 주석 참고).

    def _retry_send_goal(self, timer):
        timer.cancel()
        self.send_goal(self._zone)

    def _maybe_start_docking(self):
        if self._docking_active or self._done:
            return
        if self._cur_pose is None or self._goal_pose is None:
            return
        cx, cy, _ = self._cur_pose
        gx, gy, _ = self._goal_pose
        dist = math.hypot(gx - cx, gy - cy)
        radius = self.get_parameter('final_approach_radius').value
        # [진단용, 2026-08-20] Nav2 feedback의 distance_remaining(경로 기준)과 AMCL
        # 직선거리가 크게 어긋나는 사례가 실측됨(전자 0.00m인데 후자 3m대) — 실제 궤적을
        # 눈으로 보기 위한 임시 로그. 원인 확인되면 제거할 것.
        self.get_logger().info(
            f'[진단] AMCL 직선거리={dist:.2f}m (cur=({cx:.2f},{cy:.2f}))', throttle_duration_sec=2.0)
        if dist <= radius:
            self._start_docking()

    def _start_docking(self):
        """Nav2 costmap 기반 계획을 벗어나, 목표까지 남은 짧은 구간을 AMCL pose
        피드백만으로 직접 /cmd_vel을 publish해서 마무리한다(위 __init__ 주석 참고).
        costmap 충돌검사를 아예 안 하므로(그게 이 우회로의 목적) 장애물 인식이
        전혀 없다 — _docking_control_loop()의 정지감지+후진탈출이 유일한 방어선."""
        self.get_logger().info('정밀 접근 반경 진입 — Nav2 목표 취소하고 직접 제어로 전환')
        self._docking_active = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        timeout_sec = self.get_parameter('docking_timeout_sec').value
        self._docking_deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        # ── 정지 감지 상태 ──
        # [2026-08-20] 도킹 루프엔 costmap 충돌검사가 없어서, 실제로 벽에 밀어붙인 채
        # 명령만 계속 나가는(바퀴는 헛돔) 상황이 40초 내내 지속되는 걸 실측 확인함
        # (헤딩 계산 자체는 문제 없었는데 물리적으로 못 움직이고 있었음). 일정 시간
        # 동안 실제 위치 변화가 없으면 "막혔다"로 보고 짧게 후진해서 빠져나온다.
        self._docking_progress_pose = self._cur_pose
        self._docking_progress_t = self.get_clock().now().nanoseconds
        self._docking_backoff_until = None
        self._docking_maneuver_active = False
        self._docking_maneuver_phase_until = None

        # [2026-08-22] target_heading 모드(점 추종 vs 고정 goal_yaw)를 매 틱 "현재" 거리로
        # 재판정하던 것을 도킹 진입 시점 1회로 고정. K턴은 설계상 전후진을 오가며 순간적으로
        # 목표에서 멀어질 수 있는데(정상 동작), 그때마다 dist가 close_radius를 다시 넘어
        # target_heading이 고정 goal_yaw -> 점 추종(atan2)으로 도로 튀었다 — 이 근접 거리대의
        # 점 추종 베어링은 위치가 조금만 바뀌어도 100도 이상 요동치는 게 이미 실측돼 있어서
        # (아래 _docking_arc_control 근처 주석 참고), 그 순간 K턴이 새로 진입하며 dir_sign이
        # 매번 다시 뒤집혀 회전이 누적 안 되고 오히려 거리가 늘어나는 걸 실측 확인함(런36:
        # 헤딩오차 93->−136->−137->−98->56->158...도 반전 반복, 거리 0.6m->1.59m로 증가 후
        # 타임아웃). final_approach_radius(도킹 진입 문턱) < close_radius(항상 +0.05m 이상
        # 크게 설계됨)이므로 도킹 진입 순간엔 항상 "고정 goal_yaw" 조건을 만족한다 — 이
        # 판정을 진입 시점에 굳혀서, 이후 K턴 중 일시적으로 멀어져도 점 추종으로 되돌아가지
        # 않게 한다(정상적인 진행 중 재진입만 있을 뿐, 목표를 "잃어버린" 게 아니므로).
        cx0, cy0, _ = self._cur_pose
        gx0, gy0, _ = self._goal_pose
        entry_dist = math.hypot(gx0 - cx0, gy0 - cy0)
        xy_tol = self.get_parameter('docking_xy_tolerance').value
        close_radius = max(xy_tol * 3.0, self.get_parameter('final_approach_radius').value + 0.05)
        self._docking_use_fixed_heading = entry_dist <= close_radius

        self._docking_timer = self.create_timer(0.1, self._docking_control_loop)

    def _docking_control_loop(self):
        if self._cur_pose is None or self._goal_pose is None:
            return

        cx, cy, cyaw = self._cur_pose
        gx, gy, gyaw = self._goal_pose
        dist = math.hypot(gx - cx, gy - cy)
        xy_tol = self.get_parameter('docking_xy_tolerance').value
        yaw_tol = self.get_parameter('docking_yaw_tolerance').value
        yaw_err_to_goal = normalize_angle(gyaw - cyaw)

        if dist <= xy_tol and abs(yaw_err_to_goal) <= yaw_tol:
            self._finish_docking(success=True)
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= self._docking_deadline:
            self.get_logger().error(
                f'정밀 접근 타임아웃 — 남은거리={dist:.2f}m, 헤딩오차={math.degrees(yaw_err_to_goal):.1f}deg')
            self._finish_docking(success=False)
            return

        # ── 정지 감지: STUCK_TIMEOUT_SEC 동안 "진행"이 없으면 막힌 것으로 보고
        #    BACKOFF_SEC 동안 강제 직진 후진한다(조향 0 — 헤딩 보정보다 탈출 우선).
        #    [2026-08-20] 처음엔 위치(POSITION) 변화로만 진행을 쟀는데, 실측해보니
        #    벽까지 0.4m나 여유 있는 완전히 열린 공간에서도 계속 "막혔다"고 오판했다
        #    — K턴은 설계상 제자리에서 회전하는 게 정상 동작(순이동이 거의 0인 게
        #    맞음)이라, 위치 기준 정지감지가 K턴 자체를 오판한 것이었다. K턴 중엔
        #    위치 대신 헤딩 변화로 진행을 판단하도록 분리한다 — 일반 원호 제어일
        #    때는 여전히 위치 기준(그땐 실제로 전후진 이동이 목적이므로).
        STUCK_MOVE_M = 0.03
        STUCK_YAW_RAD = math.radians(8.0)
        STUCK_TIMEOUT_SEC = 2.0
        STUCK_TIMEOUT_SEC_MANEUVER = 5.0  # K턴 phase 2번(왕복 한 사이클, 2.4s)보다 여유있게
        BACKOFF_SEC = 1.5
        BACKOFF_SPEED = 0.12

        stuck_timeout = STUCK_TIMEOUT_SEC_MANEUVER if self._docking_maneuver_active else STUCK_TIMEOUT_SEC
        px, py, pyaw = self._docking_progress_pose
        if self._docking_maneuver_active:
            progressed = abs(normalize_angle(cyaw - pyaw)) >= STUCK_YAW_RAD
        else:
            progressed = math.hypot(cx - px, cy - py) >= STUCK_MOVE_M
        if progressed:
            self._docking_progress_pose = self._cur_pose
            self._docking_progress_t = now_ns
        elif self._docking_backoff_until is None and \
                (now_ns - self._docking_progress_t) / 1e9 >= stuck_timeout:
            self.get_logger().warn(
                f'{stuck_timeout:.0f}초 이상 진행 없음 — 장애물에 막힌 것으로 보고 '
                f'{BACKOFF_SEC:.1f}초 후진 탈출 시도')
            self._docking_backoff_until = now_ns + int(BACKOFF_SEC * 1e9)

        if self._docking_backoff_until is not None:
            if now_ns < self._docking_backoff_until:
                cmd = Twist()
                cmd.linear.x = -BACKOFF_SPEED
                self._cmd_vel_pub.publish(cmd)
                return
            # 후진 종료 — 정지 감지 타이머 리셋하고 다시 정상 제어로 복귀. K턴 중이었으면
            # phase 타이머도 새로 시작해야 한다(안 하면 backoff 전의 stale한 phase_until이
            # 그대로 남아 다음 틱에 곧바로 다시 phase가 뒤집혀버릴 수 있음).
            self._docking_backoff_until = None
            self._docking_progress_pose = self._cur_pose
            self._docking_progress_t = now_ns
            if self._docking_maneuver_active:
                self._docking_maneuver_phase_until = now_ns + int(1.2 * 1e9)

        # 목표점까지 먼 동안엔 목표점 방향(alpha)을 조향 목표로, 거의 다 왔으면
        # 최종 목표 헤딩(goal_yaw)을 조향 목표로 삼는다 — 순수 목표점 추종만 쓰면
        # 도착 직전 헤딩이 goal_yaw와 어긋나도 못 고치기 때문.
        # [2026-08-22] close_radius=0.15m이었을 때 실측 확인된 버그: final_approach_radius를
        # 0.6m로 줄이면서(런36/37, Nav2가 그 전 구간을 전부 담당하게 됨) 도킹 진입~0.15m
        # 사이 전체 구간(0.6~0.15m)에서 target_heading이 atan2(gy-cy, gx-cx)(목표"점"
        # 방향)로 계속 재계산됐는데, 이 거리대에서는 "점 추종" bearing이 차량의 작은
        # 위치 변화에도 각도가 100도 이상씩 요동친다(bearing 민감도가 거리에 반비례).
        # K턴은 헤딩오차 부호로 dir_sign을 진입 시 한 번만 고정하는데, 이 요동 때문에
        # 매번 새로 진입할 때마다 dir_sign이 뒤집혀 누적 회전이 전혀 안 쌓이고(런37 로그:
        # 112→123→161→-104→98→-150...도로 부호 반전 반복) 오히려 거리가 0.60m→1.27m로
        # 늘어나며 타임아웃남. 도킹은 애초에 "이미 목표 근처에 도달한 상태에서 최종
        # pose(위치+방향)를 맞추는" 구간이라 점 추종이 필요 없다 — close_radius를
        # final_approach_radius와 같게 둬서 도킹이 활성화된 전체 구간에서 항상 고정된
        # goal_yaw를 목표로 삼도록 한다(부동 소수 비교 안전하게 살짝 여유를 둠).
        # [2026-08-22] 위 판정을 매 틱 "현재" dist로 재계산하는 한 이 수정으로도 안 끝났다
        # — K턴은 전후진을 오가며 순간적으로 목표에서 멀어지는 게 정상 동작인데, 그때마다
        # dist가 close_radius를 다시 넘어 target_heading이 고정 goal_yaw에서 점 추종
        # (atan2)으로 도로 튀었다(재현: 헤딩오차 93→-136→-137→-98→56→158...도 반전 반복,
        # 거리 0.6m→1.59m로 증가 후 타임아웃). 도킹 진입 시점에 이미 계산해 고정해 둔
        # self._docking_use_fixed_heading(_start_docking() 참고)을 대신 써서, 진입 이후엔
        # 절대 점 추종으로 되돌아가지 않게 한다.
        if self._docking_use_fixed_heading:
            target_heading = gyaw
        else:
            target_heading = math.atan2(gy - cy, gx - cx)
        heading_err = normalize_angle(target_heading - cyaw)

        # [2026-08-20] 전진만 지원하던 버전은 헤딩오차가 90도를 넘으면(목표 방향이
        # 차량 뒤쪽) 못 고쳤다가, 단발성 전진/후진 전환(아래 MANEUVER_ENTRY_DEG 미만
        # 구간)으로 바꿨는데 그것도 -79~-143도대의 "큰" 오차에서는 계속 실패했다
        # (실측: 헤딩오차가 안 줄고 제자리 맴돎). 애커먼만 차량은 큰 헤딩오차를 단일
        # 원호(전진 또는 후진 한 번)로는 못 줄인다 — 실제 운전자가 좁은 곳에서 하는
        # "K턴"(전진-좌, 후진-우, 전진-좌... 번갈아가며 같은 방향으로 계속 회전)처럼
        # 방향을 고정한 채 전후진을 번갈아야 누적 회전이 생긴다. 아래
        # _docking_kturn_control()이 그 상태기계를 담당하고, 작은 오차는 기존
        # 단일-원호 P제어로 충분하므로 그대로 둔다.
        MANEUVER_ENTRY_DEG = 55.0
        if abs(math.degrees(heading_err)) > MANEUVER_ENTRY_DEG or self._docking_maneuver_active:
            cmd = self._docking_kturn_control(heading_err, dist, now_ns)
        else:
            cmd = self._docking_arc_control(heading_err, dist)
        self._cmd_vel_pub.publish(cmd)

    def _docking_arc_control(self, heading_err, dist):
        """헤딩오차가 크지 않을 때 쓰는 단일 원호 P제어 — |heading_err|>90도면
        후진(차량 뒤쪽이 target_heading을 향하도록), 아니면 전진."""
        if abs(heading_err) > math.pi / 2.0:
            direction = -1.0
            steer_err = normalize_angle(heading_err + math.pi)
        else:
            direction = 1.0
            steer_err = heading_err

        kp = self.get_parameter('docking_yaw_kp').value
        approach_speed = self.get_parameter('final_approach_speed').value
        speed = max(approach_speed * 0.4, min(approach_speed, dist * 0.6))

        # [2026-08-20] K턴 회전반경 문제(위 _docking_kturn_control 주석)와 같은 이유로,
        # kp*steer_err가 크면(오차가 55도 근처, 이 함수 진입 문턱값) 회전반경이 차체
        # 길이(0.64m)보다 훨씬 작아져 몸통이 주변을 쓸고 지나갈 수 있다 — 최소 안전
        # 회전반경(MIN_SAFE_RADIUS_M)을 못 넘도록 angular.z 크기를 clamp한다.
        # [2026-08-20] 0.35m로 뒀더니 이 속도대(≤0.15m/s)에서 상한이 초당 0.43라디안
        # 밖에 안 돼서, kp(1.5)×오차가 조금만 커도(약 16도 이상) 거의 항상 그 상한에
        # 눌려버려 오차 크기와 무관하게 똑같이 느린 회전만 나가는 문제가 실측됨(P제어가
        # 사실상 방향만 정하는 bang-bang이 되고, 수렴이 너무 느려져 좁은 구간 안에서
        # 시간 내 못 빠져나감). 0.25m로 완화 — 원래 문제(0.092m, 차체가 훨씬 넓게
        # 쓸고 지나감)보다는 여전히 훨씬 안전하면서, 수렴 속도는 확보.
        MIN_SAFE_RADIUS_M = 0.25
        max_angular = speed / MIN_SAFE_RADIUS_M
        angular_z = kp * steer_err
        # [2026-08-20] 라이다로 좌/우 근접 장애물을 보고 반대쪽으로 미는 바이어스 추가
        # (_lidar_steer_bias() 참고) — 헤딩만 보고 도는 것보다, 실제로 스치기 직전인
        # 방향이 있으면 그쪽을 피하는 게 반복적인 "장애물에 막힘" 실측 사례에 직접
        # 도움이 될 것으로 판단.
        angular_z += self._lidar_steer_bias(direction)
        angular_z = max(-max_angular, min(max_angular, angular_z))

        cmd = Twist()
        cmd.linear.x = direction * speed
        cmd.angular.z = angular_z  # direction 안 곱함(_docking_arc_control 도입 전 주석과 동일 이유)
        return cmd

    def _docking_kturn_control(self, heading_err, dist, now_ns):
        """큰 헤딩오차용 K턴 상태기계. 목표 회전방향(dir_sign)을 진입 시 한 번만
        정하고, 그 방향 요레이트(angular.z)는 전진/후진 내내 고정한 채 linear.x
        부호만 PHASE_DURATION_SEC마다 번갈아 — 자전거모델에서 v의 부호가 바뀌어도
        조향각을 반대로 주면 같은 방향 회전이 누적된다(전진-좌회전, 후진-우회전을
        번갈으면 둘 다 반시계 회전으로 쌓이는 것과 동일 원리). 오차가
        MANEUVER_EXIT_DEG 밑으로 떨어지면 단일 원호 제어로 복귀(히스테리시스로
        진입/이탈 문턱을 다르게 둬서 경계에서 떨림 방지)."""
        MANEUVER_EXIT_DEG = 25.0
        PHASE_DURATION_SEC = 1.2
        # [2026-08-20] 원래 1.3rad/s(=회전반경 v/w≈0.092m)였는데, 이 차 길이가 0.64m
        # (vehicle_spec.py VEHICLE_LENGTH_M)라 회전반경이 차체 길이의 1/7밖에 안
        # 됐다 — 바퀴 자체는 그 반경을 낼 수 있어도(최대조향 80도) 차체(특히 앞뒤
        # 튀어나온 부분)는 그 작은 원을 돌며 훨씬 넓은 범위를 쓸고 지나가서, 좁은
        # 공간에서 거의 항상 제 몸통으로 주변에 부딪혀 못 나아가는 것으로 실측
        # 확인됨(반복적인 "장애물에 막힘" 로그). 0.35m로 낮췄더니 이번엔 반대로 너무
        # 느려서(P제어가 사실상 상한에 눌린 bang-bang이 됨) 좁은 구간 안에서 시간 내에
        # 못 빠져나가는 문제가 나옴 — _docking_arc_control()의 MIN_SAFE_RADIUS_M과
        # 동일하게 0.25m로 재조정(원래 0.092m보다는 훨씬 안전, 0.35m보다는 빠름).
        # [2026-08-22] 이 회전반경(0.25m)은 그대로 두되, 속도/각속도 크기를 0.6배로
        # 낮춘다 — K턴 도중 AMCL이 발산해 헤딩오차가 매 틱 -178도~176도 사이를 무작위로
        # 오가는 게 실측 확인됨(global_costmap sensor origin이 지도 경계 밖(y=7.09,
        # 지도최대 6.38)까지 찍힘 — 실측 위치가 아니라 추정 자체가 튄 것). 요레이트
        # 0.48rad/s(27.5deg/s)가 AMCL 모션모델/스캔매칭이 매 스캔 사이 따라잡기엔 너무
        # 빨랐을 가능성이 있어, 회전반경(=MANEUVER_SPEED/MANEUVER_ANGULAR)은 그대로
        # 0.25m로 유지한 채(몸통 스침 방지 요구사항은 안 건드림) 두 값을 같은 비율로
        # 낮춰 시간당 회전량만 완화한다.
        MANEUVER_SPEED = 0.12 * 0.6
        MANEUVER_ANGULAR = 0.48 * 0.6  # rad/s, 고정 크기(회전반경 ≈ MANEUVER_SPEED/MANEUVER_ANGULAR ≈ 0.25m, 비율 유지)

        if abs(math.degrees(heading_err)) <= MANEUVER_EXIT_DEG:
            self._docking_maneuver_active = False
            self._docking_maneuver_phase_until = None
            # 오차가 충분히 줄었으니 이번 틱부터 단일 원호 제어로 넘긴다.
            return self._docking_arc_control(heading_err, dist)

        if not self._docking_maneuver_active:
            self.get_logger().warn(
                f'헤딩오차 {math.degrees(heading_err):.0f}도 — K턴(전후진 번갈아 회전) 진입')
            self._docking_maneuver_active = True
            self._docking_maneuver_dir_sign = 1.0 if heading_err > 0 else -1.0
            self._docking_maneuver_phase = 1.0  # 전진부터 시작
            self._docking_maneuver_phase_until = now_ns + int(PHASE_DURATION_SEC * 1e9)
        elif now_ns >= self._docking_maneuver_phase_until:
            self._docking_maneuver_phase *= -1.0
            self._docking_maneuver_phase_until = now_ns + int(PHASE_DURATION_SEC * 1e9)

        # [2026-08-20] 라이다 근접 바이어스를 K턴에도 더하되, K턴의 핵심 전제(요레이트
        # 부호를 전진/후진 내내 고정해야 회전이 누적됨, 위 docstring 참고)를 깨지
        # 않도록 작은 가중치(0.4)만 섞는다 — dir_sign이 정하는 "의도한 회전"을
        # 뒤집지 않으면서, 바로 옆에 뭔가 있으면 살짝 밀어내는 정도로만 개입.
        angular_z = self._docking_maneuver_dir_sign * MANEUVER_ANGULAR
        angular_z += 0.4 * self._lidar_steer_bias(self._docking_maneuver_phase)

        cmd = Twist()
        cmd.linear.x = self._docking_maneuver_phase * MANEUVER_SPEED
        cmd.angular.z = angular_z
        return cmd

    def _finish_docking(self, success: bool):
        if self._docking_timer is not None:
            self._docking_timer.cancel()
            self._docking_timer = None
        self._cmd_vel_pub.publish(Twist())  # 정지
        if success:
            self.get_logger().info('주차 완료: 정밀 접근으로 목표 pose에 도달했습니다.')
        self._success = success
        self._done = True

    def _result_cb(self, future):
        if self._docking_active:
            # 이 결과는 _start_docking()이 스스로 취소한 목표에 대한 것 — 최종 성패는
            # _finish_docking()이 정한다. 여기서 _done을 건드리면 그 결과를 덮어쓴다.
            status = future.result().status
            self.get_logger().info(f'(정밀 접근 전환으로 취소된 Nav2 목표 결과: status={status})')
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('주차 완료: 목표 pose에 도달했습니다.')
            self._success = True
        else:
            self.get_logger().error(f'주차 실패 (status={status}).')
            self._success = False
        self._done = True

    def _spin_once_safe(self, timeout_sec=0.2):
        """[2026-08-20] rclpy.spin_once()가 드물게(실측: Nav2 controller_server 활성화로
        시스템 부하가 몰리는 순간과 겹칠 때) 'Unable to convert call argument to Python
        object'라는 RuntimeError로 죽는 것을 확인함 — 초음파 Range 구독을 추가한 뒤부터
        재현됨(엔티티 수가 늘어난 것과 관련된 걸로 보임, 메시지 자체는 `ros2 topic echo`로
        연속 수신해봐도 문제 없었음). 메시지 한 번 못 읽은 걸로 미션 전체가 죽으면 안
        되므로, 그 틱만 건너뛰고 계속 진행한다(다음 스핀에서 정상적으로 다시 받음)."""
        try:
            rclpy.spin_once(self, timeout_sec=timeout_sec)
        except RuntimeError as e:
            self.get_logger().warn(f'spin_once 일시 오류(무시하고 계속): {e}', throttle_duration_sec=2.0)

    def run(self):
        zone = self.get_parameter('parking_zone').value

        if self.get_parameter('set_initial_pose').value:
            wait_sec = self.get_parameter('initial_pose_wait_sec').value
            deadline = self.get_clock().now().nanoseconds + int(wait_sec * 1e9)
            while (self._initial_pose_pub.get_subscription_count() == 0
                   and self.get_clock().now().nanoseconds < deadline):
                self._spin_once_safe(timeout_sec=0.2)
            self.publish_initial_pose_until_amcl_ready(wait_sec)

        self.get_logger().info('navigate_to_pose 액션 서버 대기 중...')
        self._nav_client.wait_for_server()

        self.send_goal(zone)

        while rclpy.ok() and not self._done:
            self._spin_once_safe(timeout_sec=0.2)

        return self._success


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNavigator()
    try:
        success = node.run()
    except ValueError as e:
        node.get_logger().error(str(e))
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
