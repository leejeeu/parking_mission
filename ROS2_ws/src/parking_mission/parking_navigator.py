import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
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

        # [2026-08-24] 이 노드는 use_sim_time:=true로 실행되는데, Gazebo+Nav2+AMCL을
        # 동시에 돌리는 이 개발환경은 부하에 따라 시뮬레이션 real-time-factor가
        # 크게 튄다 — 실측 확인: 15초 타임아웃이 실제 1.4초 만에 끝나거나(부하가
        # 걷히며 시뮬레이션 시계가 밀린 걸 몰아서 따라잡음), 반대로 4초 넘게 정지해
        # 있었는데도 2초 무진행 감지가 발동을 안 함(부하로 시뮬레이션 시계가 실제보다
        # 느리게 감). 데드라인/정지감지/속도램프처럼 "실제로 몇 초가 지났는가"가
        # 중요한 내부 타이밍 계산은 전부 이 별도 SYSTEM_TIME(실제 시계)를 쓰고,
        # self.get_clock()(시뮬레이션 시계)는 TF/메시지 타임스탬프처럼 ROS 시스템
        # 전체와 시간대를 맞춰야 하는 곳에만 남겨둔다.
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # [2026-08-24] 공식 경기 규정 확인 결과 이 미션은 "출발 → A 주차 → B 주차 →
        # 출발지 복귀"를 전부 한 번에 수행해야 한다(과거엔 A 또는 B 중 하나에만 가고
        # 끝나는 것으로 잘못 구현돼 있었음). 'FULL'(기본값)이면 MISSION_LEGS 전체를
        # 순서대로 수행하고, 'A'/'B'처럼 개별 레그 이름을 주면 그 레그 하나만 실행한다
        # (기존 단일 레그 동작 — 디버깅/개별 구간 튜닝용으로 유지).
        self.declare_parameter('parking_zone', 'FULL')
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

        # ── 레그별 병목 구간 우회("route bypass") — 2026-08-22 도입, 2026-08-24 일반화 ──
        #   대회 제공 실측 지도(parking_map.pgm, 사용자 확인 — 시뮬레이션 임의 지도가
        #   아니라 실제 대회장 지도)에 있는 중앙 기둥과 서쪽 벽 사이 통로(x≈0.7~1.7,
        #   y≈1.5~3.9)가 실측(거리변환) 결과 폭 0.7m 이상, 중앙 클리어런스 0.35~0.45m로
        #   물리적으로는 충분히 넓은데도(풋프린트 inscribed radius 0.225m보다 큼),
        #   SmacPlannerHybrid+RPP는 이 구간에서 반복적으로 한쪽(기둥 쪽, 클리어런스
        #   0.10m 안팎)에 붙어 지나가다 "Starting point in lethal space"로 멈추는 걸
        #   5회 반복 테스트 전부(5/5)에서 재현 확인함 — inflation_radius(0.1->0.25로
        #   상향, 목표 도달가능성 재계산해서 안전 확인함)와 cost_penalty(2.0->4.0/6.0)
        #   조정으로도 못 고쳤다. 목표 근처의 "정밀 접근"(도킹)과 동일한 근거로, 코스트맵
        #   기반 전역계획을 아예 거치지 않고 AMCL 피드백만으로 이 구간의 중심선(가장
        #   넓은 지점들을 지나는 웨이포인트)까지 직접 주행한 뒤, 그 지점부터 다시 Nav2에
        #   넘긴다(그 지점 이후는 목표까지 열린 공간이라 Nav2가 충분히 처리 가능).
        #   [2026-08-24] 미션이 "출발→A→B→출발복귀" 다중 레그로 바뀌면서, 이 우회를
        #   "zone A 전용"이 아니라 "어느 (이전레그,다음레그) 쌍에서 이 병목을 지나는지"
        #   기준으로 일반화했다(ROUTE_BYPASS_CENTERLINE 참고). 지금 검증된 건 출발→A
        #   구간 하나뿐이고, A→B/B→출발 구간은 이번에 새로 생기는 미검증 경로라 테이블에
        #   아직 없음 — 시뮬레이션에서 같은 증상이 재현되면 그때 추가할 것.
        self.declare_parameter('corridor_bypass_enable', True)
        self.declare_parameter('corridor_bypass_xy_tolerance', 0.15)
        self.declare_parameter('corridor_bypass_speed', 0.45)
        # [2026-08-24] 3분(180초) 안에 A/B 주차 + 출발지 복귀까지 전부 마쳐야 하는
        # 제약이 생겨서(과거엔 구역 하나만 가면 끝이라 여유로웠음) 30초 → 15초로 축소됐었으나,
        # 실측 결과(데드레커닝 도입 후) 첫 웨이포인트 도달까지만 20초 넘게 걸리는 걸
        # 확인해서 25초로, 그 뒤 좁은 통로 5개 웨이포인트를 전부 통과(wp1이 15초 가까이
        # 걸림)하고 마지막 긴 직선구간(0.95,3.55, 1.8m)까지 가는 데는 25초로도 부족한
        # 걸 실측 확인해서 45초로 재상향 — 전체 미션 예산(mission_deadline_sec=170s)
        # 안에서는 여전히 여유 있음(START->A 구간 하나만 이 정도 쓰고 나머지 레그는 병목이 없음).
        self.declare_parameter('corridor_bypass_timeout_sec', 55.0)

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
        self.declare_parameter('final_approach_speed', 0.35)
        # nav2_params.yaml의 goal_checker(xy_goal_tolerance/yaw_goal_tolerance)와
        # 동일 값을 기본값으로 두되, 이 노드만 따로 튜닝할 수 있게 별도 파라미터로 뺌.
        self.declare_parameter('docking_xy_tolerance', 0.05)
        self.declare_parameter('docking_yaw_tolerance', 0.05)
        # [2026-08-20] final_approach_radius를 2.5m로 늘려 도킹이 더 긴 구간을
        # 커버해야 하므로, 그만큼 시간 여유도 같이 늘림(대략 2.5m/0.15m/s ≈ 17s +
        # 헤딩보정 여유). [2026-08-24] 3분 예산을 3개 레그가 나눠 써야 해서 40→20초로
        # 축소 — 후진(K턴) 복원으로 큰 헤딩오차도 더 빨리 좁힐 수 있어 여유 있게 줄임.
        self.declare_parameter('docking_timeout_sec', 20.0)
        self.declare_parameter('docking_yaw_kp', 1.5)

        # [2026-08-24] 전체 미션(출발→A→B→출발복귀) 소프트 데드라인. 경기 규정 제한시간
        # 180초보다 여유를 두어(레그 도중 강제 중단하면 오히려 위험한 자세로 멈출 수
        # 있으므로), 이 시간을 넘기면 진행 중이던 레그를 즉시 정지시키고 남은 레그는
        # 건너뛴 채 미션을 종료한다 — 심판이 강제로 중단시키는 것보다 스스로 안전하게
        # 멈추는 편이 규정 제8조 취지에 맞다.
        self.declare_parameter('mission_deadline_sec', 170.0)

        # [2026-08-24] launch/parking_mission.launch.py의 lidar_yaw_deg(라이다 장착
        # 회전각)를 이 노드에도 전달받는다 — _lidar_front_clearance()/_lidar_steer_bias()가
        # 원시 /scan 각도(angle_min/angle_increment)를 TF 없이 직접 써서 "각도 0=차량
        # 정면"으로 가정하고 있었는데, 실제로는 base_link->laser 정적 TF에 이 회전 오프셋이
        # 들어가야 맞다(UMK 저장소 LIDAR_ANGLE_OFFSET_DEG=80(2026-07-22 실측, 재확인 결과
        # 88~89로 드리프트 정황 있음) 참고 — 같은 실차라 그 값이 출발점으로 유효하나,
        # 재확인 전까지는 launch 기본값 0.0 그대로 둔다). 이 오프셋이 틀리면 "정면"
        # 안전정지/조향바이어스가 엉뚱한 방향을 보게 되므로 실차 투입 전 반드시 재실측할 것.
        self.declare_parameter('lidar_yaw_deg', 0.0)
        self._lidar_yaw_offset_rad = math.radians(self.get_parameter('lidar_yaw_deg').value)

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
        # [2026-08-24] 미션이 다중 레그(출발→A→B→출발복귀)로 바뀌면서 "레그 하나 완료"와
        # "미션 전체 완료"를 구분해야 한다 — _leg_done/_leg_success는 send_goal() 호출마다
        # run()의 레그 루프가 리셋하며 지켜보는 값이고, 미션 전체 성패는 run()이 레그별
        # 결과를 모아서 별도로 판단한다.
        self._leg_done = False
        self._leg_success = False
        self._current_leg = None
        self._retry_count = 0

        # [2026-08-22] 직접제어(도킹/병목우회 공용) 가속 램프 상태 — _ramp_linear_speed()
        # 참고. 목표속도를 매틱 한 번에 명령하면 이 차의 약한 속도 PID(ackermann_drive,
        # urdf/xycar.urdf 참고)가 못 따라가는 걸 실측 확인해서(명령 0.15m/s인데 실제
        # 0.006m/s, 명령의 4.3%) 도입.
        self._last_direct_linear_x = 0.0
        self._last_direct_cmd_t_ns = None
        # [2026-08-24] 위 속도 램프와 같은 이유로 조향(angular.z)에도 램프를 추가.
        # 헤딩오차가 부호를 뒤집을 때마다(병목우회/도킹 둘 다 이런 구간을 반복 통과)
        # kp*heading_err가 clamp 상한(max_angular)까지 순간적으로 튀어 최대좌->최대우로
        # 한 틱만에 널뛰는 걸 실측 확인함(사용자 관찰: "타이어가 와리가리 흔들림") —
        # 조향에는 이 램프가 없어서 속도 램프와 달리 계단형 명령이 그대로 나갔었다.
        self._last_direct_angular_z = 0.0
        self._last_direct_angular_t_ns = None

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
        # [2026-08-24] START->A 병목구간(중앙기둥 남동쪽 모서리 근처, x~1.5/y~1.75)
        # 전용 데드레커닝용. AMCL이 이 구간에서 실제 위치와 무관하게 그쪽으로 쏠리는
        # 국소 함정(local trap)이 있는 걸 반복 재현으로 확인함(웨이포인트/조향가중치/
        # 허용오차를 바꿔도 매번 같은 좌표로 수렴하거나 순간이동) — 스캔매칭 자체의
        # 문제로 판단, 이 구간만은 AMCL을 안 쓰고 odom_publisher.py가 발행하는 /odom
        # (VESC 속도 + IMU 요만 사용, 정확도 실측 확인됨)으로 상대이동만 추적한다.
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self._odom_msg = None
        self._dr_anchor = None  # (map_x, map_y, odom_x, odom_y, yaw_offset) — 병목구간 진입 시 1회 설정
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

        # ── K턴(후진 포함 헤딩보정) 상태 — 2026-08-24 복원 ──
        # 후진이 실제로는 완전히 허용된다는 게 확인되어(2026-08-24), 헤딩오차가 큰
        # 경우 "후진 없이 큰 원호로만 도는" 방식 대신 원래 있었던 K턴(전진-한쪽조향/
        # 후진-반대조향을 번갈아 실행해 제자리에 가깝게 헤딩을 좁히는 방식)을 되살린다
        # (_docking_k_turn_control() 참고).
        self._docking_k_turn_active = False
        self._docking_k_turn_dir_sign = 0.0
        self._docking_k_turn_phase_direction = 1.0
        self._docking_k_turn_phase_deadline_ns = None

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

    # 미션 레그 고정 순서 — 공식 맵 안내문 기준("출발 → A 주차 → B 주차 → 출발지 복귀").
    MISSION_LEGS = ['A', 'B', 'START']

    def _leg_pose(self, leg: str):
        """레그 이름('A'/'B'/'START')에 대응하는 목표 pose. 'START'는 출발지 복귀용으로
        기존 start_x/y/yaw 파라미터(초기 pose 설정에 쓰던 값과 동일)를 재사용한다."""
        leg = leg.upper()
        if leg == 'START':
            x = self.get_parameter('start_x').value
            y = self.get_parameter('start_y').value
            yaw = self.get_parameter('start_yaw').value
            return x, y, yaw
        return self.zone_pose(leg)

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

    def _cb_odom(self, msg: Odometry):
        self._odom_msg = msg

    def _odom_xyz(self):
        """/odom(odom_publisher.py, VESC속도+IMU요 데드레커닝)에서 (x,y,yaw) 추출.
        아직 못 받았으면 None."""
        if self._odom_msg is None:
            return None
        p = self._odom_msg.pose.pose
        return (p.position.x, p.position.y, quaternion_to_yaw(p.orientation))

    def _dr_set_anchor(self):
        """병목구간 진입 시 1회 호출 — 그 순간의 AMCL pose(신뢰 가능, 방금
        publish_initial_pose_until_amcl_ready()로 확인된 직후)를 기준점으로 삼고,
        같은 순간의 /odom 값과의 오프셋을 저장한다. 이후 _dr_pose()는 이 기준점 +
        odom 상대이동만으로 위치를 추정해, 이 구간에서 반복 재현된 AMCL 국소 함정
        (위 __init__ 주석 참고)의 영향을 받지 않는다."""
        amcl = self._cur_pose
        odom = self._odom_xyz()
        if amcl is None or odom is None:
            self._dr_anchor = None
            return False
        map_x, map_y, map_yaw = amcl
        odom_x, odom_y, odom_yaw = odom
        yaw_offset = normalize_angle(map_yaw - odom_yaw)
        self._dr_anchor = (map_x, map_y, odom_x, odom_y, yaw_offset)
        return True

    def _dr_pose(self):
        """데드레커닝 추정 pose(x,y,yaw) — _dr_set_anchor() 이후, 기준점에 odom
        상대이동(기준시점 오프셋만큼 회전변환)을 더해서 계산. 앵커/odom 둘 다
        없으면 AMCL(self._cur_pose)로 폴백(둘 다 못 받은 극초반 대비)."""
        odom = self._odom_xyz()
        if self._dr_anchor is None or odom is None:
            return self._cur_pose
        map_x0, map_y0, odom_x0, odom_y0, yaw_offset = self._dr_anchor
        odom_x, odom_y, odom_yaw = odom
        dx, dy = odom_x - odom_x0, odom_y - odom_y0
        cos_o, sin_o = math.cos(yaw_offset), math.sin(yaw_offset)
        map_dx = dx * cos_o - dy * sin_o
        map_dy = dx * sin_o + dy * cos_o
        return (map_x0 + map_dx, map_y0 + map_dy, normalize_angle(odom_yaw + yaw_offset))

    def _cb_scan(self, msg: LaserScan):
        self._scan_msg = msg

    def _lidar_steer_bias(self, direction: float) -> float:
        """도킹 이동 방향(direction>0=전진, direction<0=후진 — 2026-08-24 K턴 복원으로
        후진 phase에서도 호출됨) 쪽 반원에서 좌/우 중 더 가까운 장애물을 찾아 반대쪽으로
        미는 조향 바이어스(angular.z에 더할 값, rad/s)를 반환한다. 가까운 게 없으면
        0.0. "카메라로 왼쪽 장애물 보이면 오른쪽으로 꺾기"와 같은 발상을 라이다로
        구현한 것(이 파이프라인엔 카메라가 없음, 라이다+AMCL만 사용)."""
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
            # [2026-08-24] lidar_yaw_deg(라이다 장착 회전 오프셋)를 반영 — 이 함수는
            # base_link->laser TF를 거치지 않고 raw /scan 각도를 직접 쓰므로, 라이다가
            # 정면 기준으로 돌아가 있으면 이 보정 없이는 "정면"을 엉뚱한 방향으로
            # 오판한다(실차 UMK 저장소 LIDAR_ANGLE_OFFSET_DEG=80 참고 — 같은 실차).
            rel = normalize_angle(angle - base_angle + self._lidar_yaw_offset_rad)
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

    def _lidar_front_clearance(self, direction: float = 1.0) -> float:
        """진행방향(direction>0=전진) 정면 좁은 원뿔 내 최소 라이다 거리(m). 장애물이
        없으면 CHECK_RANGE_M(사실상 '제한 없음')을 반환.

        [2026-08-22] Gazebo 재현: 병목 구간(중앙 기둥 근처) 통과 중 AMCL 추정 pose가
        실제 Gazebo pose와 약 1m 어긋난 채(원인은 그 구간의 코너 케이스 스캔매칭
        저하로 추정, 근본 수정은 별도 과제) _run_route_bypass()가 "AMCL 기준으로는
        열린 공간"이라고 믿고 그대로 기둥에 차체를 박아 넣었다(실측: cur pose는
        (1.22,1.34) 정지 상태를 15초 넘게 보고했는데, 그 시점 /gazebo/odom 기준 실제
        위치는 기둥 바로 옆이었음). _lidar_steer_bias()는 조향 바이어스만 줄 뿐 속도를
        줄이지 않아 막지 못한다 — AMCL 오차와 무관하게 차량 기준 직접 측정값인 라이다
        정면 거리로 전진 속도 자체를 낮춰서(_front_safety_speed_cap 참고), 위치추정이
        틀리더라도 실제로 닿기 전에 감속/정지하도록 하는 국소 안전망."""
        if self._scan_msg is None:
            return float('inf')
        msg = self._scan_msg
        CHECK_RANGE_M = 0.5
        CHECK_HALF_ANGLE = math.radians(20)
        base_angle = 0.0 if direction >= 0 else math.pi
        min_r = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (0.05 < r < CHECK_RANGE_M):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # [2026-08-24] _lidar_steer_bias()와 동일한 이유로 lidar_yaw_deg 오프셋 반영.
            rel = normalize_angle(angle - base_angle + self._lidar_yaw_offset_rad)
            if abs(rel) > CHECK_HALF_ANGLE:
                continue
            min_r = min(min_r, r)
        return min_r

    def _front_safety_speed_cap(self, nominal_speed: float, direction: float = 1.0) -> float:
        """_lidar_front_clearance() 기반 속도 상한 — FRONT_SAFETY_MARGIN_M 이내면 완전
        정지, FRONT_SLOWDOWN_RANGE_M 밖이면 무제한, 그 사이는 선형 감속. 정지 감지 후
        탈출(backoff) 동작에는 적용하지 않음(그쪽은 이미 저속+반대쪽 조향으로 의도적으로
        접근하는 전용 로직이라 그대로 0으로 눌리면 영영 못 빠져나온다) — 두 호출부
        모두 backoff 분기가 이 함수 호출 전에 이미 return/continue한다."""
        FRONT_SAFETY_MARGIN_M = 0.12
        FRONT_SLOWDOWN_RANGE_M = 0.35
        front_clear = self._lidar_front_clearance(direction)
        if front_clear <= FRONT_SAFETY_MARGIN_M:
            return 0.0
        if front_clear >= FRONT_SLOWDOWN_RANGE_M:
            return nominal_speed
        ratio = (front_clear - FRONT_SAFETY_MARGIN_M) / (FRONT_SLOWDOWN_RANGE_M - FRONT_SAFETY_MARGIN_M)
        return nominal_speed * ratio

    def publish_initial_pose(self, x=None, y=None, yaw=None):
        """x/y/yaw을 안 주면 미션 시작점(start_x/y/yaw 파라미터)을 쓴다 — 기존
        호출부(미션 시작 시 1회) 그대로 호환. [2026-08-24] 병목구간 데드레커닝->AMCL
        전환 시점에 임의의 위치로 재정렬시키기 위해 인자를 받도록 일반화(아래
        _run_route_bypass 참고)."""
        if x is None:
            x = self.get_parameter('start_x').value
        if y is None:
            y = self.get_parameter('start_y').value
        if yaw is None:
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
        deadline = self._wall_clock.now().nanoseconds + int(timeout_sec * 1e9)
        republish_period_sec = 1.0
        next_republish = self._wall_clock.now().nanoseconds

        self.publish_initial_pose()
        while self._cur_pose is None and self._wall_clock.now().nanoseconds < deadline:
            self._spin_once_safe(timeout_sec=0.2)
            if self._cur_pose is None and self._wall_clock.now().nanoseconds >= next_republish:
                self.publish_initial_pose()
                next_republish = self._wall_clock.now().nanoseconds + int(republish_period_sec * 1e9)

        received = self._cur_pose is not None
        if received:
            self.get_logger().info('AMCL이 초기 pose를 처리함(/amcl_pose 수신 확인)')
        else:
            self.get_logger().warn(
                f'/amcl_pose를 {timeout_sec:.0f}초 안에 못 받음 — 그래도 목표 전송 시도')
        return received

    def _resync_amcl_to(self, x, y, yaw, timeout_sec):
        """[2026-08-24] 병목구간을 데드레커닝(_dr_pose)으로 통과하는 동안, AMCL 자체는
        (제어에 안 쓰일 뿐) 계속 라이다로 자체 추정을 갱신하고 있는데 — 좁은 통로+기둥
        근처의 대칭적 형상 때문에 실측으로 여러 번 확인된 "AMCL 국소함정"(전혀 다른
        위치로 잘못 수렴)에 빠진 채로 있는 경우가 있다. 그 상태에서 idx>=dr_n(AMCL
        신뢰 전환)에 도달하면, 신뢰할 수 있었던 데드레커닝 대신 이미 틀어진 AMCL을
        그대로 넘겨받아 dist/heading_err가 1m+로 튀고 그 자리에서 헛돌다 타임아웃 →
        Nav2가 "Starting point in lethal space"로 못 뜨는 연쇄 실패가 실측 재현됨.
        publish_initial_pose_until_amcl_ready()와 같은 방식(RViz 2D Pose Estimate
        재현)으로, 신뢰할 수 있는 데드레커닝 추정치를 /initialpose로 주기 재발행해
        AMCL 파티클을 강제로 그 근방에 재수렴시킨다 — 단발 발행 대신 실제
        self._cur_pose가 그 근방에 도달했는지(수렴 확인)까지 기다리는 점이
        publish_initial_pose_until_amcl_ready()와 다르다(그쪽은 최초 기동 시
        self._cur_pose가 아예 None이라 '아무 값이나 옴'만 확인하면 충분했음)."""
        CONVERGE_TOL_M = 0.35
        deadline = self._wall_clock.now().nanoseconds + int(timeout_sec * 1e9)
        republish_period_sec = 1.0
        next_republish = self._wall_clock.now().nanoseconds

        self.publish_initial_pose(x, y, yaw)
        while rclpy.ok() and self._wall_clock.now().nanoseconds < deadline:
            self._spin_once_safe(timeout_sec=0.2)
            if self._cur_pose is not None:
                cx, cy, _ = self._cur_pose
                if math.hypot(cx - x, cy - y) <= CONVERGE_TOL_M:
                    self.get_logger().info(
                        f'AMCL 재정렬 완료 — 데드레커닝 기준({x:.2f},{y:.2f}) 근방({cx:.2f},{cy:.2f})으로 수렴')
                    return True
            now_ns = self._wall_clock.now().nanoseconds
            if now_ns >= next_republish:
                self.publish_initial_pose(x, y, yaw)
                next_republish = now_ns + int(republish_period_sec * 1e9)
        self.get_logger().warn(
            f'AMCL 재정렬 타임아웃({timeout_sec:.0f}초) — 데드레커닝 기준({x:.2f},{y:.2f})으로 '
            f'수렴 확인 못 함, 그래도 계속 진행')
        return False

    def send_goal(self, leg: str):
        """레그(leg) 하나의 Nav2 목표를 전송한다. 다중 레그 미션(run() 참고)에서 매
        레그마다 새로 호출되므로, 레그 단위로 리셋돼야 하는 상태(재시도 카운터,
        레그 완료 플래그, 도킹 활성 상태)를 여기서 초기화한다."""
        self._current_leg = leg.upper()
        # 주의: _retry_count는 여기서 리셋하지 않는다 — _retry_send_goal()도 이
        # 함수를 재호출하므로, 여기서 리셋하면 재시도 카운터가 매번 0으로 돌아가
        # GOAL_RETRY_MAX가 무력화된다. 레그당 1회 리셋은 run()의 레그 루프가 담당한다.
        self._leg_done = False
        self._leg_success = False
        self._docking_active = False

        x, y, yaw = self._leg_pose(leg)
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

        leg_desc = f'주차영역 {self._current_leg}' if self._current_leg in ('A', 'B') else '출발지 복귀'
        self.get_logger().info(
            f"{leg_desc} 목표 전송: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

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
                self._leg_done = True
                self._leg_success = False
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)
        # 목표가 정상 수락된 이후부터 _cb_amcl_pose()가 매 pose 갱신마다
        # _maybe_start_docking()을 호출해 정밀 접근 반경 진입을 확인한다(별도
        # 타이머는 안 씀 — 위 _cb_amcl_pose 주석 참고).

    def _retry_send_goal(self, timer):
        timer.cancel()
        self.send_goal(self._current_leg)

    def _maybe_start_docking(self):
        if self._docking_active or self._leg_done:
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

    # [2026-08-22 도입, 2026-08-24 레그별 테이블로 일반화] 병목구간 "안전 중심선" 중간
    # 웨이포인트 — 기존엔 시작점에서 최종 지점(0.95, 3.9) 단 하나만 직선(점추종)으로
    # 조준했는데, 그 직선이 실제 자유공간 중심선과 크게 어긋난다는 게 실측으로 확인됨:
    # maps/parking_map.pgm에 scipy.ndimage.distance_transform_edt로 각 y에서 가장 가까운
    # 장애물까지 거리가 최대인 x(=안전 중심선)를 구해보면, y=1.5 부근 중심선은 x≈0.60인데
    # 시작점(1.8,0.9) -> 최종 웨이포인트(0.95,3.9) 직선은 그 y에서 x≈1.63을 지난다 — 1m
    # 넘게 기둥 쪽으로 치우친 경로였다. 이게 실제 Gazebo 재현(2026-08-22,
    # _front_safety_speed_cap 도입 계기)에서 AMCL 추정치와 무관하게 매번 같은 지점(기둥
    # 근처)에서 버벅인 근본 원인 — 라이다 안전감속(_front_safety_speed_cap)은 "박는
    # 순간"을 완화할 뿐 애초에 기둥 쪽으로 붙어가는 경로 자체는 못 고친다.
    #
    # [2026-08-24] 미션이 다중 레그(출발→A→B→출발복귀)로 바뀌면서 "zone A 전용"이던
    # 게이트를 (이전레그, 다음레그) 쌍 기준 테이블로 일반화했다. 지금 값이 있는 건
    # 실측 검증된 출발→A 구간뿐 — A→B/B→출발 구간은 이번에 처음 생기는 경로라 아직
    # 값이 없다(시뮬레이션에서 같은 "Starting point in lethal space" 증상이 재현되면
    # 그때 추가할 것, 아래 _run_route_bypass 참고).
    # [2026-08-24] parking_map.pgm에 로봇 반경(0.30m) 팽창 후 Dijkstra 최단경로를
    # 직접 계산해서 얻은 값으로 교체. 기존 (0.60,1.70)은 이 y구간(1.6~1.75)에서
    # 서쪽 ㄱ자 걸쇠 벽 쪽으로 붙는 좌표라, 로봇 반경 기준으로 보면 계산상 안전한
    # 통과지점(x≈0.95~1.00)보다 오히려 여유가 부족했다 — 실측으로 반복 재현된
    # "병목에서 못 빠져나옴" 실패의 원인 중 하나였다. 맨 앞 (1.20,1.15)는 추가
    # 진입각 보정용 — 지도 원본 해상도로 확인한 결과, 시작점(1.8,0.9)에서 바로
    # (1.00,1.60)으로 조향하면 그 벽 모서리(약 x=1.3~1.45,y=1.70~1.85)를 스치듯
    # 크게 도는 궤적이 나와 반복 재현됐다(웨이포인트/조향가중치/허용오차를 바꿔도
    # 동일 지점에서 멈춤 — 튜닝으로 안 풀리는 기하학적 문제로 판단). 모서리에서
    # 충분히 남쪽(clearance 0.585m)인 이 점을 먼저 거치게 해서, 모서리를 넓게 돈
    # 뒤 이미 남쪽 열린 공간에 있는 상태에서 (1.00,1.60)로 짧게 북상하도록 유도한다.
    # [2026-08-24] 위 (1.00,1.60)/(1.00,1.70)/(0.95,1.75)는 Dijkstra 최단경로가 준
    # "도달 가능한" 지점이었을 뿐 실제 "가장 안전한"(양쪽 벽에서 최대로 먼) 지점이
    # 아니었다 — 사용자가 시뮬레이션에서 차가 벽에 긁히는 걸 직접 확인, distance_transform
    # 으로 이 y구간(1.55~1.85)의 진짜 중심선(최근접 장애물까지 거리 최대인 x)을 다시
    # 계산해보니 x≈0.50~0.58(여유 0.67~0.80m)로, 기존 x=1.00(여유 0.30~0.35m)보다
    # 훨씬 넉넉했다. y=1.90~3.55 구간은 중심선이 x≈0.63->0.98로 완만하게 이동한다 —
    # 이 곡선을 따라가도록 중간 전환점을 추가.
    # [2026-08-24] (1.20,1.15)->(0.55,1.65) 직행(거리 1.44m)이 조향으로 못 따라잡고
    # 그냥 목표를 지나쳐 벽 밖(x=-0.60)까지 가버리는 걸 실측 확인 — 사용자 제안대로
    # 손으로 몇 점 고르는 대신, distance_transform 중심선을 0.20m 간격으로 촘촘히
    # 리샘플링해서 웨이포인트 목록 전체를 계산했다(scripts, ROS2_ws/src/maps 기준
    # world 좌표). 각 세그먼트가 짧아(<=0.2m 대각선) 조향이 못 따라가 지나치는
    # 문제 자체가 구조적으로 안 생긴다. 중심선 탐색은 "이전 지점 ±0.15m 근방"으로만
    # 제한해서(넓게 잡으면 서쪽 완전히 다른 개활지로 새는 걸 확인함) 경로 연속성을
    # 보장했다.
    # [2026-08-24] 시작점(1.80,0.90)->(1.05,1.15) 구간(0.79m)이 나머지 세그먼트(전부
    # ~0.20m 간격)보다 4배 가까이 길어서, 위와 동일한 "긴 세그먼트에서 조향이
    # 못 따라잡고 벽을 스친다" 문제가 실행 시작 직후(사용자 확인, Gazebo 실측)
    # 재현됨 — 같은 해법(0.20m 간격 리샘플링)으로 이 구간도 직선상에 중간점
    # 3개를 추가로 끼워넣는다(시작점~(1.05,1.15) 구간은 열린 공간이라 직선
    # 보간으로 충분, distance_transform 재계산 불필요).
    ROUTE_BYPASS_CENTERLINE = {
        ('START', 'A'): [
            (1.61, 0.96),
            (1.42, 1.03),
            (1.24, 1.09),
            (1.05, 1.15),
            (0.90, 1.35),
            (0.75, 1.55),
            (0.60, 1.75),   # 병목 진짜 중심선(가장 좁은 지점), 여유 0.70m대
            (0.68, 1.95),
            (0.78, 2.15),
            (0.88, 2.35),
            (0.93, 2.55),
            (0.93, 2.75),
            (0.93, 2.95),
            (0.93, 3.15),
            (0.98, 3.35),
            (0.98, 3.55),
            # [2026-08-24] (1.00,3.75, clearance 0.350m) -> (0.90,4.00, clearance
            # 0.412m)로 교체 — 실측 결과 (1.00,3.75) 근처(허용오차 0.15m 안에서
            # 도달한 실제 지점 (1.03,3.60), clearance 0.304m)가 Nav2 global costmap
            # 기준으로 "Starting point in lethal space"였다. 지도 조회로 주변
            # 후보들의 clearance를 비교해 가장 여유로운 지점으로 교체.
            (0.90, 4.00),
        ],
    }
    # [2026-08-24] 세그먼트가 전부 짧고(0.20m 간격) 촘촘한 중심선이라, 앞부분 몇 개만
    # 골라 타이트하게/느슨하게 나누던 예전 방식 대신 마지막 직전까지 전부 같은
    # 기준(0.20m)으로 통일 — 짧은 구간에서 느슨한 허용오차(0.45m 이상)를 쓰면
    # 다음 세그먼트를 시작하기도 전에 이미 "도달"로 오판될 수 있다.
    ROUTE_BYPASS_NARROW_COUNT = {
        ('START', 'A'): 16,  # 마지막(0.90,4.00) 제외 전부(진입정렬 4개 이후부터)
    }
    # [2026-08-24] 시작점~(1.05,1.15) 사이 새로 추가한 중간점 3개 + 기존 진입각
    # 보정 지점(1.05,1.15) = 4개는 전부 열린 공간(위 ROUTE_BYPASS_CENTERLINE 주석
    # 참고) — 타이트한 허용오차를 주면 idx==0에서 겪었던 것과 같은 맴돌이가
    # 재현될 수 있어 이 4개 전부 느슨한 허용오차를 쓴다.
    ROUTE_BYPASS_ENTRY_COUNT = {
        ('START', 'A'): 4,
    }
    # [2026-08-24] AMCL 국소함정은 병목 진입~진짜 병목 통과 직후(idx 0~4,
    # (0.68,1.95)까지)에만 있었다 — 그 이후(전환/직선구간)는 AMCL로 복귀해 데드레커닝
    # 순수적분 누적오차를 피한다(실측: 긴 구간 전체를 데드레커닝에 맡기면 Nav2 인계
    # 시점에 실제 위치와 최대 1m 어긋남 확인됨).
    ROUTE_BYPASS_DR_COUNT = {
        ('START', 'A'): 8,  # 새로 추가된 중간점 3개만큼 기존 5에서 +3
    }

    def _run_route_bypass(self, from_leg: str, to_leg: str):
        """[2026-08-22 도입, 2026-08-24 일반화] (from_leg, to_leg) 구간이
        ROUTE_BYPASS_CENTERLINE에 등록돼 있으면 그 병목 구간을 Nav2/코스트맵 없이 AMCL
        피드백만으로 직접 통과한다. run()의 레그 루프에서 매 레그 시작 전(send_goal()
        전) 블로킹으로 한 번 호출됨 — publish_initial_pose_until_amcl_ready()와 동일한
        "이 노드가 직접 spin하며 대기" 패턴. 테이블에 없는 구간이거나 파라미터로 꺼져
        있으면 즉시 반환(no-op, Nav2에게 통째로 맡김).

        전체 구간에 timeout_sec 하나를 공유(중간 지점 하나에서 오래 막히면 이후
        지점들은 못 거치고 그대로 Nav2로 넘어간다 — 기존 '통째로 실패하면 그냥 진행'
        정책과 동일)."""
        if not self.get_parameter('corridor_bypass_enable').value:
            return
        waypoints = self.ROUTE_BYPASS_CENTERLINE.get((from_leg.upper(), to_leg.upper()))
        if not waypoints:
            return

        xy_tol = self.get_parameter('corridor_bypass_xy_tolerance').value
        speed_param = self.get_parameter('corridor_bypass_speed').value
        timeout_sec = self.get_parameter('corridor_bypass_timeout_sec').value

        while (self._cur_pose is None or self._odom_msg is None) and rclpy.ok():
            self._spin_once_safe(timeout_sec=0.2)
        if self._cur_pose is None:
            return

        # [2026-08-24] 이 구간 전체는 AMCL이 아니라 데드레커닝(_dr_pose())으로
        # 진행한다 — 위 __init__/_dr_set_anchor() 주석 참고. 여기서 딱 한 번 AMCL을
        # 기준점으로 신뢰하고(방금 publish_initial_pose_until_amcl_ready()로 확인된
        # 직후라 신뢰 가능), 이후로는 그 기준점 + odom 상대이동만 쓴다.
        self._dr_set_anchor()
        cx0, cy0, _ = self._dr_pose()
        self.get_logger().info(
            f'병목 구간 우회 시작(데드레커닝 기준, {from_leg}->{to_leg}) — 현재=({cx0:.2f},{cy0:.2f}), '
            f'중심선 웨이포인트 {len(waypoints)}개 통과 예정')
        self._last_direct_linear_x = 0.0
        self._last_direct_cmd_t_ns = None
        self._last_direct_angular_z = 0.0
        self._last_direct_angular_t_ns = None

        deadline = self._wall_clock.now().nanoseconds + int(timeout_sec * 1e9)
        # 중간 지점은 "지나가기만" 하면 되므로 정밀 정지가 필요 없다 — xy_tol 그대로 쓰면
        # 근처에서 헤딩만 미세조정하느라 시간을 낭비할 수 있어 느슨하게(3배) 잡는다.
        # 마지막 지점만 원래 tolerance를 그대로 쓴다. [2026-08-24] 단, 폭 0.7~1m
        # 병목 통로 구간(ROUTE_BYPASS_NARROW_COUNT)만은 예외로 0.20m까지 좁힌다 —
        # 위 ROUTE_BYPASS_CENTERLINE 주석의 "벽 픽셀 실측 확인" 참고.
        narrow_n = self.ROUTE_BYPASS_NARROW_COUNT.get((from_leg.upper(), to_leg.upper()), 0)
        dr_n = self.ROUTE_BYPASS_DR_COUNT.get((from_leg.upper(), to_leg.upper()), 0)
        # [2026-08-24] 원래 "idx==0만 느슨"이었는데, 시작점 근처에 열린 공간
        # 진입정렬 웨이포인트가 여러 개(entry_n개) 있는 구간도 생겨서 일반화.
        entry_n = self.ROUTE_BYPASS_ENTRY_COUNT.get((from_leg.upper(), to_leg.upper()), 1)
        for idx, (wx, wy) in enumerate(waypoints):
            is_final = (idx == len(waypoints) - 1)
            if is_final:
                seg_tol = xy_tol
            elif idx < entry_n:
                # [2026-08-24] 맨 앞 진입각 보정 지점(들)은 실제 장애물까지
                # 0.585m 여유가 있는 완전히 열린 공간이다(지도 조회로 확인) — 그런데
                # 타이트한 허용오차(0.20m)를 적용했더니 차가 0.20~0.21m 경계에서
                # 못 넘어가고 계속 맴돌아, 여기서만 14초(전체 25초 예산의 절반 이상)를
                # 날리는 걸 실측 확인함. 이 지점들은 좁은 통로가 아니라 사전 정렬용
                # 여유공간이라 느슨한 허용오차로 되돌린다 — 진짜 좁은 구간(idx>=entry_n)만
                # 타이트하게 유지.
                seg_tol = max(xy_tol * 3.0, 0.25)
            elif idx < narrow_n:
                seg_tol = max(xy_tol * 1.3, 0.20)
            else:
                seg_tol = max(xy_tol * 3.0, 0.25)
            # [2026-08-24] 데드레커닝(_dr_pose)은 순수 적분이라 시간이 지날수록
            # 오차가 누적된다 — 실측 확인: 병목 진입부(narrow_n 구간)를 지나 긴
            # 직선구간(마지막 웨이포인트)까지 40초 넘게 전부 데드레커닝만 믿게 했더니,
            # Nav2 인계 시점에 실제(AMCL) 위치가 데드레커닝 추정과 약 1m 어긋나 있었다
            # (Nav2가 그 실제 위치 기준으로 lethal space 판정). AMCL 국소함정은
            # narrow_n 구간(병목 진입~통과 직후)에만 있었으므로, 그 이후(먼 직선구간)는
            # 다시 AMCL을 신뢰해 누적오차를 원천 차단한다.
            use_dr = idx < dr_n
            # [2026-08-24] idx==dr_n(막 DR에서 AMCL로 넘어가는 바로 그 지점)에서,
            # 넘겨받을 AMCL이 "국소함정"에 빠져 있으면 이 시점 그대로 dist/heading_err가
            # 크게 튀는 게 실측 확인됨(위 _resync_amcl_to 주석 참고) — 넘어가기 직전
            # 신뢰할 수 있는 데드레커닝 추정치로 AMCL을 강제 재정렬시켜 이 점프를 막는다.
            if idx == dr_n and dr_n > 0:
                rx, ry, ryaw = self._dr_pose()
                self._resync_amcl_to(rx, ry, ryaw, timeout_sec=4.0)
            reached = self._drive_corridor_segment(wx, wy, seg_tol, speed_param, deadline, use_dr=use_dr)
            if not reached:
                return  # 타임아웃 — 남은 지점은 건너뛰고 Nav2로 넘어감(기존 정책과 동일)
        self.get_logger().info('병목 구간 통과 완료 — 모든 중심선 웨이포인트 도달')

    def _drive_corridor_segment(self, wx, wy, xy_tol, speed_param, deadline_ns, use_dr=True) -> bool:
        """_run_route_bypass()의 한 웨이포인트 구간을 주행 — 도달하면 True, 공유
        deadline_ns를 넘기면 False를 반환한다(정지 감지+backoff 로직은 기존과 동일,
        구간이 바뀔 때마다 진행추적 상태를 리셋한다). [2026-08-24] use_dr=True면
        AMCL 대신 _dr_pose()(데드레커닝, _run_route_bypass 주석 참고)를 쓴다 — 병목
        진입부(narrow_n)에만 적용, 그 이후 긴 직선구간은 데드레커닝 누적오차를
        피하려 다시 AMCL(use_dr=False)을 쓴다."""
        pose_fn = self._dr_pose if use_dr else (lambda: self._cur_pose)
        progress_pose = pose_fn()
        progress_t = self._wall_clock.now().nanoseconds
        backoff_until = None
        STUCK_MOVE_M = 0.03
        STUCK_TIMEOUT_SEC = 2.0
        BACKOFF_SEC = 1.5
        BACKOFF_SPEED = 0.08

        # _docking_arc_control()은 self.get_parameter('final_approach_speed')를 속도
        # 상한으로 쓰므로, 이 구간 전용 속도(corridor_bypass_speed)를 쓰려면 그 파라미터
        # 값을 잠깐 덮어쓸 수 없다(rclpy Parameter는 코드에서 임의로 재선언 못 함) —
        # 대신 원호 제어를 여기서 직접 계산한다(_docking_arc_control과 동일 로직, 속도
        # 파라미터만 다름).
        kp = self.get_parameter('docking_yaw_kp').value
        MIN_SAFE_RADIUS_M = 0.25

        while rclpy.ok():
            self._spin_once_safe(timeout_sec=0.05)
            dr = pose_fn()
            if dr is None:
                continue
            cx, cy, cyaw = dr
            dist = math.hypot(wx - cx, wy - cy)
            now_ns = self._wall_clock.now().nanoseconds

            if dist <= xy_tol:
                return True
            if now_ns >= deadline_ns:
                self._cmd_vel_pub.publish(Twist())
                self.get_logger().warn(
                    f'병목 구간 우회 타임아웃 — 목표=({wx:.2f},{wy:.2f}) 남은거리={dist:.2f}m, '
                    f'그래도 Nav2로 계속 진행')
                return False

            px, py, _ = progress_pose
            if math.hypot(cx - px, cy - py) >= STUCK_MOVE_M:
                progress_pose = dr
                progress_t = now_ns
            elif backoff_until is None and (now_ns - progress_t) / 1e9 >= STUCK_TIMEOUT_SEC:
                self.get_logger().warn(
                    f'병목 구간 우회: {STUCK_TIMEOUT_SEC:.0f}초 진행없음 — 후진으로 탈출 시도')
                backoff_until = now_ns + int(BACKOFF_SEC * 1e9)

            if backoff_until is not None:
                if now_ns < backoff_until:
                    # [2026-08-24] 후진이 허용되므로, 막혔을 때(대개 전방에 뭔가 있다는
                    # 뜻) 전진 우회보다 후진으로 물러나는 쪽이 더 확실하게 빠져나온다.
                    cmd = Twist()
                    cmd.linear.x = -BACKOFF_SPEED
                    cmd.angular.z = self._lidar_steer_bias(-1.0)
                    self._cmd_vel_pub.publish(cmd)
                    continue
                backoff_until = None
                progress_pose = pose_fn()
                progress_t = now_ns

            heading_err = normalize_angle(math.atan2(wy - cy, wx - cx) - cyaw)
            speed = max(speed_param * 0.4, min(speed_param, dist * 0.6))
            speed = self._front_safety_speed_cap(speed, 1.0)
            speed = self._ramp_linear_speed(speed)
            # [2026-08-24] max_angular = speed/MIN_SAFE_RADIUS_M라서 속도를 올릴수록
            # 허용 조향각속도도 같이 커진다 — corridor_bypass_speed를 0.35->0.45로
            # 올린 뒤 max_angular이 1.57rad/s까지 커져서, 크고 빠른 좌우 조향이
            # 반복되며 지그재그로 상쇄되는 걸 실측 확인함(20초간 x=1.5~1.6,y=0.87에서
            # 거의 못 움직임, heading_err도 -70도에서 거의 안 줄어듦). 속도와 무관한
            # 절대 상한(ABS_MAX_ANGULAR)을 별도로 둬서, 직진 구간은 빠르게 가되 급격한
            # 큰 회전 자체는 못 하게 막는다.
            ABS_MAX_ANGULAR = 1.0
            max_angular = min(speed / MIN_SAFE_RADIUS_M, ABS_MAX_ANGULAR)
            # [2026-08-24] 헤딩오차가 클 때(큰 방향전환이 급함) 라이다 바이어스를
            # 그대로 더하면 두 신호가 서로 부호가 다를 때 상쇄/증폭을 반복해서 매틱
            # 최대좌<->최대우로 널뛰는 진동이 생기는 걸 실측 확인함(corridor_bypass_speed
            # 상향 후 dist=0.8~0.9대에서 22틱 연속 진행 없이 진동만 함, 2026-08-24).
            # 헤딩오차가 이미 작을 때(거의 정렬됨, 벽 스치기만 방지하면 되는 상황)만
            # 라이다를 우선시키고, 오차가 크면(45도 이상) 헤딩추종이 항상 우선이어야
            # 큰 방향전환 자체를 못 하고 지나쳐버리거나 진동하는 문제가 안 생긴다.
            # [2026-08-24] 위 조건부 가중치로도 못 잡힌 진동의 진짜 원인을 찾음 —
            # 실측 결과 이 지점(장애물까지 1.5m 이상 여유, 지도로 확인)에서도
            # lidar_bias가 매틱 큰 값<->0 사이를 오가서, 조향 램프가 매번 초기화되다시피
            # 하며 10초간 명령은 계속 -1.0rad/s인데 실제 헤딩은 17도밖에 안 변하는
            # 것까지 확인함(명령의 약 1/30). 애초에 이 라이다 조향보정은 AMCL 오차
            # 보정용으로 넣은 것인데, 이제 이 구간 전체가 데드레커닝(_dr_pose)을 쓰므로
            # 그 문제 자체가 해소됐다 — 병목구간에서는 조향보정을 빼고 헤딩추종만
            # 쓴다(감속용 _front_safety_speed_cap은 그대로 유지, 그건 라이다 정면
            # 거리만 보고 진동 소스가 아니었음).
            angular_z = kp * heading_err
            self.get_logger().info(
                f'[진단] 병목우회 cur=({cx:.2f},{cy:.2f},{math.degrees(cyaw):.0f}deg) target=({wx:.2f},{wy:.2f}) '
                f'dist={dist:.2f} heading_err={math.degrees(heading_err):.0f}deg speed={speed:.2f} '
                f'angular_z(clamp전)={angular_z:.2f} max_angular={max_angular:.2f}',
                throttle_duration_sec=0.5)
            angular_z = max(-max_angular, min(max_angular, angular_z))
            angular_z = self._ramp_angular_speed(angular_z)

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = angular_z
            self._cmd_vel_pub.publish(cmd)
        return False

    def _start_docking(self):
        """Nav2 costmap 기반 계획을 벗어나, 목표까지 남은 짧은 구간을 AMCL pose
        피드백만으로 직접 /cmd_vel을 publish해서 마무리한다(위 __init__ 주석 참고).
        costmap 충돌검사를 아예 안 하므로(그게 이 우회로의 목적) 장애물 인식이
        전혀 없다 — _docking_control_loop()의 정지감지+전진탈출(후진 없음)이 유일한
        방어선."""
        self.get_logger().info('정밀 접근 반경 진입 — Nav2 목표 취소하고 직접 제어로 전환')
        self._docking_active = True
        self._last_direct_linear_x = 0.0
        self._last_direct_cmd_t_ns = None
        self._last_direct_angular_z = 0.0
        self._last_direct_angular_t_ns = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        timeout_sec = self.get_parameter('docking_timeout_sec').value
        self._docking_deadline = self._wall_clock.now().nanoseconds + int(timeout_sec * 1e9)
        # ── 정지 감지 상태 ──
        # [2026-08-20] 도킹 루프엔 costmap 충돌검사가 없어서, 실제로 벽에 밀어붙인 채
        # 명령만 계속 나가는(바퀴는 헛돔) 상황이 지속되는 걸 실측 확인함(헤딩 계산
        # 자체는 문제 없었는데 물리적으로 못 움직이고 있었음). 일정 시간 동안 실제
        # 위치 변화가 없으면 "막혔다"로 보고 후진으로 빠져나온다(2026-08-24: 후진
        # 허용 확인됨에 따라 전진전용 탈출에서 원복).
        self._docking_progress_pose = self._cur_pose
        self._docking_progress_t = self._wall_clock.now().nanoseconds
        self._docking_backoff_until = None

        # K턴 상태도 레그마다 새로 시작(이전 레그에서 활성화된 채 남아있으면 안 됨).
        self._docking_k_turn_active = False
        self._docking_k_turn_dir_sign = 0.0
        self._docking_k_turn_phase_direction = 1.0
        self._docking_k_turn_phase_deadline_ns = None

        # [2026-08-22] target_heading 모드(점 추종 vs 고정 goal_yaw)를 매 틱 "현재" 거리로
        # 재판정하던 것을 도킹 진입 시점 1회로 고정. 이 근접 거리대의 점 추종 베어링은
        # 위치가 조금만 바뀌어도 100도 이상 요동치는 게 실측돼 있어서(아래
        # _docking_arc_control 근처 주석 참고), final_approach_radius(도킹 진입 문턱) <
        # close_radius(항상 +0.05m 이상 크게 설계됨)이므로 도킹 진입 순간엔 항상 "고정
        # goal_yaw" 조건을 만족한다 — 이 판정을 진입 시점에 굳혀서, 이후 일시적으로
        # 멀어져도 점 추종으로 되돌아가지 않게 한다.
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

        now_ns = self._wall_clock.now().nanoseconds
        if now_ns >= self._docking_deadline:
            self.get_logger().error(
                f'정밀 접근 타임아웃 — 남은거리={dist:.2f}m, 헤딩오차={math.degrees(yaw_err_to_goal):.1f}deg')
            self._finish_docking(success=False)
            return

        # ── 정지 감지: STUCK_TIMEOUT_SEC 동안 위치 변화가 없으면 막힌 것으로 보고
        #    BACKOFF_SEC 동안 후진하며 빠져나온다(2026-08-24: 후진 허용 확인에 따라
        #    전진전용 탈출에서 원복). K턴 모드 중에도 이 정지감지는 그대로 유효 —
        #    K턴 자체가 전후진을 번갈아 매 위치 변화를 만들어내므로, 진짜로 막힌
        #    경우(K턴으로도 위치가 전혀 안 바뀜)와 정상 K턴 진행을 여전히 잘 구분한다.
        STUCK_MOVE_M = 0.03
        STUCK_TIMEOUT_SEC = 2.0
        BACKOFF_SEC = 1.5
        BACKOFF_SPEED = 0.08

        px, py, _ = self._docking_progress_pose
        progressed = math.hypot(cx - px, cy - py) >= STUCK_MOVE_M
        if progressed:
            self._docking_progress_pose = self._cur_pose
            self._docking_progress_t = now_ns
        elif self._docking_backoff_until is None and \
                (now_ns - self._docking_progress_t) / 1e9 >= STUCK_TIMEOUT_SEC:
            self.get_logger().warn(
                f'{STUCK_TIMEOUT_SEC:.0f}초 이상 진행 없음 — 장애물에 막힌 것으로 보고 '
                f'{BACKOFF_SEC:.1f}초간 반대쪽으로 최대 조향하며 저속 전진 탈출 시도')
            self._docking_backoff_until = now_ns + int(BACKOFF_SEC * 1e9)

        if self._docking_backoff_until is not None:
            if now_ns < self._docking_backoff_until:
                # [2026-08-24] direction=-1(후진) 기준으로 가장 가까운 장애물의
                # 반대쪽으로 미는 바이어스를 조향각으로 써서 후진 탈출한다 — 막힌
                # 원인은 대개 전방(도킹은 항상 전진/K턴 위주라 정면에 뭔가 있을 가능성이
                # 큼)이므로 후진이 전진 우회보다 확실하게 빠져나온다.
                cmd = Twist()
                cmd.linear.x = -BACKOFF_SPEED
                cmd.angular.z = self._lidar_steer_bias(-1.0)
                self._cmd_vel_pub.publish(cmd)
                return
            # 탈출 시도 종료 — 정지 감지 타이머 리셋하고 다시 정상 제어로 복귀.
            self._docking_backoff_until = None
            self._docking_progress_pose = self._cur_pose
            self._docking_progress_t = now_ns

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

        # [2026-08-24] 후진이 완전히 허용됨이 확인되어, 헤딩오차가 클 때(예: 목표가
        # 차량 거의 정반대 방향) 전진 원호만으로 느리게 수렴시키는 대신 K턴(전진-한쪽
        # 조향/후진-반대조향을 번갈아 실행)으로 더 빠르게 좁힌다 — 헤딩오차가 작아지면
        # 다시 전진 원호 제어로 전환한다(_docking_arc_control 내부 분기 참고).
        cmd = self._docking_arc_control(heading_err, dist)
        self._cmd_vel_pub.publish(cmd)

    def _ramp_linear_speed(self, target_speed, max_accel=0.3):
        """직접제어(도킹/병목우회) 공용 속도 램프. [2026-08-22] 목표속도를 한 틱에
        그대로 명령했더니 실제 이동속도가 명령의 4.3%(0.15m/s 명령 -> 실측 0.006m/s)
        밖에 안 나오는 걸 /gazebo/odom 위치추적으로 확인함 — 반면 Nav2 컨트롤러가
        보내는, 0->목표속도까지 여러 틱에 걸쳐 부드럽게 램프업되는 명령 패턴에서는
        훨씬 잘 따라갔다(같은 구간 재실측, 명령 재개 후 평균 0.04m/s대로 개선). 이
        차의 속도 PID(urdf/xycar.urdf ackermann_drive)가 계단형 목표치 변화를 잘 못
        따라가는 것으로 보여, 직접제어 쪽에서도 매틱 max_accel(m/s^2)만큼만 속도를
        바꾸도록 흉내낸다."""
        now_ns = self._wall_clock.now().nanoseconds
        dt = 0.1 if self._last_direct_cmd_t_ns is None else \
            max(0.0, (now_ns - self._last_direct_cmd_t_ns) / 1e9)
        self._last_direct_cmd_t_ns = now_ns
        max_delta = max_accel * dt
        current = self._last_direct_linear_x
        if target_speed > current:
            new_speed = min(target_speed, current + max_delta)
        else:
            new_speed = max(target_speed, current - max_delta)
        self._last_direct_linear_x = new_speed
        return new_speed

    def _ramp_angular_speed(self, target_angular, max_alpha=2.5):
        """_ramp_linear_speed()와 동일한 이유로 조향(angular.z)에도 램프를 건다 —
        [2026-08-24] 헤딩오차가 부호를 뒤집을 때마다 clamp 상한(max_angular)까지
        한 틱만에 튀는 계단형 명령이 실제로는 "타이어가 와리가리 흔들림"으로 나타남을
        사용자가 시뮬레이션에서 직접 관찰해 확인. max_alpha(rad/s^2)만큼만 매틱
        바꾸도록 제한한다. _last_direct_cmd_t_ns(속도 램프용)는 같은 틱 안에서 이미
        _ramp_linear_speed()가 먼저 갱신해버리므로(dt=0이 돼 조향이 그대로 얼어붙음),
        별도 타임스탬프(_last_direct_angular_t_ns)를 쓴다."""
        now_ns = self._wall_clock.now().nanoseconds
        dt = 0.1 if self._last_direct_angular_t_ns is None else \
            max(0.0, (now_ns - self._last_direct_angular_t_ns) / 1e9)
        self._last_direct_angular_t_ns = now_ns
        max_delta = max_alpha * dt
        current = self._last_direct_angular_z
        if target_angular > current:
            new_angular = min(target_angular, current + max_delta)
        else:
            new_angular = max(target_angular, current - max_delta)
        self._last_direct_angular_z = new_angular
        return new_angular

    # [2026-08-24] 헤딩오차가 이 이상이면 K턴(후진 포함) 모드로 진입, 이 이하로 줄면
    # 전진 원호 모드로 복귀한다. 두 값 사이 히스테리시스를 둬서 경계 근방에서 매 틱
    # 모드가 왔다갔다 하는 걸 막는다(과거 point-tracking bearing 요동으로 dir_sign이
    # 매번 뒤집히던 것과 같은 종류의 문제 재발 방지).
    K_TURN_ENTER_RAD = math.radians(70)
    K_TURN_EXIT_RAD = math.radians(45)

    def _docking_arc_control(self, heading_err, dist):
        """헤딩오차 크기에 따라 전진 원호 제어(작을 때)와 K턴(클 때, 후진 포함)을
        분기한다 — 2026-08-24, 후진 허용 확인에 따라 K턴 복원."""
        if not self._docking_k_turn_active and abs(heading_err) > self.K_TURN_ENTER_RAD:
            self._docking_k_turn_active = True
            self._docking_k_turn_dir_sign = 1.0 if heading_err > 0 else -1.0
            self._docking_k_turn_phase_direction = 1.0
            self._docking_k_turn_phase_deadline_ns = None  # 아래에서 즉시 phase 시작

        if self._docking_k_turn_active:
            if abs(heading_err) <= self.K_TURN_EXIT_RAD:
                self._docking_k_turn_active = False
            else:
                return self._docking_k_turn_control()

        return self._docking_forward_arc_control(heading_err, dist)

    def _docking_k_turn_control(self):
        """K턴 한 phase를 실행 — dir_sign(조향 방향)은 K턴 진입 시 1회 고정되고
        (_docking_arc_control 참고), phase_direction(+1=전진/-1=후진)만 K_TURN_PHASE_SEC
        마다 뒤집힌다. 전진 phase에는 dir_sign 쪽으로, 후진 phase에는 그 반대쪽으로
        조향해야 두 phase가 서로 상쇄되지 않고 같은 방향 회전이 누적된다 — 실제 K턴
        주차(예: 전진-좌 다음 후진-우를 해야 좌회전이 쌓인다)와 동일한 원리."""
        K_TURN_PHASE_SEC = 1.0
        K_TURN_SPEED = 0.15
        MIN_SAFE_RADIUS_M = 0.25

        now_ns = self._wall_clock.now().nanoseconds
        if (self._docking_k_turn_phase_deadline_ns is None
                or now_ns >= self._docking_k_turn_phase_deadline_ns):
            self._docking_k_turn_phase_direction *= -1.0
            self._docking_k_turn_phase_deadline_ns = now_ns + int(K_TURN_PHASE_SEC * 1e9)

        direction = self._docking_k_turn_phase_direction
        steer_sign = (self._docking_k_turn_dir_sign if direction > 0
                      else -self._docking_k_turn_dir_sign)

        target_speed = direction * self._front_safety_speed_cap(K_TURN_SPEED, direction)
        speed = self._ramp_linear_speed(target_speed)

        max_angular = K_TURN_SPEED / MIN_SAFE_RADIUS_M
        angular_z = steer_sign * max_angular
        angular_z += self._lidar_steer_bias(direction)
        angular_z = self._ramp_angular_speed(angular_z)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular_z
        return cmd

    def _docking_forward_arc_control(self, heading_err, dist):
        """전진 + 원호 P제어로 헤딩을 맞춘다 — 헤딩오차가 K_TURN_ENTER_RAD 이하일
        때만 호출됨(그보다 크면 _docking_k_turn_control이 담당)."""
        direction = 1.0
        steer_err = heading_err

        kp = self.get_parameter('docking_yaw_kp').value
        approach_speed = self.get_parameter('final_approach_speed').value
        speed = max(approach_speed * 0.4, min(approach_speed, dist * 0.6))
        speed = self._front_safety_speed_cap(speed, direction)
        speed = self._ramp_linear_speed(speed)

        # [2026-08-20] kp*steer_err가 크면(헤딩오차가 클수록) 회전반경이 차체
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
        angular_z = self._ramp_angular_speed(angular_z)

        cmd = Twist()
        cmd.linear.x = direction * speed
        cmd.angular.z = angular_z  # direction 안 곱함(_docking_arc_control 도입 전 주석과 동일 이유)
        return cmd

    def _finish_docking(self, success: bool):
        if self._docking_timer is not None:
            self._docking_timer.cancel()
            self._docking_timer = None
        self._cmd_vel_pub.publish(Twist())  # 정지
        if success:
            self.get_logger().info(
                f"레그 '{self._current_leg}' 완료: 정밀 접근으로 목표 pose에 도달했습니다.")
        self._leg_success = success
        self._leg_done = True

    def _result_cb(self, future):
        if self._docking_active:
            # 이 결과는 _start_docking()이 스스로 취소한 목표에 대한 것 — 최종 성패는
            # _finish_docking()이 정한다. 여기서 _leg_done을 건드리면 그 결과를 덮어쓴다.
            status = future.result().status
            self.get_logger().info(f'(정밀 접근 전환으로 취소된 Nav2 목표 결과: status={status})')
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"레그 '{self._current_leg}' 완료: 목표 pose에 도달했습니다.")
            self._leg_success = True
        else:
            self.get_logger().error(f"레그 '{self._current_leg}' 실패 (status={status}).")
            self._leg_success = False
        self._leg_done = True

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
        """미션 전체를 레그 순서대로 수행한다(2026-08-24 재작성). 기본값('FULL')이면
        MISSION_LEGS(출발→A→B→출발복귀)를 전부 순회하고, parking_zone 파라미터로
        개별 레그 이름(예: 'A')을 주면 그 레그 하나만 실행한다(디버깅용, 기존 단일
        레그 동작과 동일).

        레그 하나가 실패(타임아웃/거부)해도 다음 레그로 계속 진행한다 — 경기 규정상
        한 구역을 놓쳤다고 나머지를 포기하는 것보다 남은 구역이라도 시도하는 쪽이
        유리하다(부분 감점 vs 전체 포기)."""
        zone_param = self.get_parameter('parking_zone').value.upper()
        legs = self.MISSION_LEGS if zone_param == 'FULL' else [zone_param]

        if self.get_parameter('set_initial_pose').value:
            wait_sec = self.get_parameter('initial_pose_wait_sec').value
            deadline = self._wall_clock.now().nanoseconds + int(wait_sec * 1e9)
            while (self._initial_pose_pub.get_subscription_count() == 0
                   and self._wall_clock.now().nanoseconds < deadline):
                self._spin_once_safe(timeout_sec=0.2)
            self.publish_initial_pose_until_amcl_ready(wait_sec)

        self.get_logger().info('navigate_to_pose 액션 서버 대기 중...')
        self._nav_client.wait_for_server()

        mission_deadline_sec = self.get_parameter('mission_deadline_sec').value
        mission_deadline_ns = self._wall_clock.now().nanoseconds + int(mission_deadline_sec * 1e9)

        leg_results = {}
        prev_leg = 'START'  # 초기 pose 발행 직후이므로 "출발지에 있다"고 본다.
        for leg in legs:
            if self._wall_clock.now().nanoseconds >= mission_deadline_ns:
                self.get_logger().error(
                    f"전체 미션 제한시간({mission_deadline_sec:.0f}초) 초과 — "
                    f"레그 '{leg}' 이후는 건너뛰고 즉시 정지")
                self._cmd_vel_pub.publish(Twist())
                break

            # Nav2에 목표를 보내기 전에, 이 구간에 알려진 병목이 있으면 먼저 직접
            # 통과시킨다(위 ROUTE_BYPASS_CENTERLINE 참고, 없으면 즉시 반환).
            self._run_route_bypass(prev_leg, leg)

            self._retry_count = 0
            self.send_goal(leg)

            while rclpy.ok() and not self._leg_done:
                if self._wall_clock.now().nanoseconds >= mission_deadline_ns:
                    self.get_logger().error(
                        f"전체 미션 제한시간({mission_deadline_sec:.0f}초) 초과 — "
                        f"레그 '{leg}' 강제 종료")
                    if self._goal_handle is not None:
                        self._goal_handle.cancel_goal_async()
                    if self._docking_timer is not None:
                        self._docking_timer.cancel()
                        self._docking_timer = None
                    self._cmd_vel_pub.publish(Twist())
                    self._leg_done = True
                    self._leg_success = False
                    break
                self._spin_once_safe(timeout_sec=0.2)

            leg_results[leg] = self._leg_success
            self.get_logger().info(
                f"레그 '{leg}' 결과: {'성공' if self._leg_success else '실패'} — 다음 레그로 진행")
            prev_leg = leg

        self.get_logger().info(f'미션 종료 — 레그별 결과: {leg_results}')
        return bool(leg_results) and all(leg_results.values())


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
