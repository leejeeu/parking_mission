#=============================================
# motor_bridge.py — Gazebo 시뮬레이션 전용. 실차 하드웨어(모터/VESC)를 Gazebo
#   ackermann_drive 플러그인으로 대체하기 위한 양방향 변환 노드.
#
# 왜 필요한가: cmd_vel_bridge.py/odom_publisher.py/parking_navigator.py는 실차
#   인터페이스(xycar_motor 구독, /vesc_speed_erpm 구독)를 그대로 쓰도록 설계돼 있고,
#   이 시뮬레이션 작업은 그 노드들을 수정하지 않는 것이 목표다(시뮬에서 검증한 스택이
#   실차에서도 동일하게 동작해야 하므로). 그래서 실차 인터페이스 <-> Gazebo 인터페이스
#   사이를 여기서 중개한다:
#     xycar_motor(실차 모터 커맨드) -> /gazebo/cmd_vel(Twist, ackermann_drive 플러그인 입력)
#     /gazebo/odom(플러그인이 발행하는 물리 시뮬레이션 결과) -> /vesc_speed_erpm(실차 VESC와
#       동일한 units)
#
#   /cmd_vel, /odom 대신 /gazebo/cmd_vel, /gazebo/odom을 쓰는 이유: 그 이름들은 이미
#   Nav2 controller_server(/cmd_vel 발행)와 odom_publisher.py(/odom 발행)가 쓰고 있어서
#   그대로 재사용하면 발행자가 겹친다(xycar.urdf의 ackermann_drive 플러그인 쪽에서
#   remapping 처리).
#
# 변환식: cmd_vel_bridge.py의 변환(Twist -> xycar_motor)을 그대로 역산한다.
#   steer_rad = atan2(angular_z*L, v) 로 인코딩됐으므로, v!=0인 한
#   angular_z = v * tan(steer_rad) / L 로 복원 가능(atan2/tan은 이 범위에서 서로 역함수).
#   부호규약도 cmd_vel_bridge.py와 동일하게 반전(UMK: 음수 조향각=좌회전).
#
# 안전장치: xycar_motor가 CMD_TIMEOUT_SEC 이상 안 오면(예: 상위 노드가 죽음) 워치독이
#   /gazebo/cmd_vel에 0을 발행 — cmd_vel_bridge.py의 워치독과 동일한 설계 근거(마지막
#   명령을 계속 유지하는 게 더 위험하다).
#=============================================
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray

from ..vehicle_spec import WHEELBASE_M, METERS_PER_SPEED_UNIT, VESC_SPEED_TO_ERPM_GAIN

CMD_TIMEOUT_SEC = 0.5
WATCHDOG_PERIOD_S = 0.1


class MotorBridge(Node):

    def __init__(self):
        super().__init__('sim_motor_bridge')

        self._cmd_vel_pub = self.create_publisher(Twist, '/gazebo/cmd_vel', 10)
        self._vesc_pub = self.create_publisher(Float32, '/vesc_speed_erpm', 10)

        self.create_subscription(Float32MultiArray, 'xycar_motor', self._cb_xycar_motor, 10)
        self.create_subscription(Odometry, '/gazebo/odom', self._cb_gazebo_odom, 10)

        self._last_cmd_t = None
        self.create_timer(WATCHDOG_PERIOD_S, self._watchdog)

        self.get_logger().info(
            f'sim_motor_bridge 초기화 완료 | wheelbase={WHEELBASE_M}m, '
            f'm_per_speed_unit={METERS_PER_SPEED_UNIT}, vesc_gain={VESC_SPEED_TO_ERPM_GAIN}')

    def _cb_xycar_motor(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().error(
                f'[xycar_motor] data 길이 이상(len={len(msg.data)}) — 무시', throttle_duration_sec=2.0)
            return

        self._last_cmd_t = self.get_clock().now()
        angle_deg, speed_unit = msg.data[0], msg.data[1]
        self._publish_cmd_vel(angle_deg, speed_unit)

    def _publish_cmd_vel(self, angle_deg: float, speed_unit: float):
        linear_x = speed_unit * METERS_PER_SPEED_UNIT

        steer_rad = -math.radians(angle_deg)   # cmd_vel_bridge.py 부호반전의 역변환
        angular_z = linear_x * math.tan(steer_rad) / WHEELBASE_M

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(twist)

    def _cb_gazebo_odom(self, msg: Odometry):
        v_mps = msg.twist.twist.linear.x
        # ackermann_drive 플러그인이 정지 상태 근처에서 조향 PID 계산 중 순간적으로 NaN을
        # 내보내는 경우가 실측됨(2026-08-19, 시뮬 기동 직후 반복 확인) — 실제 VESC는 NaN을
        # 낼 수 없으므로 이 값이 그대로 odom_publisher.py까지 흘러가면 TF_NAN_INPUT으로
        # AMCL이 깨진다. 여기서 걸러서 0으로 대체(마지막 유효값 대신 0을 쓰는 이유는
        # odom_publisher.py의 "센서 죽음=v=0" 안전장치와 동일한 설계 근거).
        if not math.isfinite(v_mps):
            self.get_logger().warn(
                '[sim_motor_bridge] /gazebo/odom linear.x가 NaN/Inf — 0으로 대체', throttle_duration_sec=2.0)
            v_mps = 0.0
        # [2026-08-22] NaN은 아니지만 "크지만 유한한" 오버슈트도 걸러야 함을 실측 확인.
        # 이 차(3kg, 바퀴 관성 극소)의 ackermann_drive 속도 PID가 K턴처럼 linear.x 부호를
        # 급하게 뒤집는 명령을 받으면, 발산까지는 안 가도 순간적으로 물리적으로 말이 안 되는
        # 큰 속도(수 m/s대)를 짧게 내보내는 것을 실측으로 확인함 — 이 값이 필터링 없이
        # odom_publisher.py의 데드레커닝에 들어가면 그 찰나에 위치가 몇 미터씩 "순간이동"하고,
        # 이 가짜 이동량이 AMCL 모션모델(오도메트리 델타)에 그대로 들어가 파티클(위치추정)
        # 자체가 엉뚱한 곳으로 튀어버린다(로컬 코스트맵의 반복적인 "out of map bounds" 경고와
        # K턴 중 헤딩추정 요동의 실제 원인). 이 로봇의 실제 최대속도(주차 시나리오 기준)보다
        # 훨씬 넉넉한 2.0 m/s를 상한으로 클램프 — 정상 주행 속도는 절대 안 걸리고, PID
        # 오버슈트만 걸러낸다.
        MAX_PLAUSIBLE_V_MPS = 2.0
        if abs(v_mps) > MAX_PLAUSIBLE_V_MPS:
            self.get_logger().warn(
                f'[sim_motor_bridge] /gazebo/odom linear.x={v_mps:.2f}m/s — 물리적으로 비현실적'
                f'(PID 오버슈트로 추정), {MAX_PLAUSIBLE_V_MPS}m/s로 클램프',
                throttle_duration_sec=2.0)
            v_mps = math.copysign(MAX_PLAUSIBLE_V_MPS, v_mps)
        erpm_msg = Float32()
        erpm_msg.data = float(v_mps * VESC_SPEED_TO_ERPM_GAIN)
        self._vesc_pub.publish(erpm_msg)

    def _watchdog(self):
        if self._last_cmd_t is None:
            return
        age_sec = (self.get_clock().now() - self._last_cmd_t).nanoseconds / 1e9
        if age_sec > CMD_TIMEOUT_SEC:
            self.get_logger().warn(
                '[sim_motor_bridge] xycar_motor 끊김 — 안전정지(cmd_vel=0) 발행', throttle_duration_sec=2.0)
            self._publish_cmd_vel(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
