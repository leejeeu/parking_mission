#=============================================
# cmd_vel_bridge.py — Nav2 controller_server가 발행하는 /cmd_vel(geometry_msgs/Twist)을
#   xycar 실차 모터 인터페이스(std_msgs/Float32MultiArray[angle_deg, speed_unit], 토픽명
#   'xycar_motor')로 변환한다.
#
# 왜 필요한가: 이 차량은 표준 diff-drive cmd_vel을 그대로 못 받는다 — UMK(track_drive.py
#   drive())가 쓰는 실제 모터 인터페이스는 [조향각(도), 속도(0~100 커맨드 단위)] 2요소
#   배열이다(differential wheel speed가 아니라 애커먼만/자전거 모델 직접 명령). Nav2
#   controller_server(DWB 등)는 Twist(linear.x m/s, angular.z rad/s)만 내보내므로 그 사이를
#   이 노드가 변환한다.
#
# 변환식 (자전거 모델): angular_z = v * tan(steer) / L  =>  steer = atan2(angular_z * L, v).
#   v=0 근처에서도 atan2는 안전하게 ±90도 근방을 반환(어차피 ANGLE_MAX로 clip됨).
#
# 부호규약: ROS 표준(REP103)은 angular.z > 0이 반시계(좌회전)인데, UMK 프로젝트는 반대로
#   음수가 좌회전(config.py TURN_ANGLE=-60 주석 "좌회전"). 그래서 부호를 뒤집는다.
#
# 안전장치: /cmd_vel이 CMD_VEL_TIMEOUT_SEC 이상 안 오면(예: Nav2가 멈췄거나 프로세스가 죽음)
#   watchdog 타이머가 speed=0을 강제 발행한다 — "마지막 명령을 계속 유지"가 더 위험하다.
#=============================================
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from .vehicle_spec import WHEELBASE_M, METERS_PER_SPEED_UNIT, ANGLE_MAX_DEG

CMD_VEL_TIMEOUT_SEC = 0.5
WATCHDOG_PERIOD_S = 0.1


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self._motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self._last_cmd_t = None

        self.create_timer(WATCHDOG_PERIOD_S, self._watchdog)

        self.get_logger().info(
            f'cmd_vel_bridge 초기화 완료 | wheelbase={WHEELBASE_M}m, '
            f'm_per_speed_unit={METERS_PER_SPEED_UNIT}, angle_max={ANGLE_MAX_DEG}deg')

    def _cb_cmd_vel(self, msg: Twist):
        self._last_cmd_t = self.get_clock().now()
        self._publish_motor(msg.linear.x, msg.angular.z)

    def _publish_motor(self, linear_x, angular_z):
        steer_rad = math.atan2(angular_z * WHEELBASE_M, linear_x)
        angle_deg = -math.degrees(steer_rad)   # UMK 부호규약(음수=좌회전)에 맞춰 반전
        angle_deg = max(-ANGLE_MAX_DEG, min(ANGLE_MAX_DEG, angle_deg))

        speed_unit = linear_x / METERS_PER_SPEED_UNIT if METERS_PER_SPEED_UNIT else 0.0
        speed_unit = max(-100.0, min(100.0, speed_unit))

        msg = Float32MultiArray()
        msg.data = [float(angle_deg), float(speed_unit)]
        self._motor_pub.publish(msg)

    def _watchdog(self):
        if self._last_cmd_t is None:
            return
        age_sec = (self.get_clock().now() - self._last_cmd_t).nanoseconds / 1e9
        if age_sec > CMD_VEL_TIMEOUT_SEC:
            self.get_logger().warn(
                '[cmd_vel_bridge] /cmd_vel 끊김 — 안전정지(speed=0) 발행', throttle_duration_sec=2.0)
            self._publish_motor(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
