#=============================================
# odom_publisher.py — VESC 실측속도(v_mps) + IMU yaw로 /odom(nav_msgs/Odometry) 발행
#   + odom->base_link TF를 브로드캐스트하는 독립 노드.
#
# 왜 필요한가: Nav2 AMCL은 odom 모션모델(스캔 사이에 "얼마나 움직였는지" 델타)을 위해
#   반드시 odom 프레임 토픽 + TF가 필요하다. UMK(track_drive) 저장소엔 이 워크스페이스
#   전체에 TF/오도메트리가 아예 없어서(카메라/라이다/IMU만 구독) 이 패키지에서 자체적으로
#   만든다. UMK의 EncoderPoseEstimator(엔코더/VESC 데드레커닝)를 그대로 옮겨와 재사용한다
#   (localization/pose_estimator.py, vehicle_spec.py 참고 — 독립 패키지라 원본을 import하지
#   않고 복제했다. UMK track_drive/config.py 값이 바뀌면 vehicle_spec.py도 같이 갱신할 것).
#
# odom 원점이 매 실행마다 달라도 되는 이유: AMCL은 odom의 절대값이 아니라 이전 틱 대비
#   델타만 참조한다. 전역 위치(맵 기준)는 parking_navigator 노드가 /initialpose로 넣어주는
#   시작 pose + AMCL 자체 스캔매칭이 담당하므로, 이 노드는 매번 (0,0,0)에서 새로 적분해도 된다.
#
# yaw 소스가 IMU 고정인 이유: 데드레커닝(조향각 적분)을 쓰려면 현재 명령 조향각을 알아야
#   하는데, 이 노드는 조향 제어 노드(parking_navigator/controller_server)와 별개 프로세스라
#   접근이 없다. 대신 IMU 실측 yaw를 그대로 쓴다(x,y 위치 적분은 v_mps만 있으면 되므로
#   yaw 소스와 무관).
#
# ★실측 필요★ — base_link->laser 정적 TF(launch에서 별도로 잡음, 이 노드와 무관):
#   라이다 장착 위치(평행이동)는 UMK 저장소에서 한 번도 실측된 적이 없다
#   (measure_lidar_camera_offset.md 참고). launch/parking_mission.launch.py의
#   static_transform_publisher 인자를 실측 후 갱신할 것.
#=============================================
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from ..vehicle_spec import WHEELBASE_M, VESC_SPEED_TO_ERPM_GAIN, VESC_STALE_SEC, IMU_STALE_SEC
from .pose_estimator import EncoderPoseEstimator

# [2026-08-22] 20Hz -> 100Hz. K턴이 1.2초마다 전진/후진 부호를 뒤집는데, 20Hz(0.05s)
# 오일러 적분은 콜백이 조금만 밀려도 "그 순간의 속도"를 훨씬 긴 dt에 곱하게 돼 이산화
# 오차가 커진다 — 실측 확인: K턴 도중 /odom이 /gazebo/odom(ground truth) 대비 3~6m까지
# 벌어짐(IMU 헤딩은 실측 결과 ground truth와 오차 0 확인됨 — 적분 샘플링 쪽이 원인).
# 주기를 5배 높여 부호전환 사이 샘플 수를 늘려 이산화 오차를 줄인다.
ODOM_PUBLISH_PERIOD_S = 0.01   # 100Hz


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value

        self.v_mps = 0.0
        self._vesc_t = None
        self.imu_yaw = 0.0
        self._imu_t = None
        self._last_update_ns = None
        self._prev_yaw_for_twist = None

        self.pose_estimator = EncoderPoseEstimator(wheelbase_m=WHEELBASE_M)
        self.pose_estimator.set_yaw_source('imu')

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, '/imu', self._cb_imu, qos_profile_sensor_data)
        self.create_subscription(Float32, '/vesc_speed_erpm', self._cb_vesc, qos_profile_sensor_data)

        self.create_timer(ODOM_PUBLISH_PERIOD_S, self._update_and_publish)

        self.get_logger().info(
            f'odom_publisher 초기화 완료 | frame: {self._odom_frame} -> {self._base_frame}, '
            f'wheelbase={WHEELBASE_M}m, yaw_source=imu')

    # UMK track_drive.py의 cb_imu()/cb_vesc()와 동일한 파싱 로직(일관성 유지 목적)
    def _cb_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._imu_t = time.time()

    def _cb_vesc(self, msg):
        try:
            self.v_mps = float(msg.data) / VESC_SPEED_TO_ERPM_GAIN
            self._vesc_t = time.time()
        except (TypeError, ValueError) as e:
            self.get_logger().error(f'[vesc] /vesc_speed_erpm 파싱 실패: {e}', throttle_duration_sec=2.0)

    def _vesc_live(self):
        return self._vesc_t is not None and (time.time() - self._vesc_t) < VESC_STALE_SEC

    def _imu_live(self):
        return self._imu_t is not None and (time.time() - self._imu_t) < IMU_STALE_SEC

    def _update_and_publish(self):
        # [2026-08-22] dt를 time.time()(벽시계)으로 재던 것을 self.get_clock().now()
        # (use_sim_time을 따르는 ROS 클럭)로 교체 — v_mps(/vesc_speed_erpm)는 Gazebo
        # 시뮬레이션 시간 기준 속도인데, 벽시계 dt로 적분하면 Gazebo의 real-time factor가
        # 1.0이 아닐 때(이 환경처럼 CPU 부하가 커서 물리엔진이 실시간보다 느리게 도는
        # 경우) "실제 흐른 시뮬레이션 시간"보다 훨씬 큰 dt로 속도를 적분해 위치가 실제보다
        # 훨씬 많이 튀어나간다. 실측 확인: K턴 도중 /odom이 /gazebo/odom(ground truth)
        # 대비 수 미터(6m 안팎)까지 발산 — local_costmap이 이 발산한 odom을 기준으로
        # 롤링윈도우를 잡다 보니 "Sensor origin ... out of map bounds" 경고가 반복되고,
        # AMCL 모션모델에도 잘못된 델타가 들어가 도킹 중 헤딩추정이 요동치는 것까지 이어졌다.
        now = self.get_clock().now()

        if self._last_update_ns is None:
            self._last_update_ns = now.nanoseconds
            return
        dt = (now.nanoseconds - self._last_update_ns) / 1e9
        self._last_update_ns = now.nanoseconds

        # 센서가 죽어있으면 "조용히 마지막 값 유지"가 아니라 v=0으로 접어서 위치가
        # 계속 튀어나가지 않게 하되, 경고는 매번(throttle) 남긴다.
        if not self._vesc_live():
            self.get_logger().warn('[odom_publisher] VESC 죽음 — v_mps=0으로 적분', throttle_duration_sec=2.0)
            v_for_update = 0.0
        else:
            v_for_update = self.v_mps

        if not self._imu_live():
            self.get_logger().warn('[odom_publisher] IMU 죽음 — yaw 갱신 보류(마지막 값 유지)', throttle_duration_sec=2.0)
            imu_yaw_for_update = self.pose_estimator.yaw
        else:
            imu_yaw_for_update = self.imu_yaw

        x, y, yaw = self.pose_estimator.update(v_for_update, 0.0, dt, imu_yaw=imu_yaw_for_update)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        # 데드레커닝 기반이라 실측 공분산이 아니다 — AMCL이 상대적 신뢰도만 참고하도록
        # 대략적인 값만 채워둔다(추후 실차 드리프트 관찰 후 조정 가능).
        odom_msg.pose.covariance[0] = 0.05    # x
        odom_msg.pose.covariance[7] = 0.05    # y
        odom_msg.pose.covariance[35] = 0.05   # yaw
        odom_msg.twist.twist.linear.x = v_for_update
        if self._prev_yaw_for_twist is not None and dt > 0:
            odom_msg.twist.twist.angular.z = (yaw - self._prev_yaw_for_twist) / dt
        self._prev_yaw_for_twist = yaw
        self._odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
