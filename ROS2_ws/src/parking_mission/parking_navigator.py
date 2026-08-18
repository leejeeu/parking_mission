import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    """2D yaw(rad) -> (x, y, z, w) quaternion."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ParkingNavigator(Node):

    def __init__(self):
        super().__init__('parking_navigator')

        self.declare_parameter('parking_zone', 'A')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('initial_pose_wait_sec', 5.0)

        self.declare_parameter('start_x', 1.8)
        self.declare_parameter('start_y', 0.9)
        self.declare_parameter('start_yaw', 3.14)

        self.declare_parameter('zone_a_x', 0.0)
        self.declare_parameter('zone_a_y', 4.2)
        self.declare_parameter('zone_a_yaw', 0.0)

        self.declare_parameter('zone_b_x', 2.1)
        self.declare_parameter('zone_b_y', 3.3)
        self.declare_parameter('zone_b_yaw', -1.57)

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

    def send_goal(self, zone: str):
        x, y, yaw = self.zone_pose(zone)
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
            self.get_logger().error('목표가 거부되었습니다.')
            self._done = True
            self._success = False
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('주차 완료: 목표 pose에 도달했습니다.')
            self._success = True
        else:
            self.get_logger().error(f'주차 실패 (status={status}).')
            self._success = False
        self._done = True

    def run(self):
        zone = self.get_parameter('parking_zone').value

        if self.get_parameter('set_initial_pose').value:
            wait_sec = self.get_parameter('initial_pose_wait_sec').value
            deadline = self.get_clock().now().nanoseconds + int(wait_sec * 1e9)
            while (self._initial_pose_pub.get_subscription_count() == 0
                   and self.get_clock().now().nanoseconds < deadline):
                rclpy.spin_once(self, timeout_sec=0.2)
            self.publish_initial_pose()

        self.get_logger().info('navigate_to_pose 액션 서버 대기 중...')
        self._nav_client.wait_for_server()

        self.send_goal(zone)

        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.2)

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
