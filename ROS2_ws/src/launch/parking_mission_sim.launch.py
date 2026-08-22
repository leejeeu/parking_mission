import os
import random
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# [2026-08-20] 랜덤 시작 위치 후보 — maps/parking_map.pgm을 실제로 픽셀 분석해서 고른
# 지점들(distance-transform으로 가장 가까운 벽까지 0.4m 이상 여유, 목표(0,4.2)와
# 같은 연결된 자유공간 컴포넌트인지도 확인함 — 갇힌 방에 스폰되는 사고 방지).
# [2026-08-20 수정] 처음엔 지도 전체(먼 구석 포함)에서 뽑았다가, 실측 결과 지도의
# 다른 구역(예: -2.00,1.10 근처)에 우리가 전혀 튜닝 안 한 새로운 좁은 지점이 있어서
# 거기서도 "lethal space"가 재현됨 — 요청에 따라 실제 주차장 영역(시작점/A/B 주변,
# x:[-0.8,2.6] y:[-0.3,5.0] 박스)으로 후보를 좁혔다. yaw는 딱히 중요치 않아 각
# 지점마다 임의로 하나 배정. 새 지도로 바뀌면 이 목록도 다시 뽑아야 한다(스크립트:
# scipy.ndimage.distance_transform_edt + label로 재생성 가능).
RANDOM_START_CANDIDATES = [
    (1.80, 0.90, 3.14),    # 기존 고정 시작점(호환용으로 포함)
    (1.15, 0.90, 0.0),
    (-0.35, 2.25, 1.2),
    (2.45, 2.50, -2.0),
    (0.90, 4.50, 0.6),
    (2.10, 3.65, -1.4),
    (0.40, 1.25, 2.3),
    (1.75, 1.15, -0.5),
    (2.15, 4.90, 3.0),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_mission')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'parking_map.world')
    default_urdf = os.path.join(pkg_share, 'urdf', 'xycar.urdf')

    world = LaunchConfiguration('world')
    urdf = LaunchConfiguration('urdf')
    parking_zone = LaunchConfiguration('parking_zone')
    # parking_navigator.py의 start_x/start_y/start_yaw 기본값과 동일 — 그 노드가 보내는
    # /initialpose가 실제 스폰 위치와 어긋나면 AMCL이 엉뚱한 곳에서 수렴을 시작한다.
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    # [2026-08-20] 기본값은 매 launch마다 위 후보 목록에서 무작위로 하나 고른다.
    # random_start(DeclareLaunchArgument)는 LaunchConfiguration이라 이 함수가 실행되는
    # 시점(launch 트리 "구성" 단계)엔 아직 substitution이 안 끝나 값을 못 읽는다 —
    # ros2 launch가 실제로 넘기는 형태(sys.argv의 "key:=value")를 직접 봐서 이
    # "무작위 기본값을 계산할지 말지"를 launch 트리 구성 이전에 결정한다(다른
    # DeclareLaunchArgument 기본값들과 동일한 타이밍 제약). random_start:=false로
    # 주면 후보 목록의 첫 번째(기존 고정 시작점)를 그대로 쓴다 — 재현 디버깅용.
    _random_start_arg = next(
        (a.split(':=', 1)[1] for a in sys.argv if a.startswith('random_start:=')), 'true')
    if _random_start_arg.lower() != 'false':
        rx, ry, ryaw = random.choice(RANDOM_START_CANDIDATES)
    else:
        rx, ry, ryaw = RANDOM_START_CANDIDATES[0]
    declare_random_start_arg = DeclareLaunchArgument(
        'random_start', default_value='true',
        description='true(기본값)면 RANDOM_START_CANDIDATES 중 무작위 스폰, false면 기존 고정 시작점')

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo 월드 파일(scripts/generate_parking_world.py로 maps/parking_map.pgm에서 생성됨)')
    declare_urdf_arg = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='스폰할 차량 URDF')
    declare_parking_zone_arg = DeclareLaunchArgument(
        'parking_zone', default_value='A',
        description="목표 주차영역: 'A' 또는 'B' (parking_mission.launch.py로 그대로 전달)")
    declare_start_x_arg = DeclareLaunchArgument(
        'start_x', default_value=str(rx),
        description='스폰 시작 x(random_start:=true면 무작위 선택된 값 — parking_navigator.py로 그대로 전달됨)')
    declare_start_y_arg = DeclareLaunchArgument(
        'start_y', default_value=str(ry),
        description='스폰 시작 y(random_start:=true면 무작위 선택된 값 — parking_navigator.py로 그대로 전달됨)')
    declare_start_yaw_arg = DeclareLaunchArgument(
        'start_yaw', default_value=str(ryaw),
        description='스폰 시작 yaw(rad, random_start:=true면 무작위 선택된 값)')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )

    # 차량을 지면 위로 살짝 띄워서 스폰 — base_link 원점=지면투영(z=0)이라 초기
    # 관통(interpenetration) 지터를 피하려고 약간의 여유(0.02m)를 둔다.
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'xycar',
            '-file', urdf,
            '-x', start_x, '-y', start_y, '-z', '0.02',
            '-Y', start_yaw,
        ],
        output='screen',
    )

    motor_bridge_node = Node(
        package='parking_mission',
        executable='sim_motor_bridge',
        name='sim_motor_bridge',
        output='screen',
    )

    # Gazebo가 완전히 뜨고 차량이 스폰돼 /scan, /imu, /gazebo/odom이 발행되기 시작할
    # 시간을 확보한 뒤에야 AMCL/Nav2 스택을 올린다 — 기존 parking_mission.launch.py의
    # parking_navigator_node TimerAction(3.0)과 동일한 설계 근거.
    #
    # 30초로 둔 이유(2026-08-19 실측, 두 단계로 확인):
    #  1) nav2_bringup 컴포지션(맵서버+AMCL+controller/planner/behavior/bt_navigator 등
    #     9개 lifecycle 노드 순차 configure+activate)이 이 환경에서 6초 이상 걸림 —
    #     parking_mission.launch.py 내부 parking_navigator_node의 고정 TimerAction(3.0)보다
    #     오래 걸려서 처음엔 "목표가 거부되었습니다"로 실패했다.
    #  2) 그 다음 문제가 더 컸다: parking_navigator가 발행하는 /initialpose(RELIABLE+
    #     TRANSIENT_LOCAL)는 QoS 자체는 AMCL의 구독(BEST_EFFORT+VOLATILE)과 호환되지만,
    #     Gazebo 물리엔진 + Nav2 컴포지션을 동시에 돌리는 이 환경의 CPU 부하 때문에
    #     DDS가 그 transient-local 메시지를 실제로 전달하기까지 실측 15초 가까이 걸렸다
    #     (수동으로 /initialpose를 발행해 재현 확인: publish~receive 간격 ~15초).
    #     parking_navigator는 "구독자 연결됨"만 확인하고 바로 발행+목표전송하므로 이
    #     지연을 못 버티고 AMCL이 초기 pose를 받기 전에 goal을 보내 매번 거부당했다.
    #     실차는 Gazebo 물리 연산이 없어 이 정도 DDS 지연이 생기지 않을 걸로 보이므로
    #     parking_navigator.py/parking_mission.launch.py의 실차 타이밍 값은 건드리지 않고,
    #     이 sim launch의 바깥쪽 지연만 넉넉히 30초로 늘려 위 두 지연을 모두 흡수한다.
    #
    # map/params_file을 명시적으로 넘기는 이유: TimerAction으로 지연된 IncludeLaunchDescription
    # 안에서는 parking_mission.launch.py 자신의 DeclareLaunchArgument 기본값(map/params_file)이
    # 제대로 해석되지 않고 빈 문자열로 substitution되는 launch 프레임워크 동작을 실제로
    # 재현 확인함(2026-08-19, "FileNotFoundError: [Errno 2] No such file or directory: ''"
    # at nav2_common/launch/rewritten_yaml.py). 기본값에 의존하지 않고 항상 명시적으로
    # 전달해서 우회한다.
    default_map = os.path.join(pkg_share, 'maps', 'parking_map.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    parking_mission_launch = TimerAction(
        period=30.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'parking_mission.launch.py')),
                launch_arguments={
                    'map': default_map,
                    'params_file': default_params_file,
                    'use_sim_time': 'true',
                    'bringup_sensors': 'false',  # Gazebo가 /scan, /imu를 대신 발행하므로 실제 드라이버는 스킵
                    'parking_zone': parking_zone,
                    'set_initial_pose': 'true',
                    # [2026-08-20] 이전엔 여기서 안 넘겨서 AMCL 초기 pose가 실제 스폰
                    # 위치와 항상 어긋나 있었다(위 declare_random_start_arg 주석 참고) —
                    # spawn_entity_node와 반드시 같은 값을 써야 한다.
                    'start_x': start_x,
                    'start_y': start_y,
                    'start_yaw': start_yaw,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        declare_world_arg,
        declare_urdf_arg,
        declare_parking_zone_arg,
        declare_random_start_arg,
        declare_start_x_arg,
        declare_start_y_arg,
        declare_start_yaw_arg,
        gazebo_launch,
        spawn_entity_node,
        motor_bridge_node,
        parking_mission_launch,
    ])
