import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_mission')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_share, 'maps', 'parking_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    parking_zone = LaunchConfiguration('parking_zone')
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    bringup_sensors = LaunchConfiguration('bringup_sensors')
    laser_frame_id = LaunchConfiguration('laser_frame_id')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_yaw_deg = LaunchConfiguration('lidar_yaw_deg')

    declare_map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='사용할 occupancy grid map yaml 경로 (parking_map.yaml)')

    declare_params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 파라미터 yaml 경로')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Gazebo 등 시뮬레이션 사용 시 true, 실제 로봇 구동 시 false')

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 lifecycle 노드 자동 활성화 여부')

    # [2026-08-24] 공식 경기 규정 확인 결과 이 미션은 "출발 → A 주차 → B 주차 →
    # 출발지 복귀"를 한 번에 전부 수행해야 한다 — 기본값을 'FULL'로 바꿔 그 전체
    # 시퀀스가 기본 동작이 되도록 함. 'A'/'B'처럼 개별 레그 이름을 주면 그 레그
    # 하나만 실행한다(디버깅/개별 구간 튜닝용, 기존 단일 레그 동작과 동일).
    declare_parking_zone_arg = DeclareLaunchArgument(
        'parking_zone', default_value='FULL',
        description="'FULL'(기본값)이면 출발→A→B→출발복귀 전체 순회, "
                     "'A'/'B'/'START'를 주면 그 레그 하나만 실행(디버깅용)")

    declare_set_initial_pose_arg = DeclareLaunchArgument(
        'set_initial_pose', default_value='true',
        description='시작 시 출발영역 pose를 /initialpose로 자동 발행할지 여부')

    # [2026-08-20] 이전엔 이 launch가 start_x/y/yaw를 아예 안 받아서
    # parking_navigator_node가 항상 자기 내부 기본값(1.8/0.9/3.14)만 썼다 — 시뮬레이션
    # 쪽(parking_mission_sim.launch.py)에서 스폰 위치를 랜덤화해도 AMCL은 여전히
    # "1.8, 0.9에 있다"고 잘못 믿게 되는 불일치가 있었다. 이제 명시적으로 받아서
    # 아래 parking_navigator_node.parameters로 전달한다(기본값은 기존 고정 시작점과 동일).
    declare_start_x_arg = DeclareLaunchArgument(
        'start_x', default_value='1.8',
        description='parking_navigator가 /initialpose로 발행할 시작 x(실제 스폰 위치와 일치해야 함)')
    declare_start_y_arg = DeclareLaunchArgument(
        'start_y', default_value='0.9',
        description='parking_navigator가 /initialpose로 발행할 시작 y(실제 스폰 위치와 일치해야 함)')
    declare_start_yaw_arg = DeclareLaunchArgument(
        'start_yaw', default_value='3.14',
        description='parking_navigator가 /initialpose로 발행할 시작 yaw(rad, 실제 스폰 위치와 일치해야 함)')

    declare_bringup_sensors_arg = DeclareLaunchArgument(
        'bringup_sensors', default_value='true',
        description='라이다/IMU 드라이버를 이 launch에서 직접 띄울지 여부. UMK track_drive.launch.py'
                     ' 등 다른 곳에서 이미 띄운 상태로 주차미션만 추가 실행하는 경우 false로.')

    declare_laser_frame_id_arg = DeclareLaunchArgument(
        'laser_frame_id', default_value='laser',
        description='xycar_lidar가 /scan에 싣는 frame_id. 실제 값과 다르면 AMCL이 TF를 못 찾으니'
                     ' 실차에서 `ros2 topic echo /scan --field header.frame_id`로 확인 후 맞출 것.')

    # ★실측 필요★ — 라이다 장착 위치(base_link 기준 평행이동)는 UMK 저장소에서 한 번도
    #   실측된 적이 없다(measure_lidar_camera_offset.md 확인, "평행이동 오프셋은 한 번도
    #   실측된 적이 없었다"고 명시). 아래 기본값 0,0,0은 플레이스홀더이니 실차에서 줄자로
    #   재고 반드시 갱신할 것 — 틀린 채로 두면 AMCL 스캔매칭이 그 오차만큼 계속 어긋난다.
    #   회전(yaw)도 UMK config.py LIDAR_ANGLE_OFFSET_DEG=80(2026-07-22 실측)이 최근 재확인에서
    #   88~89로 드리프트한 정황이 있어(measure_lidar_camera_offset.md "알려진 한계" 참고)
    #   그대로 못 가져다 쓰고 기본값 0으로 둔다 — 실차에서 라이다 정면 기준으로 재실측할 것.
    declare_lidar_x_arg = DeclareLaunchArgument(
        'lidar_x', default_value='0.0',
        description='★실측 필요★ base_link 기준 라이다 x(전방+) 오프셋(m). 플레이스홀더.')
    declare_lidar_y_arg = DeclareLaunchArgument(
        'lidar_y', default_value='0.0',
        description='★실측 필요★ base_link 기준 라이다 y(좌+) 오프셋(m). 플레이스홀더.')
    declare_lidar_yaw_deg_arg = DeclareLaunchArgument(
        'lidar_yaw_deg', default_value='0.0',
        description='★실측 필요★ base_link 기준 라이다 장착 회전각(deg). 플레이스홀더 —'
                     ' UMK LIDAR_ANGLE_OFFSET_DEG를 그대로 쓰지 않음(드리프트 의심 정황 있음).')

    # ── 센서 드라이버 (라이다/IMU) — UMK track_drive.launch.py와 동일 패키지 include ──
    #   카메라는 주차 미션(라이다+AMCL 기반)에 안 쓰므로 띄우지 않는다.
    #
    # [2026-08-25] IfCondition(bringup_sensors)는 "이 액션을 실행할지"만 런타임에 제어할 뿐,
    # get_package_share_directory('xycar_lidar'/'xycar_imu')는 launch 트리를 "구성"하는
    # 시점(이 함수가 실행되는 시점)에 조건과 무관하게 항상 즉시 호출된다 — 그 패키지가
    # 설치 안 된 환경(예: Gazebo만 있고 xycar 실차 드라이버는 없는 시뮬레이션/개발 환경)에서는
    # bringup_sensors:=false를 줘도 PackageNotFoundError로 launch 전체가 죽는 걸 실측
    # 확인함(Docker 기반 시뮬레이션 테스트 중 발견).
    #
    # 처음엔 parking_mission_sim.launch.py의 random_start 처리와 동일한 패턴(sys.argv를
    # 직접 읽어 구성 시점에 분기)으로 고치려 했으나, 실측해보니 그 패턴은 최상위 `ros2
    # launch` CLI 인자에만 통한다 — bringup_sensors:=false는 parking_mission_sim.launch.py
    # 내부에서 IncludeLaunchDescription(launch_arguments={...})로 "프로그램적으로" 전달되지
    # sys.argv엔 안 잡힌다(같은 오류 재현 확인). 그래서 대신 이 패키지가 실제로 설치돼
    # 있는지(get_package_share_directory 성공 여부)로 판단한다 — bringup_sensors 값과
    # 무관하게 "패키지가 없으면 건너뛴다"는 게 원하는 동작과 정확히 일치하고, CLI/중첩
    # include 어느 경로로 호출되든 항상 안전하다.
    sensor_includes = []
    try:
        xycar_lidar_share = get_package_share_directory('xycar_lidar')
        xycar_imu_share = get_package_share_directory('xycar_imu')
    except PackageNotFoundError:
        xycar_lidar_share = None
        xycar_imu_share = None
    if xycar_lidar_share is not None and xycar_imu_share is not None:
        sensor_includes = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(xycar_lidar_share, 'launch', 'xycar_lidar.launch.py')),
                condition=IfCondition(bringup_sensors),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(xycar_imu_share, 'launch', 'xycar_imu.launch.py')),
                condition=IfCondition(bringup_sensors),
            ),
        ]

    # base_link -> laser 정적 TF. AMCL이 /scan(laser_frame_id)을 base_link로 변환하는 데 필요.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', '0.0',
                   '--yaw', lidar_yaw_deg, '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', laser_frame_id],
    )

    # VESC(v_mps) + IMU(yaw) -> /odom + odom->base_link TF. (parking_mission/localization/odom_publisher.py)
    odom_publisher_node = Node(
        package='parking_mission',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Nav2 /cmd_vel(Twist) -> xycar_motor(Float32MultiArray[angle,speed]) 변환.
    cmd_vel_bridge_node = Node(
        package='parking_mission',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Nav2 표준 bringup: map_server + amcl(localization) + planner/controller/behavior
    # + bt_navigator + waypoint_follower + 각 lifecycle_manager 포함
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    # AMCL/lifecycle이 active 상태가 될 시간을 확보한 뒤 목표 전송 노드 실행
    parking_navigator_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='parking_mission',
                executable='parking_navigator',
                name='parking_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'parking_zone': parking_zone,
                    'set_initial_pose': set_initial_pose,
                    'start_x': start_x,
                    'start_y': start_y,
                    'start_yaw': start_yaw,
                    # [2026-08-24] 이전엔 static_tf_lidar(TF)에만 전달되고 parking_navigator
                    # 노드 자신에게는 안 넘어갔다 — 그 결과 _lidar_front_clearance()/
                    # _lidar_steer_bias()가 raw /scan 각도를 TF 없이 직접 쓰면서도 이
                    # 오프셋을 몰라 "정면"을 잘못 판단하는 버그가 있었다(위
                    # declare_lidar_yaw_deg_arg 주석과 동일 근거). 이제 노드 파라미터로도
                    # 전달해 그 두 함수가 보정할 수 있게 한다.
                    'lidar_yaw_deg': lidar_yaw_deg,
                }],
            )
        ],
    )

    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_parking_zone_arg,
        declare_set_initial_pose_arg,
        declare_start_x_arg,
        declare_start_y_arg,
        declare_start_yaw_arg,
        declare_bringup_sensors_arg,
        declare_laser_frame_id_arg,
        declare_lidar_x_arg,
        declare_lidar_y_arg,
        declare_lidar_yaw_deg_arg,
        *sensor_includes,
        static_tf_lidar,
        odom_publisher_node,
        cmd_vel_bridge_node,
        nav2_bringup_launch,
        parking_navigator_node,
    ])
