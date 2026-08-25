# parking_mission

> **2026-08-22 — 팀원 커밋과 병합충돌 정리 노트**: 이 문서와 `config/nav2_params.yaml`이
> 팀원(이지유)의 독립 커밋(`190b764 Fix wall-collision: inflation radius bug, switch to
> SmacPlannerHybrid, boost obstacle weight`)과 병합충돌이 났었다. 흥미롭게도 팀원도
> 완전히 독립적으로 같은 세 가지 진단(inflation_radius<robot_radius 버그,
> NavfnPlanner→SmacPlannerHybrid 교체 필요성, 컨트롤러 장애물 가중치 부족)에 도달했다 —
> 방향 자체는 맞았다는 뜻. 다만 실제 값은 이 세션에서 그동안 실측으로 검증한 쪽을
> 남겼다(아래 §2/§5/§6 및 `nav2_params.yaml`의 관련 주석 참고): plugin 이름은
> "패키지/클래스" 슬래시 형식만 이 환경에서 로드되고(콜론 형식은 FATAL로 죽음),
> `minimum_turning_radius`는 이 저장소가 실제로 쓰는 최대조향각 80도 기준값(0.06m)으로,
> `motion_model_for_search`는 컨트롤러의 `allow_reversing: false`와 맞춰 DUBIN으로,
> `inflation_radius`는 0.45로 올리면 목표 지점 자체가 통행불가가 돼서 0.1로 유지했다.
> 컨트롤러 장애물 가중치 상향이라는 의도는 `use_cost_regulated_linear_velocity_scaling`
> 등 RegulatedPurePursuitController 쪽 파라미터로 이미 반영돼 있다(팀원 커밋은 이미
> 교체되기 전인 DWB 시절 파라미터를 손본 것이라 지금 컨트롤러엔 안 먹음).

> **[2026-08-24 정정] "후진 금지"는 오해였다**: 위 노트 및 2026-08-22의 여러 커밋이
> "대회 실차 규정상 후진이 금지되어 있다"는 전제로 planner(`DUBIN`)/controller
> (`allow_reversing: false`)/BT(커스텀 no-reverse 트리)/도킹 제어(전진 전용 큰 원호)를
> 다시 짰었는데, 사용자 확인 결과 **후진은 완전히 허용된다** — 그 전제 자체가 잘못된
> 것이었다. 이번 커밋에서 planner를 다시 `REEDS_SHEPP`, controller를
> `allow_reversing: true`로 되돌리고, 커스텀 BT 트리를 삭제해 nav2 기본 트리(BackUp
> 포함)로 복귀했으며, 도킹 제어에 후진 포함 K턴을 복원했다. 또한 대회 측 공식 맵
> 안내문을 확인한 결과 이 미션은 A 하나만 가는 게 아니라 **"출발→A 주차→B 주차→출발지
> 복귀"를 전부 수행**해야 하는 것으로 확인돼, 미션 구조 자체도 다중 레그로 재작성했다
> (아래 §3 참고).

> **[2026-08-25 재정정]** 위 08-24 메모의 "A→B 둘 다 방문" 부분이 다시 틀린 것으로
> 확인됐다. 실제 미션은 **A 또는 B 중 실행 전에 지정된 한 구역만 방문 후 출발지 복귀**다
> (두 구역 동시 방문 아님). `MISSION_LEGS`(A,B,START 고정 시퀀스)를 제거하고
> `parking_zone`('A'/'B' 필수)으로 지정한 구역 + START 두 레그만 도는 것으로 되돌렸다
> (아래 §3 참고).

자율주행 주차 미션: Nav2(ROS2 Navigation2) 기반으로 출발 지점에서 지정된 주차영역(A 또는 B)을
방문해 정밀 주차한 뒤 출발지로 복귀하는 패키지. 실차(xycar, 애커먼만/자전거 모델
조향, 후진 가능)와 Gazebo 시뮬레이션을 동일한 코드로 구동한다.

## 1. 주행 파이프라인

```
[AMCL 로컬라이제이션] → [전역 경로계획] → [지역 경로추종/장애물회피] → [모터 인터페이스 변환] → 주행
```

| 단계 | 노드 | 역할 |
|---|---|---|
| 로컬라이제이션 | `amcl` (nav2_amcl) | 라이다 스캔을 맵과 대조해 현재 pose 추정 |
| 전역 경로계획 | `planner_server` (nav2_navfn_planner) | 현재 위치 → 목표까지 장애물 회피 경로 계산 |
| 지역 경로추종 | `controller_server` (nav2_regulated_pure_pursuit_controller) | 경로를 따라가는 (속도, 조향) 명령 계산 |
| 행동관리/복구 | `bt_navigator`, `behavior_server` | 목표 전송, 주기적 재계획, 막히면 후진/제자리대기 등 복구 |
| 오도메트리 | `odom_publisher`(`parking_mission/localization`) | VESC 속도 + IMU yaw로 `/odom` + TF 발행 |
| 모터 변환 | `cmd_vel_bridge` | Nav2의 `Twist`(/cmd_vel) → 실차 모터 인터페이스(`xycar_motor`: [조향각, 속도]) 변환 |
| 미션 진행 | `parking_navigator` | 초기 pose 설정 + 목표(A/B) 전송 + 결과 판정 |

## 2. 알고리즘

### 로컬라이제이션 — AMCL (Adaptive Monte Carlo Localization)
파티클 필터 기반 확률적 위치추정. 라이다 스캔을 사전에 만들어둔 점유격자 맵(`maps/parking_map.pgm`)에
계속 매칭시켜 파티클(위치 후보)들을 확률적으로 좁혀가며 pose를 수렴시킨다.

### 전역 경로계획 — NavFn (`nav2_navfn_planner`)
격자 위에서 파도전파(wavefront propagation) 방식으로 최단 경로를 계산. `nav2_params.yaml`에서
`use_astar: true`로 설정해 A* 모드로 동작.

### 지역 경로추종 + 장애물회피 — Regulated Pure Pursuit Controller
경로 위 일정 거리 앞의 "lookahead point"를 향해 곡률(curvature)을 계산해 조향각으로 변환하는
Pure Pursuit 알고리즘. 곡률이 크거나(급회전) 장애물이 가까우면 속도를 자동으로 줄인다
(`use_regulated_linear_velocity_scaling`). 로컬 코스트맵(라이다 실시간 반영)을 보고 충돌이
예상되면 정지/감속(`use_collision_detection`).

**원래는 DWB(Dynamic Window Approach)였는데 RegulatedPurePursuit로 교체했다** — DWB의
`RotateToGoal` 크리틱은 미분구동 로봇처럼 "제자리 회전"(linear.x=0, angular.z≠0)을 명령할 수
있는데, 이 차는 애커먼만(자전거 모델) 조향이라 그 명령을 물리적으로 수행할 수 없다(속도 0에서는
조향각을 아무리 줘도 못 돎). Pure Pursuit는 항상 curvature 기반 (v,w) 쌍을 계산하므로 이
문제가 없다. Nav2 공식 문서도 애커먼만/차량형 로봇에는 이 컨트롤러를 권장한다.

장애물회피 자체는 DWA처럼 매번 여러 궤적을 시뮬레이션해서 고르는 방식이 아니라, ① 코스트맵에
장애물을 반영해 애초에 그 위로 경로가 안 잡히게 하고(전역), ② 주행 중 상황이 바뀌면
`bt_navigator`가 주기적으로 경로를 재계획하는 구조다. 막히면 `behavior_server`가 후진(BackUp)·
제자리 대기(Wait)·회전(Spin) 같은 복구 동작을 시도한다.

### 목표 판정 — SimpleGoalChecker
`xy_goal_tolerance: 0.05`(5cm), `yaw_goal_tolerance: 0.05`(약 3도) 이내에 도달해야 "주차 완료"로
판정한다.

## 3. 미션 시퀀스 — 출발 → (A 또는 B 중 지정된 한 곳) 주차 → 출발지 복귀

> **[2026-08-25 정정]** 2026-08-24 메모는 "A와 B를 순서대로 둘 다 방문"이 공식 규정이라고
> 적었으나 잘못된 이해였다. 실제로는 **A/B 둘 중 실행 전에 지정된 한 구역만 방문**하고
> 출발지로 복귀한다(두 구역을 동시에 가지 않음). 어느 구역인지는 센서로 자동판단하지 않고
> 실행 전 사람이 `parking_zone` 파라미터로 지정한다.

`parking_navigator.py`의 `run()`은 `parking_zone`('A' 또는 `'B'`, 필수)으로 지정된 구역과
`'START'`(출발지 복귀) 두 레그만 순서대로 수행한다(레그마다 Nav2 목표 전송 → 필요하면
병목구간 우회 → 정밀 접근(도킹) 순). A/B 좌표는 `zone_a_x/y/yaw`, `zone_b_x/y/yaw`
파라미터에 고정값(A=(0.0, 4.2, 0°), B=(2.1, 3.3, -90°))으로 있고, 출발지 복귀 목표는
`start_x/y/yaw` 파라미터를 재사용한다.

레그 하나가 타임아웃 등으로 실패해도 미션은 멈추지 않고 다음 레그(복귀)로 계속 진행한다.

카메라/AR태그 등으로 "어느 자리가 비어있는지" 감지해서 A/B를 자동 선택하는 로직은 없다 —
실행 전 사람이 고르는 방식으로 확인됨.

## 4. 실행 방법

### 4.1 실차

```bash
# 사전 1회: nav2 설치 확인
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

cd ~/parking_mission/ROS2_ws
colcon build --packages-select parking_mission
source install/setup.bash

ros2 launch parking_mission parking_mission.launch.py parking_zone:=A
```

`parking_zone:=B`로 바꾸면 B구역으로 간다. `lidar_x`/`lidar_y`/`lidar_yaw_deg`(라이다 장착
오프셋)는 실차에서 실측 후 갱신 필요(launch 파일 내 "★실측 필요★" 주석 참고, 현재 플레이스홀더 0).

### 4.2 Gazebo 시뮬레이션

Gazebo Classic 11 + `ros-humble-gazebo-*`가 설치돼 있어야 한다(2026-08-19 확인, 이미 설치됨).
실차 코드(`odom_publisher`/`cmd_vel_bridge`/`parking_navigator`/`parking_mission.launch.py`)는
전혀 수정하지 않고, Gazebo가 실제 라이다·IMU·모터를 대신하도록 새 파일들만 추가했다
(`urdf/xycar.urdf`, `worlds/parking_map.world`, `parking_mission/sim/motor_bridge.py`,
`launch/parking_mission_sim.launch.py`).

```bash
cd ~/parking_mission/ROS2_ws
colcon build --packages-select parking_mission
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash   # xycar_lidar/xycar_imu 패키지 참조용(bringup_sensors:=false라 실제로 안 뜨지만 launch 파일이 경로를 찾음)
source install/setup.bash

# GUI로 보면서 실행 (Gazebo 창이 뜬다)
ros2 launch parking_mission parking_mission_sim.launch.py parking_zone:=A

# 헤드리스(GUI 없이, 토픽/로그만)
ros2 launch parking_mission parking_mission_sim.launch.py parking_zone:=A gui:=false
```

주요 launch 인자:
- `parking_zone`: `A` 또는 `B` (기본 `A`)
- `start_x`/`start_y`/`start_yaw`: 스폰 시작 pose (기본 1.8/0.9/3.14, `parking_navigator.py`의
  `start_x/y/yaw` 기본값과 반드시 일치시켜야 함 — 다르면 AMCL이 엉뚱한 곳에서 시작)
- `gui`: `false`면 gzclient(3D 뷰) 없이 gzserver만

맵→Gazebo 월드 변환은 `scripts/generate_parking_world.py`가 담당한다. `maps/parking_map.pgm`이
새로 SLAM으로 갱신되면 이 스크립트를 재실행해서 `worlds/parking_map.world`를 다시 만들 것.

## 5. 시뮬레이션 구축 중 찾아 고친 실차급 버그 (2026-08-19)

Gazebo 연동 작업 중, Gazebo와 무관하게 실차에서도 똑같이 막혔을 pre-existing 버그를 다수
발견해 고쳤다(전부 `nav2_params.yaml`/`parking_navigator.py`, 실차 코드/설정):

- `map_server.yaml_filename`이 빈 채로 있어 맵 로딩 자체가 실패 → 명시적 플레이스홀더 추가.
- `nav2_navfn_planner`/`nav2_behaviors`/`plugin_lib_names`가 불완전해 행동트리(bt_navigator)가
  아예 못 뜸 → nav2_bringup 공식 목록으로 교체.
- `bt_navigator.default_nav_to_pose_bt_xml`이 빈 문자열이라 "Empty Tree"로 매번 즉시 실패
  → 해당 키 자체를 생략(내장 기본값 사용).
- `parking_navigator.py`가 AMCL 초기 pose 확인 없이 바로 목표를 보내는 레이스 컨디션
  → `/amcl_pose` 수신 확인 + 재발행 + 목표 거부 시 재시도 로직 추가(`GOAL_RETRY_MAX=20`).
- DWB 컨트롤러가 애커먼만 차량에 불가능한 "제자리 회전"을 명령 → RegulatedPurePursuitController로
  교체(`use_rotate_to_heading: false` 필수).
- 주차영역 A 목표(0,4.2)가 벽에서 0.5m밖에 안 떨어져 있어 `inflation_radius`(원래 0.3)가
  목표 지점 자체를 막음 → `inflation_radius: 0.1`로 축소, 대신 `cost_scaling_factor: 8.0`으로
  좁은 반경 안에서도 장애물 회피력을 보강.

## 6. 알려진 한계 (2026-08-19 기준)

- **B→출발복귀 구간은 미검증** — 위 §3 참고. 지금까지의 모든 실측 튜닝(inflation_radius,
  cost_penalty, 병목우회 웨이포인트)은 출발→A 구간 하나만 검증된 것이라, B→출발 구간은
  같은 종류의 병목/lethal-space 문제가 재현될 수 있으니 실차 투입 전 시뮬레이션으로
  확인할 것.
- **좁은 통로(내부 기둥 사이 ~0.9m) 클리어런스가 빡빡함** — 로봇 외접원(`robot_radius`
  0.36m)을 그대로 쓰면 통로 폭 대비 여유가 편도 9cm 남짓뿐이라, 경로가 한쪽 장애물에
  붙어서 `controller_server`가 "Failed to make progress"/"detected collision ahead"로
  멈추는 경우가 실행마다 편차 있게 관측됨. 직사각형 `footprint`(0.64x0.31)로 바꿔 여유를
  늘려보려 했으나 RegulatedPurePursuit의 충돌예측 스윕 계산과 안 맞아 스폰 직후부터
  false-positive 충돌 감지로 더 나빠지는 회귀가 나서 원형으로 되돌림 — 다음에 다시 시도한다면
  RPP 소스의 회전 footprint 충돌검사 로직을 먼저 확인할 것. `cost_scaling_factor`를
  3.0→8.0으로 올려 어느 정도 완화했지만 완전히 해결되진 않음.
- **Gazebo 물리 파라미터는 근사치** — 차량 URDF의 질량/관성, ackermann_drive 플러그인 PID
  게인(`urdf/xycar.urdf`)은 실측이 아니라 추정/스케일링값(데모의 1300kg급 차량 기준
  게인을 그대로 쓰면 3kg대 이 차에선 즉시 물리 발산(NaN)하는 것을 확인해 대폭 축소함).
  조향·가감속 응답이 실차와 정확히 같지 않을 수 있음. 벽/바퀴 접촉 강성(kp/kd)도
  벽 쪽만 완화했다 — 바퀴/차체 쪽에 같은 완화를 넣으면 오히려 지면 접촉이 불안정해져서
  주행 시작하자마자 NaN이 나는 회귀가 있었으니 건드리지 말 것.
- **최종 정밀접근 단계 편차** — 시뮬레이션에서 목표 부근(5cm/3도 이내로 마지막 정렬하는 구간)
  진행 속도가 실행마다 다르게 나타나는 경우를 관측함. 코어 파이프라인(스폰→로컬라이제이션→
  경로계획→주행)은 반복 검증됨.
- **라이다 장착 오프셋 미실측** — `lidar_x`/`lidar_y`/`lidar_yaw_deg`가 플레이스홀더(0)로 남아
  있음. 실차에서 줄자로 재측정 후 launch 인자 기본값과 `urdf/xycar.urdf`의 센서 마운트 pose를
  같이 갱신할 것.
- **`ROS_DOMAIN_ID` 환경변수 주의** — 이 계정 `.bashrc`가 `ROS_DOMAIN_ID=7`을 설정한다.
  같은 터미널/셸 환경에서 `ros2` 명령을 실행해야 서로의 노드가 보인다(다른 도메인이면
  launch는 정상 동작해도 다른 셸에서 `ros2 topic list` 등으로 관찰이 안 됨).
