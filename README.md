# parking_mission

자율주행 주차미션 — ROS2 Humble + Nav2(AMCL/costmap/planner/controller) 기반으로
출발영역에서 지정된 주차영역(A 또는 B)까지 자율주행 후 정차하는 패키지.

패키지 실체는 `ROS2_ws/src/` 아래에 있다(colcon 워크스페이스: `ROS2_ws`, 소스: `src`).

## 실행

```bash
cd ~/xycar_ws   # ROS2_ws/src를 이 워크스페이스의 src 아래에 심볼릭링크/복사해서 사용
colcon build --packages-select parking_mission
source install/setup.bash
ros2 launch parking_mission parking_mission.launch.py parking_zone:=A
```

- `parking_zone:=A` 또는 `B`로 목표 주차영역 선택 (기본값 A)
- `use_sim_time:=true`로 시뮬레이션 전환 가능
- `bringup_sensors:=false`로 라이다/IMU를 이 launch에서 안 띄우게 할 수 있음(다른 launch가
  이미 띄운 경우)

## 2026-08-18 — 벽 충돌("차가 벽에 부딪히고 멈추는 현상") 개선

실차 테스트에서 차량이 주행 중 벽에 부딪혀 멈추는 증상이 보고됨. 라이다+카메라만 쓰는
제약 하에, `config/nav2_params.yaml`의 팽창영역(inflation)과 플래너를 점검해 세 가지를
수정했다.

### 1. 팽창영역(inflation_radius) 버그 수정

기존 `inflation_radius`(0.3m)가 `robot_radius`(0.36m)보다 **작게** 설정돼 있었다. Nav2
costmap의 팽창 비용 구배는 `robot_radius` 밖에서부터 `inflation_radius`까지 서서히
감소하는 구조인데, `inflation_radius < robot_radius`면 그 구배가 사실상 차체 반경
안쪽에서 끝나버려 "벽에 거의 닿아야 비용이 오른다"는 의미가 된다 — 팽창영역이 안전마진
역할을 거의 못 하고 있었던 것.

- `inflation_radius`: 0.3m → **0.45m** (robot_radius 0.36m + 여유 0.09m)
- `cost_scaling_factor`: 3.0 → 4.0
- local_costmap / global_costmap 둘 다 동일하게 수정

### 2. 플래너 교체 — NavfnPlanner → SmacPlannerHybrid (Modified A*)

기존 `nav2_navfn_planner::NavfnPlanner`(격자 기반 순수 A*)는 차량의 회전반경을 모른 채
경로를 짠다. 이 차는 Ackermann 조향이라, 격자 A* 경로에 코너에서 급하게 꺾이는 지점이
생기면 실제 추종 제어기가 그 꺾임을 못 따라가고 코너를 잘라먹으며 벽을 스칠 수 있다.

`nav2_smac_planner::SmacPlannerHybrid`(Nav2 표준 Hybrid-A*)로 교체해 최소 회전반경
제약을 넣었다 — 계획 단계부터 "차가 실제로 따라갈 수 있는" 곡선 경로만 생성한다.

- `minimum_turning_radius`: 0.6m
  (실측 축거 `WHEELBASE_M=0.335m` 기준, UMK `planner/hybrid_astar.py`와 동일하게
  최대조향각 30도(설계값, 실측 아님)를 가정해 `r = wheelbase / tan(30°) ≈ 0.58m` 산출 후
  여유를 더함)
- `motion_model_for_search: REEDS_SHEPP` — 전진만 가능한 DUBIN 대신 후진도 허용.
  주차는 후진 진입이 필요한 경우가 많음.

### 3. 컨트롤러(DWB) 장애물 회피 가중치 상향

플래너가 안전한 경로를 짜도, 실행 중 컨트롤러(DWB)가 장애물 회피보다 경로추종을 너무
우선시하면 여전히 벽에 붙을 수 있다. 표준 turtlebot3 템플릿에서 그대로 가져온
`BaseObstacle.scale`(0.02)이 `PathAlign`/`PathDist`(각 32.0) 등 경로추종 비용에 비해
지나치게 낮았다.

- `BaseObstacle.scale`: 0.02 → **0.5**

### 알려진 한계 / 실차 확인 필요

- `minimum_turning_radius=0.6m`은 실측이 아니라 조향각 설계값(30도) 역산치 — 실차에서
  너무 좁은 코너를 못 들어가면 낮추고, 진동/오버슈트가 있으면 올릴 것.
- `inflation_radius=0.45m`가 출발영역(0.7m×0.3m)처럼 좁은 구역에서 계획 자체를 막지
  않는지 실차에서 확인 필요 — 막히면 0.35~0.4 정도로 낮출 것.
- 위 세 가지는 팽창영역/플래너/컨트롤러 3단계 각각의 안전마진이라, 여전히 벽에
  부딪힌다면 어느 단계에서 실패하는지(계획 자체가 벽에 붙는지, 계획은 괜찮은데 추종이
  못 따라가는지) RViz의 global/local costmap과 계획 경로를 같이 보며 구분할 것.
