# 실차(xycar) 실측 스펙 — UMK 저장소(track_drive/config.py)의 실측값을 그대로 옮겨왔다.
# 이 패키지는 track_drive 패키지에 의존하지 않는 독립 패키지라 상수를 복제해서 쓴다.
# UMK/track_drive/track_drive/config.py 값이 바뀌면 이 파일도 같이 갱신할 것.

WHEELBASE_M = 0.335              # 축거(줄자 실측, UMK 2026-08-06)
VEHICLE_WIDTH_M = 0.31           # 차폭(실측, UMK 2026-08-04: 세로64cm x 가로31cm x 높이20cm)
VEHICLE_LENGTH_M = 0.64          # 차길이(실측, 위와 동일 출처)

VESC_SPEED_TO_ERPM_GAIN = 4614.0  # VESC 드라이버 vesc.yaml의 speed_to_erpm_gain (UMK 실차 확인값)

# 모터 명령속도(0~100 단위) -> 실제 지상속도(m/s) 환산.
# [2026-08-24] UMK track_drive/config.py의 재실측값(0.0848)으로 교체. 이전 값(0.068)은
# cmd=5(0.431m/s)와 cmd=20(1.449m/s) 두 점 선형회귀였는데, UMK가 그 후 cmd=20 데이터를
# "쓰지 말 것"으로 명시적으로 폐기했다(실제속도측정.md §4.7 — 그 속도대는 견인력
# 포화/슬립으로 반복측정이 불안정했음, 실측 확인). UMK가 대신 권장하는 값은 안정적인
# cmd=5/10/15 3점을 원점고정 회귀한 0.0848 — 이 저장소의 max_vel_x(0.3~0.4 m/s)는
# 항상 그 저속 구간 안에 들어오므로(오히려 UMK 원래 사용처인 트랙 주행보다 이 주차
# 미션에 더 잘 맞는 값), 같은 실차의 이 값을 그대로 재사용한다.
METERS_PER_SPEED_UNIT = 0.0848

ANGLE_MAX_DEG = 80.0              # 조향각 클램프(UMK config.py ANGLE_MAX와 동일)

VESC_STALE_SEC = 0.5              # 마지막 /vesc_speed_erpm 수신 후 이 시간 지나면 죽었다고 판단(UMK와 동일)
IMU_STALE_SEC = 0.5                # 마지막 /imu 수신 후 이 시간 지나면 죽었다고 판단(UMK와 동일)
