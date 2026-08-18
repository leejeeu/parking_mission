# 실차(xycar) 실측 스펙 — UMK 저장소(track_drive/config.py)의 실측값을 그대로 옮겨왔다.
# 이 패키지는 track_drive 패키지에 의존하지 않는 독립 패키지라 상수를 복제해서 쓴다.
# UMK/track_drive/track_drive/config.py 값이 바뀌면 이 파일도 같이 갱신할 것.

WHEELBASE_M = 0.335              # 축거(줄자 실측, UMK 2026-08-06)
VEHICLE_WIDTH_M = 0.31           # 차폭(실측, UMK 2026-08-04: 세로64cm x 가로31cm x 높이20cm)
VEHICLE_LENGTH_M = 0.64          # 차길이(실측, 위와 동일 출처)

VESC_SPEED_TO_ERPM_GAIN = 4614.0  # VESC 드라이버 vesc.yaml의 speed_to_erpm_gain (UMK 실차 확인값)

# 모터 명령속도(0~100 단위) -> 실제 지상속도(m/s) 환산.
# UMK track_drive/실제속도측정.md(2026-08-17/18 실측)의 최신 회귀 기반 대표값 사용:
#   cmd=5  -> 0.431 m/s
#   cmd=20 -> 1.449 m/s (5회 run 중 이상치 제외한 대표값)
# 두 점으로 선형회귀하면 대략 speed_unit당 ~0.068 m/s, x절편(데드존) 존재.
METERS_PER_SPEED_UNIT = 0.068

ANGLE_MAX_DEG = 80.0              # 조향각 클램프(UMK config.py ANGLE_MAX와 동일)

VESC_STALE_SEC = 0.5              # 마지막 /vesc_speed_erpm 수신 후 이 시간 지나면 죽었다고 판단(UMK와 동일)
IMU_STALE_SEC = 0.5                # 마지막 /imu 수신 후 이 시간 지나면 죽었다고 판단(UMK와 동일)
