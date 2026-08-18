# EncoderPoseEstimator — UMK 저장소(track_drive/track_drive/localization/pose_estimator.py)의
# 동일 클래스를 그대로 옮겨왔다(독립 패키지라 import 대신 복제). 원본 설계 배경/한계는
# 그쪽 주석 참고. 휠 엔코더(선속도 v)와 조향각(delta) 또는 IMU yaw로 차량 pose(x,y,yaw)를
# 자전거모델 데드레커닝으로 적분 추적한다.
import math


class EncoderPoseEstimator:

    def __init__(self, wheelbase_m=None):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.wheelbase_m = wheelbase_m
        self._yaw_source = 'dead_reckoning'   # 'dead_reckoning' | 'imu'
        self._warned_no_wheelbase = False

    def set_yaw_source(self, source):
        assert source in ('dead_reckoning', 'imu'), f"알 수 없는 yaw_source: {source}"
        self._yaw_source = source

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw

    def update(self, v_mps, steer_rad, dt, imu_yaw=None):
        """매 제어주기 호출. v_mps: 선속도(m/s, 후진이면 음수). steer_rad: 조향각(rad,
        yaw_source='dead_reckoning'일 때만 사용). dt: 경과시간(s). imu_yaw: yaw_source='imu'일
        때만 전달(rad). 반환: (x, y, yaw)."""
        self.v = v_mps

        if self._yaw_source == 'imu':
            if imu_yaw is None:
                raise ValueError("yaw_source='imu'인데 imu_yaw가 전달되지 않았다")
            self.yaw = imu_yaw
        else:
            if self.wheelbase_m is None:
                if not self._warned_no_wheelbase:
                    print('[EncoderPoseEstimator] 경고: wheelbase_m 미실측 — yaw 적분 불가, yaw를 고정값으로 유지')
                    self._warned_no_wheelbase = True
            else:
                yaw_rate = (v_mps / self.wheelbase_m) * math.tan(steer_rad)
                self.yaw += yaw_rate * dt

        self.x += v_mps * math.cos(self.yaw) * dt
        self.y += v_mps * math.sin(self.yaw) * dt

        return self.x, self.y, self.yaw
