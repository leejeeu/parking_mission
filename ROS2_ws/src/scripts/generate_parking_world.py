#!/usr/bin/env python3
#=============================================
# generate_parking_world.py — maps/parking_map.pgm(+.yaml)를 Gazebo Classic SDF 월드로
#   변환한다. AMCL이 실제로 map_server가 로드하는 것과 "같은" 벽을 스캔매칭하도록,
#   점유 픽셀을 그대로 벽 박스로 압출(extrude)한다 — 손으로 그린 placeholder 월드가
#   아니라 실측 SLAM 맵 자체를 3D화한 것.
#
# 좌표 변환 규칙(ROS map_server와 동일하게 재현해야 함):
#   pgm 이미지는 row 0이 "위쪽"이지만, map.yaml의 origin은 이미지 좌하단 픽셀의 world
#   좌표다. 그래서 row(위에서부터)를 그대로 y로 쓰면 상하가 뒤집힌다 —
#   row_from_bottom = height-1-row 로 뒤집은 뒤에 origin_y를 더해야 한다.
#
# 점유 판정(occ)도 map_server 규칙 그대로: negate=0이면 밝은 픽셀(255)이 free,
#   어두운 픽셀(0)이 occupied이므로 occ = (255-pixel)/255. occ > occupied_thresh면 벽.
#
# 벽 하나당 박스 하나면 143x149px 맵에서 수천 개가 나올 수 있어 Gazebo가 버벅인다 —
#   같은 행(row) 안에서 연속된 occupied 컬럼을 run-length로 묶어 가늘고 긴 박스
#   하나로 압축한다(열 방향 병합은 안 함 — 행 단위만으로도 이 맵 크기에선 충분히
#   가벼워서 복잡도를 더 안 들였다).
#
# 맵이 나중에 다시 SLAM으로 갱신되면 이 스크립트를 재실행해서 worlds/parking_map.world를
#   다시 만들면 된다(1회성 산출물이 아니라 재실행 가능한 변환기로 저장소에 남겨둠).
#=============================================
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

SRC_DIR = Path(__file__).resolve().parent.parent
MAP_YAML = SRC_DIR / 'maps' / 'parking_map.yaml'
OUT_WORLD = SRC_DIR / 'worlds' / 'parking_map.world'

WALL_HEIGHT_M = 0.4   # 라이다 장착 높이(~0.05m)보다 확실히 크게 — 2D 스캔 평면이 항상 벽을 본다.


def load_map(yaml_path: Path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    image_path = yaml_path.parent / meta['image']
    img = Image.open(image_path).convert('L')
    arr = np.array(img, dtype=np.float64)

    negate = meta.get('negate', 0)
    if negate:
        occ = arr / 255.0
    else:
        occ = (255.0 - arr) / 255.0

    occupied = occ > meta['occupied_thresh']
    return occupied, meta['resolution'], meta['origin']


def row_runs(occupied_row):
    """occupied_row(bool 배열)에서 연속 True 구간의 (start_col, end_col_inclusive) 목록."""
    runs = []
    start = None
    for col, val in enumerate(occupied_row):
        if val and start is None:
            start = col
        elif not val and start is not None:
            runs.append((start, col - 1))
            start = None
    if start is not None:
        runs.append((start, len(occupied_row) - 1))
    return runs


def wall_sdf(name, x_center, y_center, length_x, thickness_y):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x_center:.4f} {y_center:.4f} {WALL_HEIGHT_M / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{length_x:.4f} {thickness_y:.4f} {WALL_HEIGHT_M:.4f}</size></box>
          </geometry>
          <!-- ODE 기본 접촉강성(kp~1e12)이 이 차(전체 3kg대)에겐 너무 뻣뻣해서 벽에
               살짝만 닿아도 폭발적으로 튕겨나가는 현상을 실측 확인함(2026-08-19,
               사용자 재현). xycar.urdf의 차량측 접촉 파라미터와 동일한 값으로 완화
               (양쪽 다 뻣뻣하면 둘 중 하나만 낮춰도 접촉 강성은 두 표면의 조합으로
               정해지므로 벽 쪽도 같이 낮춘다). -->
          <surface>
            <contact>
              <ode>
                <kp>20000.0</kp>
                <kd>10.0</kd>
                <max_vel>0.5</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{length_x:.4f} {thickness_y:.4f} {WALL_HEIGHT_M:.4f}</size></box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(occupied, resolution, origin):
    origin_x, origin_y, _ = origin
    height, width = occupied.shape

    walls = []
    wall_count = 0
    for row in range(height):
        for col_start, col_end in row_runs(occupied[row]):
            row_from_bottom = height - 1 - row
            x_start = origin_x + col_start * resolution
            x_end = origin_x + (col_end + 1) * resolution
            y_center = origin_y + row_from_bottom * resolution + resolution / 2.0
            x_center = (x_start + x_end) / 2.0
            length_x = x_end - x_start

            wall_count += 1
            walls.append(wall_sdf(f'wall_{wall_count}', x_center, y_center, length_x, resolution))

    walls_sdf = ''.join(walls)

    return f"""<?xml version="1.0"?>
<!--
  parking_map.world — scripts/generate_parking_world.py로 maps/parking_map.pgm에서
  자동 생성됨. 맵이 바뀌면 이 스크립트를 재실행해서 다시 만들 것(직접 손으로 수정 X).
  벽 {wall_count}개, 해상도 {resolution}m/px, origin {origin}.
-->
<sdf version="1.6">
  <world name="parking_mission">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

{walls_sdf}
  </world>
</sdf>
"""


def main():
    if not MAP_YAML.exists():
        print(f'맵 yaml을 찾을 수 없음: {MAP_YAML}', file=sys.stderr)
        sys.exit(1)

    occupied, resolution, origin = load_map(MAP_YAML)
    world_xml = build_world(occupied, resolution, origin)

    OUT_WORLD.parent.mkdir(parents=True, exist_ok=True)
    OUT_WORLD.write_text(world_xml)
    print(f'생성 완료: {OUT_WORLD} ({world_xml.count("<model name=")}개 벽 모델)')


if __name__ == '__main__':
    main()
