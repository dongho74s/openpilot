#!/usr/bin/env python3
"""기기에서 직접 실행하는 조향 진동 분석"""
import sys
sys.path.insert(0, '/data/openpilot')

from openpilot.tools.lib.logreader import LogReader
import numpy as np
from pathlib import Path

# 가장 최근 로그 찾기
base = Path('/data/media/0/realdata')
segs = sorted([d for d in base.iterdir() if d.is_dir() and '--' in d.name],
              key=lambda x: x.stat().st_mtime, reverse=True)

# 업로드된 파일과 같은 세그먼트 찾기 (rlog_6_2292)
# 또는 가장 최근 주행 로그 사용
target = None
for seg in segs[:10]:
    if (seg / 'rlog.zst').exists():
        target = seg
        break

if not target:
    print("로그 없음")
    sys.exit(1)

print(f"분석 중: {target.name}")
print()

lr = LogReader(str(target / 'rlog.zst'))

times = []
steer = []
vego = []

for msg in lr:
    if msg.which() == 'carState':
        cs = msg.carState
        times.append(msg.logMonoTime / 1e9)
        steer.append(cs.steeringAngleDeg)
        vego.append(cs.vEgo)
    if len(times) > 10000:
        break

times = np.array(times)
steer = np.array(steer)
vego = np.array(vego)
times = times - times[0]

print(f"샘플 수: {len(times)}, 시간: {times[-1]:.1f}초")
print(f"평균 속도: {vego.mean()*3.6:.0f} km/h")
print(f"조향각: 평균={steer.mean():.3f}°, 표준편차={steer.std():.3f}°")
print(f"조향각 범위: [{steer.min():.2f}°, {steer.max():.2f}°]")
print()

# 진동 분석: 고주파 성분
# 1초 이동평균으로 트렌드 제거 후 잔차 분석
window = 100  # 1초 (100Hz 가정)
if len(steer) > window:
    trend = np.convolve(steer, np.ones(window)/window, mode='same')
    residual = steer - trend
    print(f"진동 (트렌드 제거 후):")
    print(f"  표준편차: {residual.std():.4f}°")
    print(f"  최대 편차: {abs(residual).max():.3f}°")
    
    # 0.1° 이상 진동이 전체의 몇 %인지
    pct = np.mean(abs(residual) > 0.1) * 100
    print(f"  0.1° 초과 비율: {pct:.1f}%")
    
    if residual.std() > 0.05:
        print()
        print("→ 진동이 심함. Kp를 낮추거나 Kd를 높이세요.")
    elif residual.std() > 0.02:
        print()
        print("→ 약한 진동. Friction을 조정해보세요.")
    else:
        print()
        print("→ 진동이 양호한 수준입니다.")
