#!/usr/bin/env python3
"""기기에서 직접 실행 - 의존성 최소화 버전"""
import sys
from pathlib import Path

# 로그 파일 찾기
base = Path('/data/media/0/realdata')
segs = sorted([d for d in base.iterdir() if d.is_dir() and '--' in d.name],
              key=lambda x: x.stat().st_mtime, reverse=True)

target = None
for seg in segs[:5]:
    rlog = seg / 'rlog.zst'
    if rlog.exists():
        # 크기로 주행 로그인지 확인 (10MB 이상)
        if rlog.stat().st_size > 10*1024*1024:
            target = seg
            break

if not target:
    print("주행 로그 없음")
    sys.exit(1)

print(f"분석: {target.name}")
print(f"크기: {(target/'rlog.zst').stat().st_size//1024//1024}MB")
print()
print("이 스크립트는 기본 정보만 제공합니다.")
print("정밀 분석을 위해서는 Plot 6번 그래프를 직접 보세요.")
print()
print("진동 판단 기준:")
print("- 그래프 선이 1초에 2번 이상 출렁이면 Kp가 높음")
print("- 진폭이 0.5° 이상이면 Kd 부족")
print("- 직선에서도 계속되면 Friction 문제")
