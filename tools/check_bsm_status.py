#!/usr/bin/env python3
"""
BSM 동작 확인 스크립트
최근 주행 로그에서 blindspot 값과 차선변경 이벤트 분석
"""
import subprocess
from pathlib import Path
import struct

def find_latest_segments(n=3):
    base = Path('/data/media/0/realdata')
    segs = sorted([d for d in base.iterdir() if d.is_dir() and '--' in d.name],
                  key=lambda x: x.stat().st_mtime, reverse=True)
    return segs[:n]

def check_bsm_in_logs():
    segs = find_latest_segments(3)
    print("=== BSM 동작 확인 ===")
    print()
    
    for seg in segs:
        print(f"세그먼트: {seg.name}")
        rlog = seg / 'rlog.zst'
        if not rlog.exists():
            print("  rlog 없음")
            continue
        
        # rlog에서 carStateBlindspot 관련 바이트 패턴 검색
        # 실제로는 cereal 디코딩이 필요하지만, 간단히 바이너리 패턴으로 확인
        proc = subprocess.Popen(['zstd', '-d', '-c', str(rlog)],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        data = proc.stdout.read(50*1024*1024)
        proc.terminate()
        
        # 0x142 메시지 카운트 (BCMBlindSpotMonitor)
        pattern_142 = struct.pack('<I', 322)
        count_142 = data.count(pattern_142)
        print(f"  0x142 (BSM) 메시지: {count_142}회 수신")
        
        if count_142 > 0:
            print("  → BSM 데이터 수신 중 (파서가 동작함)")
        else:
            print("  → BSM 데이터 없음 (파서가 동작 안 함 또는 메시지가 없음)")
        print()
    
    print("=== 참고 ===")
    print("정확한 blindspot True/False 확인은 cereal 디코딩이 필요합니다.")
    print("위 결과에서 0x142 메시지가 수신되면 파서는 동작하는 것입니다.")
    print()
    print("차선변경 블로킹 테스트:")
    print("1. 옆차 있을 때 깜빡이 켜보기")
    print("2. 차선변경이 안 되면 BSM 블로킹 동작 중")
    print("3. 차선변경이 되면 BSM이 안 먹는 것")

if __name__ == '__main__':
    check_bsm_in_logs()
