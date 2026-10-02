#!/usr/bin/env python3
"""
SBZA CAN ID 확인 - 간단 버전
사용법: python3 check_sbza_simple.py
최근 로그에서 SBZA CAN ID가 있는지 확인
"""
import sys
sys.path.insert(0, '/data/openpilot')

from pathlib import Path

SBZA_IDS = {2155012096, 2155003904}

def find_recent_logs():
    base = Path('/data/media/0/realdata')
    if not base.exists():
        print("로그 디렉토리 없음")
        return []
    
    # 세그먼트 디렉토리 직접 찾기 (0000002a--xxx--N 형식)
    segments = [d for d in base.iterdir() if d.is_dir() and '--' in d.name]
    segments = sorted(segments, key=lambda x: x.stat().st_mtime, reverse=True)
    
    logs = []
    for seg in segments[:5]:  # 최근 5개 세그먼트
        rlog = seg / 'rlog.zst'
        if rlog.exists():
            logs.append(rlog)
            if len(logs) >= 3:
                break
    return logs

def check_log(log_path):
    print(f"\n검사: {log_path.name}")
    try:
        from openpilot.tools.lib.logreader import LogReader
        lr = LogReader(str(log_path))
        found_ids = set()
        count = 0
        for msg in lr:
            if msg.which() == 'can':
                for c in msg.can:
                    if c.address in SBZA_IDS:
                        found_ids.add(c.address)
                count += 1
                if count > 5000:  # 처음 5000개 메시지만
                    break
        
        if found_ids:
            print(f"  발견! SBZA ID: {found_ids}")
            return True
        else:
            print(f"  없음 (검사한 CAN 메시지: {count})")
            return False
    except Exception as e:
        print(f"  오류: {e}")
        return False

if __name__ == '__main__':
    logs = find_recent_logs()
    if not logs:
        print("로그 파일 없음")
        sys.exit(1)
    
    print(f"최근 로그 {len(logs)}개 검사 중...")
    print(f"찾는 ID: {SBZA_IDS}")
    
    found_any = False
    for log in logs:
        if check_log(log):
            found_any = True
    
    print("\n" + "="*50)
    if found_any:
        print("결론: SBZA 메시지가 있음! BSM 연동 가능")
    else:
        print("결론: SBZA 메시지 없음. 차량에서 안 보내는 듯")
