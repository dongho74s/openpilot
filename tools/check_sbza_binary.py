#!/usr/bin/env python3
"""
SBZA CAN ID 확인 - 바이너리 검색 버전
의존성 없이 rlog.zst에서 CAN ID 패턴 직접 검색
"""
import struct
import subprocess
import sys
from pathlib import Path

SBZA_IDS = {
    2155012096: "SBZA_Right_Status_LS",
    2155003904: "Side_Blind_Zone_Alert_Status",
}

def find_recent_logs():
    base = Path('/data/media/0/realdata')
    segments = [d for d in base.iterdir() if d.is_dir() and '--' in d.name]
    segments = sorted(segments, key=lambda x: x.stat().st_mtime, reverse=True)
    logs = []
    for seg in segments[:5]:
        rlog = seg / 'rlog.zst'
        if rlog.exists():
            logs.append(rlog)
            if len(logs) >= 3:
                break
    return logs

def check_log_binary(log_path):
    """zstd 해제 후 바이너리에서 CAN ID 검색"""
    print(f"\n검사: {log_path.parent.name}")
    try:
        # zstd로 해제 (처음 50MB만)
        proc = subprocess.Popen(
            ['zstd', '-d', '-c', str(log_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        data = proc.stdout.read(50 * 1024 * 1024)
        proc.terminate()
        proc.wait()
        
        print(f"  읽은 데이터: {len(data) // 1024 // 1024}MB")
        
        found = []
        for can_id, name in SBZA_IDS.items():
            # 리틀엔디안 4바이트 패턴
            pattern_le = struct.pack('<I', can_id)
            # 빅엔디안 4바이트 패턴
            pattern_be = struct.pack('>I', can_id)
            
            count_le = data.count(pattern_le)
            count_be = data.count(pattern_be)
            
            if count_le > 0 or count_be > 0:
                found.append((can_id, name, count_le + count_be))
        
        if found:
            print(f"  발견!")
            for can_id, name, count in found:
                print(f"    - {can_id} ({name}): {count}회")
            return True
        else:
            print(f"  없음")
            return False
            
    except Exception as e:
        print(f"  오류: {e}")
        return False

if __name__ == '__main__':
    logs = find_recent_logs()
    if not logs:
        print("로그 없음")
        sys.exit(1)
    
    print(f"최근 로그 {len(logs)}개 바이너리 검사")
    print(f"찾는 ID: {list(SBZA_IDS.keys())}")
    
    found_any = False
    for log in logs:
        if check_log_binary(log):
            found_any = True
    
    print("\n" + "="*50)
    if found_any:
        print("결론: SBZA 메시지 있음! BSM 연동 가능")
    else:
        print("결론: SBZA 메시지 없음 (또는 다른 형식)")
        print("참고: 바이너리 검색은 완벽하지 않음")
