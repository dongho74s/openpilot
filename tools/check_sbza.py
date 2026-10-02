#!/usr/bin/env python3
"""
SBZA CAN 메시지 확인 스크립트
Trailblazer에서 SBZA (Side Blind Zone Alert) CAN 메시지가 나오는지 확인

사용법:
  python3 check_sbza.py /data/media/0/realdata/<route>/<segment>/rlog.zst
"""
import sys
from pathlib import Path

# SBZA CAN IDs (decimal)
SBZA_IDS = {
    2155012096: "SBZA_Right_Status_LS",
    2155003904: "Side_Blind_Zone_Alert_Status",
}

def check_log(log_path):
    log_path = Path(log_path)
    if not log_path.exists():
        print(f"파일 없음: {log_path}")
        return
    
    print(f"검사 중: {log_path}")
    print(f"찾는 CAN ID: {list(SBZA_IDS.keys())}")
    
    try:
        # zstd 압축 해제 후 읽기
        import subprocess
        proc = subprocess.Popen(
            ['zstd', '-d', '-c', str(log_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        
        # openpilot LogReader 사용 시도
        try:
            from openpilot.tools.lib.logreader import LogReader
            import io
            
            # 스트림을 읽어서 LogReader에 전달
            # (실제로는 파일 경로를 직접 주는 게 더 간단)
            print("LogReader로 파싱 시도...")
            
        except ImportError:
            print("LogReader 없음, 바이너리 검색으로 대체")
            # 바이너리에서 CAN ID 패턴 검색 (간단한 휴리스틱)
            data = proc.stdout.read(10 * 1024 * 1024)  # 처음 10MB만
            found = []
            for can_id, name in SBZA_IDS.items():
                # CAN ID를 리틀엔디안 4바이트로 검색
                import struct
                pattern = struct.pack('<I', can_id)
                if pattern in data:
                    found.append((can_id, name))
            
            if found:
                print("\n발견된 SBZA 메시지:")
                for can_id, name in found:
                    print(f"  - {can_id}: {name}")
            else:
                print("\nSBZA 메시지 없음 (처음 10MB에서)")
        
        proc.terminate()
        
    except Exception as e:
        print(f"오류: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    check_log(sys.argv[1])
