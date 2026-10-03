#!/usr/bin/env python3
"""BSM 파서 진단 - 간단 버전 (import 없이 파일 직접 확인)"""
import subprocess
from pathlib import Path

print("=== BSM 파서 진단 ===")
print()

# 1. DBC에 메시지가 있는지 확인
print("1. DBC 확인:")
result = subprocess.run(
    ['grep', '-c', 'BCMBlindSpotMonitor',
     '/data/openpilot/opendbc_repo/opendbc/dbc/gm_global_a_powertrain_volt.dbc'],
    capture_output=True, text=True
)
count = result.stdout.strip()
print(f"   BCMBlindSpotMonitor in DBC: {count}회")
if count != "0":
    print("   ✓ DBC에 메시지 정의 있음")
else:
    print("   ✗ DBC에 없음!")

print()

# 2. carstate.py에 파서 추가되어 있는지 확인
print("2. 파서 설정 확인:")
carstate_path = Path('/data/openpilot/opendbc_repo/opendbc/car/gm/carstate.py')
content = carstate_path.read_text()

if 'BCMBlindSpotMonitor' in content:
    print("   ✓ carstate.py에 BCMBlindSpotMonitor 언급됨")
    
    # get_can_parsers 섹션 찾기
    if 'def get_can_parsers' in content:
        # get_can_parsers부터 다음 def까지의 섹션 추출
        start = content.find('def get_can_parsers')
        end = content.find('\n  def ', start + 1)
        section = content[start:end]
        
        if 'BCMBlindSpotMonitor' in section:
            print("   ✓ get_can_parsers에 추가됨")
            if '"BCMBlindSpotMonitor"' in section or "'BCMBlindSpotMonitor'" in section:
                print("   ✓ 파서 메시지 목록에 있음")
        else:
            print("   ✗ get_can_parsers에 없음!")
else:
    print("   ✗ carstate.py에 없음!")

print()

# 3. update()에서 읽는지 확인
print("3. 데이터 읽기 확인:")
if 'pt_cp.vl["BCMBlindSpotMonitor"]' in content or "pt_cp.vl['BCMBlindSpotMonitor']" in content:
    print("   ✓ update()에서 BCMBlindSpotMonitor 읽음")
else:
    print("   ✗ update()에서 읽지 않음!")

print()
print("=== 진단 완료 ===")
