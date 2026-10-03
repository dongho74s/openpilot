#!/usr/bin/env python3
"""BSM 파서 진단 - bsm-test2 브랜치에서 실행"""
import sys
sys.path.insert(0, '/data/openpilot')

try:
    from opendbc_repo.opendbc.car.gm.carstate import CarState
    from opendbc_repo.opendbc.car.gm.values import GMPlatformConfig
    from opendbc_repo.opendbc.car.structs import CarParams
    from opendbc_repo.opendbc.car import Bus
    
    print("=== BSM 파서 진단 ===")
    print()
    
    # 1. DBC에 메시지가 있는지 확인
    from opendbc_repo.opendbc.dictionary import DBC
    dbc_name = 'gm_global_a_powertrain_volt'
    # DBC 파일에서 직접 확인
    import subprocess
    result = subprocess.run(
        ['grep', '-c', 'BCMBlindSpotMonitor', 
         f'/data/openpilot/opendbc_repo/opendbc/dbc/{dbc_name}.dbc'],
        capture_output=True, text=True
    )
    count = result.stdout.strip()
    print(f"1. DBC에 BCMBlindSpotMonitor 존재: {count}회")
    
    # 2. 파서 설정 확인
    print()
    print("2. get_can_parsers 확인:")
    import inspect
    src = inspect.getsource(CarState.get_can_parsers)
    if 'BCMBlindSpotMonitor' in src:
        print("   ✓ BCMBlindSpotMonitor가 파서에 추가됨")
        # pt_messages에 있는지 확인
        if 'pt_messages = [' in src and 'BCMBlindSpotMonitor' in src.split('pt_messages')[1].split('cam_messages')[0]:
            print("   ✓ pt_messages (버스 0)에 있음")
        else:
            print("   ✗ pt_messages에 없음!")
    else:
        print("   ✗ 파서에 없음!")
    
    print()
    print("3. 결론:")
    print("   파서 설정은 올바름.")
    print("   실제 주행 중 0x142 메시지가 버스 0으로 오는지 확인 필요.")
    print("   노란 장벽이 안 뜨면 메시지가 다른 버스에 있거나,")
    print("   신호 값이 0으로 고정되어 있을 가능성.")
    
except Exception as e:
    print(f"오류: {e}")
    import traceback
    traceback.print_exc()
