#!/usr/bin/env python3
"""
CarrotPilot 실시간 에러 모니터
- openpilot 시스템 로그를 실시간으로 감시
- Python 예외, 크루즈/블루투스 관련 오류를 감지
- 감지된 오류를 전용 로그 파일에 기록
- 사용자가 나중에 분석할 수 있도록 타임스탬프와 컨텍스트 포함

사용법:
  python3 carrot_error_monitor.py &
  # 또는 systemd 서비스로 등록

로그 위치: /data/carrot_error_monitor.log
"""

import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ERROR_LOG = Path("/data/carrot_error_monitor.log")
MAX_LOG_SIZE = 10 * 1024 * 1024  # 10MB

# 감지할 에러 패턴
ERROR_PATTERNS = [
    # Python 예외
    (re.compile(r'Traceback \(most recent call last\)'), 'PYTHON_TRACEBACK'),
    (re.compile(r'(\w+Error): (.+)'), 'PYTHON_ERROR'),
    (re.compile(r'Exception: (.+)'), 'EXCEPTION'),
    
    # 크루즈 관련
    (re.compile(r'cruise.*error', re.IGNORECASE), 'CRUISE_ERROR'),
    (re.compile(r'VCruise.*Error'), 'VCRUISE_ERROR'),
    (re.compile(r'Bluetooth.*(error|fail)', re.IGNORECASE), 'BT_ERROR'),
    
    # 블루투스 리모콘 관련 (RES/SET)
    (re.compile(r'resumeCruise|setCruise.*(error|fail|exception)', re.IGNORECASE), 'BT_REMOTE_ERROR'),
    
    # 일반적인 오류
    (re.compile(r'\bFAILED\b'), 'FAILED'),
    (re.compile(r'\bFATAL\b'), 'FATAL'),
]

# 제외할 패턴 (노이즈)
EXCLUDE_PATTERNS = [
    re.compile(r'test.*passed', re.IGNORECASE),
    re.compile(r'no error', re.IGNORECASE),
]


def log_error(category, line, context_lines=None):
    """에러를 전용 로그 파일에 기록"""
    timestamp = datetime.now().isoformat()
    
    # 로그 파일 크기 관리 (로테이션)
    if ERROR_LOG.exists() and ERROR_LOG.stat().st_size > MAX_LOG_SIZE:
        ERROR_LOG.rename(ERROR_LOG.with_suffix('.log.1'))
    
    with open(ERROR_LOG, 'a') as f:
        f.write(f"\n{'='*60}\n")
        f.write(f"[{timestamp}] {category}\n")
        f.write(f"{'='*60}\n")
        if context_lines:
            for ctx in context_lines[-5:]:  # 최근 5줄 컨텍스트
                f.write(f"  | {ctx}\n")
        f.write(f"  > {line}\n")
    
    print(f"[{timestamp}] {category}: {line[:100]}", flush=True)


def should_exclude(line):
    """노이즈 제외"""
    return any(p.search(line) for p in EXCLUDE_PATTERNS)


def check_line(line, context_buffer):
    """한 줄을 검사하고 에러면 기록"""
    if should_exclude(line):
        return
    
    for pattern, category in ERROR_PATTERNS:
        if pattern.search(line):
            log_error(category, line.strip(), list(context_buffer))
            break


def monitor_journal():
    """systemd journal 실시간 모니터링"""
    print("Starting journal monitor...", flush=True)
    proc = subprocess.Popen(
        ['journalctl', '-f', '-o', 'cat', '-n', '0'],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1
    )
    
    context_buffer = []
    try:
        for line in proc.stdout:
            line = line.rstrip('\n')
            if not line.strip():
                continue
            
            context_buffer.append(line)
            if len(context_buffer) > 20:
                context_buffer.pop(0)
            
            check_line(line, context_buffer)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()


def monitor_logfile(log_path):
    """특정 로그 파일 tail"""
    print(f"Monitoring {log_path}...", flush=True)
    proc = subprocess.Popen(
        ['tail', '-F', '-n', '0', str(log_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1
    )
    
    context_buffer = []
    try:
        for line in proc.stdout:
            line = line.rstrip('\n')
            if not line.strip():
                continue
            
            context_buffer.append(line)
            if len(context_buffer) > 20:
                context_buffer.pop(0)
            
            check_line(line, context_buffer)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()


def main():
    print(f"CarrotPilot Error Monitor starting...", flush=True)
    print(f"Error log: {ERROR_LOG}", flush=True)
    
    # 에러 로그 파일 헤더
    with open(ERROR_LOG, 'a') as f:
        f.write(f"\n\n{'#'*60}\n")
        f.write(f"# Monitor started at {datetime.now().isoformat()}\n")
        f.write(f"{'#'*60}\n")
    
    # journalctl이 있으면 사용, 없으면 로그 파일 tail
    try:
        result = subprocess.run(['which', 'journalctl'], capture_output=True)
        if result.returncode == 0:
            monitor_journal()
        else:
            # 대체: 일반적인 로그 위치 시도
            for log_path in [
                Path("/data/openpilot/log"),
                Path("/var/log/syslog"),
            ]:
                if log_path.exists():
                    monitor_logfile(log_path)
                    break
            else:
                print("No log source found!", file=sys.stderr)
                sys.exit(1)
    except KeyboardInterrupt:
        print("\nMonitor stopped.", flush=True)


if __name__ == '__main__':
    main()
