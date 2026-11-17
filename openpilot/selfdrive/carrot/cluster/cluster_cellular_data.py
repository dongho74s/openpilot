"""Cellular (ppp0) data usage provider for the external cluster HUD.

Reads the month-to-date total from vnstat (when the daemon is running with
its database at /data/vnstat) and the live transfer rate / session total
from the ppp0 interface counters. The HUD shows the monthly total against
the 15GB plan plus a live rate while data is flowing, e.g.
"데이터 8.5GB/15GB ↓12KB/s".

Never raises: any failure yields None so the HUD simply hides the text.
"""

from __future__ import annotations

import json
import subprocess
import threading
import time
from datetime import datetime

CELLULAR_DATA_PLAN_BYTES = 15 * 1024 ** 3  # JOY텔레콤 15GB/month
CELLULAR_DATA_PLAN_LABEL = "15GB"
VNSTAT_DB_DIR = "/data/vnstat"
VNSTAT_IFACE = "ppp0"
PPP0_RX_PATH = "/sys/class/net/ppp0/statistics/rx_bytes"
PPP0_TX_PATH = "/sys/class/net/ppp0/statistics/tx_bytes"
MONTHLY_REFRESH_SECONDS = 60.0
LIVE_REFRESH_SECONDS = 2.0
RATE_SHOW_THRESHOLD_BPS = 1024.0


def format_bytes(value: float) -> str:
    """Format a byte count compactly: 8.5GB / 12.3MB / 456KB / 12B."""
    if value >= 1024 ** 3:
        return f"{value / 1024 ** 3:.1f}GB"
    if value >= 1024 ** 2:
        return f"{value / 1024 ** 2:.1f}MB"
    if value >= 1024:
        return f"{value / 1024:.0f}KB"
    return f"{value:.0f}B"


def format_rate(bytes_per_s: float) -> str:
    """Format a transfer rate compactly: 12KB/s / 1.2MB/s."""
    if bytes_per_s >= 1024 ** 2:
        return f"{bytes_per_s / 1024 ** 2:.1f}MB/s"
    return f"{bytes_per_s / 1024:.0f}KB/s"


def format_cellular_data_text(
    monthly_bytes: int | None,
    session_bytes: int | None,
    rate_bps: float | None,
) -> str | None:
    """Build the one-line HUD string, or None when there is nothing to show.

    The monthly total from vnstat is preferred ("데이터 8.5GB/15GB").
    Without vnstat, fall back to the live session total ("데이터 8.5MB").
    A live transfer rate is appended only while data is actually flowing.
    """
    if monthly_bytes is not None:
        text = f"데이터 {format_bytes(monthly_bytes)}/{CELLULAR_DATA_PLAN_LABEL}"
    elif session_bytes is not None:
        text = f"데이터 {format_bytes(session_bytes)}"
    else:
        return None
    if rate_bps is not None and rate_bps >= RATE_SHOW_THRESHOLD_BPS:
        text += f" ↓{format_rate(rate_bps)}"
    return text


def _read_ppp0_counters() -> tuple[int, int] | None:
    """Read the live ppp0 rx/tx byte counters. None when cellular is down."""
    try:
        with open(PPP0_RX_PATH, encoding="ascii") as f:
            rx = int(f.read().strip())
        with open(PPP0_TX_PATH, encoding="ascii") as f:
            tx = int(f.read().strip())
    except (OSError, ValueError):
        return None
    return rx, tx


def _read_vnstat_monthly_bytes() -> int | None:
    """Month-to-date rx+tx bytes from vnstat. None when vnstat has no data."""
    try:
        result = subprocess.run(
            ["vnstat", "--dbdir", VNSTAT_DB_DIR, "-i", VNSTAT_IFACE, "--json", "m"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    try:
        data = json.loads(result.stdout)
        months = data["interfaces"][0]["traffic"]["months"]
    except (ValueError, TypeError, KeyError, IndexError):
        return None
    today = datetime.now()
    current = None
    first = None
    for entry in months:
        if not isinstance(entry, dict):
            continue
        if first is None:
            first = entry
        date = entry.get("date") or {}
        if date.get("year") == today.year and date.get("month") == today.month:
            current = entry
            break
        if entry.get("id") == 0 and current is None:
            current = entry
    entry = current or first
    if entry is None:
        return None
    try:
        return int(entry["rx"]) + int(entry["tx"])
    except (KeyError, TypeError, ValueError):
        return None


class CellularDataProvider:
    """Provides the cellular data usage text for the cluster HUD.

    The monthly total is refreshed in a background thread at most every
    60 seconds (vnstat is a subprocess call, never run on the render
    thread); the live ppp0 counters are cheap sysfs reads done on the
    calling thread at most every 2 seconds.
    """

    def __init__(
        self,
        monthly_refresh_s: float = MONTHLY_REFRESH_SECONDS,
        live_refresh_s: float = LIVE_REFRESH_SECONDS,
    ) -> None:
        self._monthly_refresh_s = max(1.0, float(monthly_refresh_s))
        self._live_refresh_s = max(0.1, float(live_refresh_s))
        self._lock = threading.Lock()
        self._monthly_bytes: int | None = None
        self._monthly_next = 0.0
        self._live_next = 0.0
        self._prev_total: int | None = None
        self._prev_time: float | None = None
        self._rate_bps: float | None = None
        self._session_bytes: int | None = None
        self._ppp0_up = True
        self._wake = threading.Event()
        self._closed = False
        self._thread: threading.Thread | None = None

    def text(self, now: float | None = None) -> str | None:
        if now is None:
            now = time.perf_counter()
        self._maybe_start_monthly_refresh(now)
        self._refresh_live(now)
        with self._lock:
            monthly = self._monthly_bytes
            session = self._session_bytes
            rate = self._rate_bps
            up = self._ppp0_up
        if not up:
            return None
        return format_cellular_data_text(monthly, session, rate)

    def close(self) -> None:
        with self._lock:
            self._closed = True
        self._wake.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _maybe_start_monthly_refresh(self, now: float) -> None:
        thread_to_start = None
        with self._lock:
            if self._closed or now < self._monthly_next:
                return
            self._monthly_next = now + self._monthly_refresh_s
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._monthly_loop,
                    name="cluster-cellular-data",
                    daemon=True,
                )
                thread_to_start = self._thread
        if thread_to_start is not None:
            thread_to_start.start()
        self._wake.set()

    def _monthly_loop(self) -> None:
        while True:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                if self._closed:
                    return
            try:
                monthly = _read_vnstat_monthly_bytes()
            except Exception:
                monthly = None
            with self._lock:
                if monthly is not None:
                    self._monthly_bytes = monthly

    def _refresh_live(self, now: float) -> None:
        if now < self._live_next:
            return
        self._live_next = now + self._live_refresh_s
        counters = _read_ppp0_counters()
        with self._lock:
            if counters is None:
                self._ppp0_up = False
                self._session_bytes = None
                self._rate_bps = None
                self._prev_total = None
                self._prev_time = None
                return
            self._ppp0_up = True
            total = counters[0] + counters[1]
            self._session_bytes = total
            if self._prev_total is not None and self._prev_time is not None:
                dt = now - self._prev_time
                if dt > 0 and total >= self._prev_total:
                    self._rate_bps = (total - self._prev_total) / dt
                else:
                    # Counters reset (interface restarted) — re-baseline.
                    self._rate_bps = None
            self._prev_total = total
            self._prev_time = now
