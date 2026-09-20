"""실험 이벤트를 한 줄 JSON으로 기록한다.

프레임 스트림이나 원본 영상은 저장하지 않고, 개입과 실제 반응처럼 분석에
필요한 사건만 남긴다. 로그 오류가 로봇 동작을 중단시키지는 않는다.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path


class EventLogger:
    """세션 단위 JSONL 이벤트 로거."""

    def __init__(self, log_dir="data/runs", condition="unknown", session_id=None):
        self.session_id = session_id or uuid.uuid4().hex[:12]
        self.condition = condition
        self.started_at = time.monotonic()
        self.path = Path(log_dir) / f"{self.session_id}.jsonl"
        self._lock = threading.Lock()
        self._disabled = False
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.record("session_started", condition=condition)
        except OSError:
            self._disabled = True

    def record(self, event, timestamp=None, **fields):
        """이벤트를 기록한다. 디스크 오류는 기능을 방해하지 않는다."""
        if self._disabled:
            return False
        now = time.monotonic() if timestamp is None else timestamp
        row = {
            "event": event,
            "session_id": self.session_id,
            "condition": self.condition,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "elapsed_sec": round(now - self.started_at, 4),
            **fields,
        }
        try:
            with self._lock:
                with self.path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        except (OSError, TypeError, ValueError):
            self._disabled = True
            return False
        return True

    def close(self, timestamp=None):
        self.record("session_finished", timestamp=timestamp)
