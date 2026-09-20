"""비동기 로컬 TTS 큐.

실시간 제어 루프를 막지 않도록 음성은 별도 스레드에서 재생한다. 실제 합성
엔진은 설정의 command로 주입하며, command가 없거나 비활성화되어도 로봇
동작에는 영향을 주지 않는다.
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from queue import Empty, Queue


class SpeechQueue:
    """중복 억제와 timeout을 포함한 비동기 음성 출력."""

    def __init__(self, config=None, runner=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.cooldown_sec = float(config.get("cooldown_sec", 8.0))
        self.timeout_sec = float(config.get("timeout_sec", 10.0))
        command = config.get("command")
        self.command = list(command) if command else self._discover_command()
        self._runner = runner or self._run_process
        self._queue = Queue()
        self._stop = object()
        self._thread = None
        self._last_text = None
        self._last_at = -float("inf")

    @staticmethod
    def _discover_command():
        if shutil.which("espeak-ng"):
            return ["espeak-ng", "-v", "ko"]
        if shutil.which("say"):
            return ["say"]
        return []

    def start(self):
        if not self.enabled or not self.command or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._worker,
                                        name="tts", daemon=True)
        self._thread.start()

    def submit(self, text, now=None):
        text = (text or "").strip()
        if not self.enabled or not self.command or not text:
            return False
        now = time.monotonic() if now is None else now
        if text == self._last_text and now - self._last_at < self.cooldown_sec:
            return False
        self._last_text = text
        self._last_at = now
        self._queue.put(text)
        return True

    def stop(self):
        if self._thread is None:
            return
        self._queue.put(self._stop)
        self._thread.join(timeout=self.timeout_sec + 1.0)
        self._thread = None

    def _worker(self):
        while True:
            try:
                text = self._queue.get(timeout=0.2)
            except Empty:
                continue
            if text is self._stop:
                return
            try:
                self._runner(self.command, text, self.timeout_sec)
            except Exception:
                # 음성 실패가 자세 제어나 안전 정리를 막으면 안 된다.
                continue

    @staticmethod
    def _run_process(command, text, timeout):
        subprocess.run([*command, text],
                       check=False,
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL,
                       timeout=timeout)
