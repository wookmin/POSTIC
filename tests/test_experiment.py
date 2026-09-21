"""실험 로그와 TTS 큐 테스트. 하드웨어·스피커 없이 돈다."""

import json

from src.audio.tts import SpeechQueue
from src.telemetry.event_log import EventLogger
from src.telemetry.response_tracker import ResponseTracker


class TestResponseTracker:
    def test_records_response_and_maintenance(self, tmp_path):
        logger = EventLogger(tmp_path, condition="mirror", session_id="responded")
        tracker = ResponseTracker(logger, response_timeout_sec=15.0)
        base = logger.started_at

        tracker.intervention(base + 1.0, "slouch", "mirror")
        tracker.update(base + 4.5, "good")
        tracker.update(base + 12.0, "slouch")
        logger.close(base + 13.0)

        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        events = [row["event"] for row in rows]
        assert events == [
            "session_started", "intervention", "posture_corrected",
            "posture_maintenance", "session_finished",
        ]
        corrected = rows[2]
        assert corrected["response_time_sec"] == 3.5
        assert rows[3]["good_duration_sec"] == 7.5

    def test_timeout_records_ignored_intervention(self, tmp_path):
        logger = EventLogger(tmp_path, condition="voice", session_id="ignored")
        tracker = ResponseTracker(logger, response_timeout_sec=5.0)
        base = logger.started_at

        tracker.intervention(base + 1.0, "forward_head", "voice")
        tracker.update(base + 6.1, "forward_head")
        logger.close(base + 7.0)

        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        assert rows[2]["event"] == "intervention_ignored"
        assert rows[2]["reason"] == "timeout"


class TestSpeechQueue:
    def test_disabled_queue_is_a_noop(self):
        speaker = SpeechQueue({"enabled": False})
        assert speaker.submit("안내") is False

    def test_duplicate_text_is_suppressed_during_cooldown(self):
        speaker = SpeechQueue({
            "enabled": True,
            "command": ["fake-tts"],
            "cooldown_sec": 8.0,
        })
        assert speaker.submit("안내", now=10.0) is True
        assert speaker.submit("안내", now=15.0) is False
        assert speaker.submit("안내", now=19.0) is True
