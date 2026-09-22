"""실험 로그와 TTS 큐 테스트. 하드웨어·스피커 없이 돈다."""

import json

from src.audio.tts import SpeechQueue
from src.telemetry.event_log import EventLogger
from src.telemetry.posture_observer import PostureEpisodeObserver
from src.telemetry.response_tracker import ResponseTracker
from src.perception.posture_features import PostureAngles
from src.posture.classifier import classify


class TestResponseTracker:
    def test_records_response_and_maintenance(self, tmp_path):
        logger = EventLogger(tmp_path, condition="mirror", session_id="responded")
        tracker = ResponseTracker(logger, response_timeout_sec=15.0)
        base = logger.started_at

        tracker.intervention(
            base + 1.0, "slouch", "mirror",
            accepted_outputs=["motor_control_queue"],
        )
        tracker.update(base + 4.5, "good")
        tracker.update(base + 5.0, "good")
        tracker.update(base + 5.5, "good")
        for step in range(12, 24):
            tracker.update(base + step * 0.5, "good")
        tracker.update(base + 12.0, "slouch")
        logger.close(base + 13.0)

        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        events = [row["event"] for row in rows]
        assert events == [
            "session_started", "intervention", "posture_recovery_candidate",
            "posture_corrected", "posture_recovered", "posture_relapsed",
            "posture_maintenance", "session_finished",
        ]
        candidate = rows[2]
        recovered = rows[4]
        maintenance = rows[6]
        assert candidate["response_onset_sec"] == 3.5
        assert recovered["confirmed_response_time_sec"] == 4.5
        assert maintenance["good_duration_sec"] == 7.5
        assert rows[1]["delivery_status"] == "accepted_by_output_queue"
        assert rows[1]["accepted_outputs"] == ["motor_control_queue"]
        assert rows[1]["intervention_id"].startswith("I0001-")

    def test_records_accepted_output_channels(self, tmp_path):
        logger = EventLogger(tmp_path, condition="posture_trigger",
                             session_id="output-channels")
        tracker = ResponseTracker(logger)
        tracker.intervention(
            logger.started_at + 1.0, "slouch", "mimic_slouch",
            accepted_outputs=["motor_control_queue", "tts_queue"],
        )
        tracker.close(logger.started_at + 2.0)
        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        intervention = next(row for row in rows
                            if row["event"] == "intervention")
        assert intervention["delivery_status"] == "accepted_by_output_queue"
        assert intervention["accepted_outputs"] == [
            "motor_control_queue", "tts_queue"]

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

    def test_unknown_interval_is_excluded_from_good_maintenance(self, tmp_path):
        logger = EventLogger(tmp_path, condition="posture_trigger",
                             session_id="observation-gap")
        tracker = ResponseTracker(logger, recovery_confirm_sec=1.0)
        base = logger.started_at
        tracker.intervention(base + 1.0, "slouch", "mimic_slouch",
                             episode_id="E0001")
        tracker.update(base + 2.0, "good")
        tracker.update(base + 2.5, "good")
        tracker.update(base + 3.0, "good")
        tracker.update(base + 4.0, "unknown")
        tracker.update(base + 10.0, "good")
        tracker.update(base + 12.0, "slouch")
        logger.close(base + 13.0)

        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        resumed = next(row for row in rows
                       if row["event"] == "intervention_observation_resumed")
        maintenance = next(row for row in rows
                           if row["event"] == "posture_maintenance")
        assert resumed["gap_duration_sec"] == 6.0
        assert maintenance["good_duration_sec"] == 1.5
        assert maintenance["episode_id"] == "E0001"


class TestPostureEpisodeObserver:
    def test_logs_episode_trigger_and_end_without_affecting_classification(
            self, tmp_path):
        logger = EventLogger(tmp_path, condition="posture_trigger",
                             session_id="episode")
        observer = PostureEpisodeObserver(logger, sustain_seconds=3.0)
        bad_angles = PostureAngles(1.0, 20.0, 2.0, 0.9)
        good_angles = PostureAngles(2.0, 0.0, 0.0, 0.9)

        observer.observe(1.0, classify(bad_angles), bad_angles)
        observer.trigger(4.2, "slouch")
        observer.observe(5.0, classify(good_angles), good_angles)
        observer.close(6.0)
        logger.close(6.0)

        rows = [json.loads(line) for line in logger.path.read_text().splitlines()]
        started = next(row for row in rows
                       if row["event"] == "posture_episode_started")
        triggered = next(row for row in rows
                         if row["event"] == "trigger_threshold_reached")
        ended = next(row for row in rows
                     if row["event"] == "posture_episode_ended")
        assert started["episode_id"] == "E0001"
        assert triggered["configured_delay_sec"] == 3.0
        assert triggered["observed_bad_duration_sec"] == 3.2
        assert ended["end_reason"] == "good_classification"
        assert ended["triggered"] is True


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
