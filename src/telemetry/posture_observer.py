"""제어 판단과 분리된 자세 에피소드 이벤트 기록기."""

from __future__ import annotations

class PostureEpisodeObserver:
    """관측된 분류 상태 변화를 세션 로그로만 기록한다."""

    def __init__(self, event_logger, sustain_seconds=3.0):
        self.logger = event_logger
        self.sustain_seconds = float(sustain_seconds)
        self.episode_index = 0
        self.active_episode = None
        self.observation_lost_at = None
        self.max_observed_frame_gap_sec = 0.5

    @property
    def episode_id(self):
        return self.active_episode["episode_id"] if self.active_episode else None

    def observe(self, now, posture, angles):
        label = posture.label
        if label == "unknown":
            if self.observation_lost_at is None:
                self.observation_lost_at = now
                self.logger.record(
                    "observation_lost", timestamp=now,
                    episode_id=self.episode_id, reason="posture_unknown",
                )
            self._end_episode(now, "observation_lost", complete=False)
            return

        if self.observation_lost_at is not None:
            self.logger.record(
                "observation_resumed", timestamp=now,
                gap_duration_sec=round(now - self.observation_lost_at, 4),
            )
            self.observation_lost_at = None

        episode = self.active_episode
        if episode is not None and episode["last_observed_at"] is not None:
            delta = now - episode["last_observed_at"]
            if 0.0 <= delta <= self.max_observed_frame_gap_sec:
                episode["observed_duration_sec"] += delta
        if episode is not None:
            episode["last_observed_at"] = now

        if posture.is_bad:
            if self.active_episode is None:
                self.episode_index += 1
                episode_id = f"E{self.episode_index:04d}"
                self.active_episode = {
                    "episode_id": episode_id,
                    "started_at": now,
                    "posture_label": label,
                    "triggered": False,
                    "last_observed_at": now,
                    "observed_duration_sec": 0.0,
                }
                self.logger.record(
                    "posture_episode_started", timestamp=now,
                    episode_id=episode_id,
                    episode_index=self.episode_index,
                    posture_label=label,
                    torso_proxy=round(angles.torso_pitch_deg, 3),
                    neck_proxy=round(angles.neck_pitch_deg, 3),
                    lateral_proxy=round(angles.lateral_tilt_deg, 3),
                    confidence=round(angles.confidence, 4),
                )
            elif label != self.active_episode["posture_label"]:
                previous = self.active_episode["posture_label"]
                self.active_episode["posture_label"] = label
                self.logger.record(
                    "posture_label_changed", timestamp=now,
                    episode_id=self.episode_id,
                    previous_label=previous, posture_label=label,
                )
            return

        self._end_episode(now, "good_classification", complete=True)

    def trigger(self, timestamp, posture_label):
        """기존 판단 스레드가 만든 트리거를 관찰 로그에 연결한다."""
        episode = self.active_episode
        episode_id = None
        observed_duration = None
        if episode is not None:
            episode_id = episode["episode_id"]
            observed_duration = max(0.0, timestamp - episode["started_at"])
            episode["triggered"] = True
        self.logger.record(
            "trigger_threshold_reached", timestamp=timestamp,
            episode_id=episode_id, posture_label=posture_label,
            configured_delay_sec=self.sustain_seconds,
            observed_bad_duration_sec=(round(observed_duration, 4)
                                       if observed_duration is not None else None),
        )
        return episode_id

    def intervention_skipped(self, timestamp, reason, **fields):
        self.logger.record(
            "intervention_skipped", timestamp=timestamp,
            episode_id=self.episode_id, reason=reason, **fields,
        )

    def close(self, now):
        self._end_episode(now, "session_finished", complete=False)

    def _end_episode(self, now, reason, complete):
        episode = self.active_episode
        if episode is None:
            return
        self.logger.record(
            "posture_episode_ended", timestamp=now,
            episode_id=episode["episode_id"],
            episode_index=self.episode_index,
            end_reason=reason,
            duration_sec=round(max(0.0, now - episode["started_at"]), 4),
            observed_duration_sec=round(episode["observed_duration_sec"], 4),
            triggered=episode["triggered"],
            observation_complete=complete,
        )
        self.active_episode = None
