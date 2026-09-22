"""개입 후 사용자 반응을 제어와 분리해 기록한다."""

from __future__ import annotations

import time
import uuid


class ResponseTracker:
    """개입 접수, 첫 반응, 회복 확정과 관측 공백을 기록한다."""

    def __init__(self, event_logger, response_timeout_sec=15.0,
                 recovery_confirm_sec=1.0):
        if response_timeout_sec <= 0:
            raise ValueError("response_timeout_sec 은 0 보다 커야 합니다")
        if recovery_confirm_sec <= 0:
            raise ValueError("recovery_confirm_sec 은 0 보다 커야 합니다")
        self.logger = event_logger
        self.response_timeout_sec = float(response_timeout_sec)
        self.recovery_confirm_sec = float(recovery_confirm_sec)
        self._active = None
        self._intervention_index = 0

    def intervention(self, now, posture_label, behavior, episode_id=None,
                     accepted_outputs=None):
        """출력 큐 접수 시점을 개입 기준으로 기록한다."""
        if self._active is not None:
            self._finish_incomplete(now, "superseded")

        self._intervention_index += 1
        intervention_id = f"I{self._intervention_index:04d}-{uuid.uuid4().hex[:6]}"
        self._active = {
            "intervention_id": intervention_id,
            "intervention_index": self._intervention_index,
            "episode_id": episode_id,
            "started_at": now,
            "posture_label": posture_label,
            "behavior": behavior,
            "first_candidate_at": None,
            "candidate_since": None,
            "candidate_observed_sec": 0.0,
            "candidate_index": 0,
            "recovered_at": None,
            "good_observed_sec": 0.0,
            "last_valid_at": None,
            "last_label": None,
            "observation_gap_at": None,
        }
        self.logger.record(
            "intervention", timestamp=now,
            intervention_id=intervention_id,
            intervention_index=self._intervention_index,
            episode_id=episode_id,
            posture=posture_label,
            behavior=behavior,
            delivery_status="accepted_by_output_queue",
            accepted_outputs=list(accepted_outputs or []),
        )
        return intervention_id

    def update(self, now, posture_label):
        """관측 상태를 기록한다. 이 메서드는 제어 명령을 만들지 않는다."""
        active = self._active
        if active is None:
            return

        if posture_label == "unknown":
            if active["observation_gap_at"] is None:
                active["observation_gap_at"] = now
                self.logger.record(
                    "intervention_observation_lost", timestamp=now,
                    **self._ids(active),
                )
            if active["candidate_since"] is not None:
                self._cancel_candidate(now, "observation_lost")
            active["last_valid_at"] = None
            active["last_label"] = "unknown"
            if (active["recovered_at"] is None
                    and now - active["started_at"]
                    >= self.response_timeout_sec):
                self._finish_incomplete(now, "timeout_during_observation_gap")
            return

        if active["observation_gap_at"] is not None:
            self.logger.record(
                "intervention_observation_resumed", timestamp=now,
                gap_duration_sec=round(
                    now - active["observation_gap_at"], 4),
                **self._ids(active),
            )
            active["observation_gap_at"] = None

        previous_at = active["last_valid_at"]
        previous_label = active["last_label"]
        if previous_at is not None and previous_label == "good":
            observed_delta = max(0.0, min(now - previous_at, 0.5))
            active["good_observed_sec"] += observed_delta

        if active["recovered_at"] is None:
            if (active["first_candidate_at"] is None
                    and now - active["started_at"]
                    >= self.response_timeout_sec):
                self._finish_ignored(now, "timeout")
                return
            self._update_recovery_candidate(now, posture_label, active)
            if (self._active is active
                    and active["recovered_at"] is None
                    and now - active["started_at"]
                    >= self.response_timeout_sec
                    and posture_label != "good"):
                self._finish_ignored(now, "timeout")
        elif posture_label != "good":
            self.logger.record(
                "posture_relapsed", timestamp=now,
                time_since_recovery_sec=round(
                    now - active["recovered_at"], 4),
                posture_label=posture_label,
                **self._ids(active),
            )
            self.logger.record(
                "posture_maintenance", timestamp=now,
                good_duration_sec=round(active["good_observed_sec"], 4),
                observation_complete=True,
                **self._ids(active),
            )
            self._active = None

        active["last_valid_at"] = now
        active["last_label"] = posture_label

    def _update_recovery_candidate(self, now, posture_label, active):
        if posture_label != "good":
            if active["candidate_since"] is not None:
                self._cancel_candidate(now, "posture_not_good")
            return

        if active["candidate_since"] is None:
            active["candidate_since"] = now
            active["candidate_observed_sec"] = 0.0
            active["candidate_index"] += 1
            if active["first_candidate_at"] is None:
                active["first_candidate_at"] = now
                response_onset = now - active["started_at"]
                self.logger.record(
                    "posture_recovery_candidate", timestamp=now,
                    candidate_index=active["candidate_index"],
                    response_onset_sec=round(response_onset, 4),
                    **self._ids(active),
                )
                self.logger.record(
                    "posture_corrected", timestamp=now,
                    response_time_sec=round(response_onset, 4),
                    response_measure="first_good_classification",
                    **self._ids(active),
                )
            else:
                self.logger.record(
                    "posture_recovery_candidate", timestamp=now,
                    candidate_index=active["candidate_index"],
                    response_onset_sec=round(
                        active["first_candidate_at"] - active["started_at"], 4),
                    **self._ids(active),
                )
            return

        previous_at = active["last_valid_at"]
        if previous_at is not None and active["last_label"] == "good":
            active["candidate_observed_sec"] += max(
                0.0, min(now - previous_at, 0.5))
        if active["candidate_observed_sec"] >= self.recovery_confirm_sec:
            active["recovered_at"] = now
            active["good_observed_sec"] = active["candidate_observed_sec"]
            self.logger.record(
                "posture_recovered", timestamp=now,
                response_onset_sec=round(
                    active["first_candidate_at"] - active["started_at"], 4),
                confirmed_response_time_sec=round(
                    now - active["started_at"], 4),
                confirmation_observed_sec=round(
                    active["candidate_observed_sec"], 4),
                **self._ids(active),
            )

    def _cancel_candidate(self, now, reason):
        active = self._active
        if active is None or active["candidate_since"] is None:
            return
        self.logger.record(
            "posture_recovery_candidate_cancelled", timestamp=now,
            candidate_index=active["candidate_index"], reason=reason,
            candidate_observed_sec=round(
                active["candidate_observed_sec"], 4),
            **self._ids(active),
        )
        active["candidate_since"] = None
        active["candidate_observed_sec"] = 0.0

    def close(self, now=None):
        """열린 응답을 세션 종료 시 불완전/관찰 종료로 기록한다."""
        if self._active is None:
            return
        now = time.monotonic() if now is None else now
        if self._active["recovered_at"] is None:
            self._finish_incomplete(now, "session_finished")
        else:
            self.logger.record(
                "posture_maintenance", timestamp=now,
                good_duration_sec=round(self._active["good_observed_sec"], 4),
                observation_complete=False,
                **self._ids(self._active),
            )
            self._active = None

    def _finish_ignored(self, now, reason):
        active = self._active
        self.logger.record(
            "intervention_ignored", timestamp=now,
            posture=active["posture_label"],
            behavior=active["behavior"], reason=reason,
            observation_complete=True,
            **self._ids(active),
        )
        self._active = None

    def _finish_incomplete(self, now, reason):
        active = self._active
        if active is None:
            return
        self.logger.record(
            "intervention_outcome_incomplete", timestamp=now,
            posture=active["posture_label"],
            behavior=active["behavior"], reason=reason,
            response_onset_sec=(round(active["first_candidate_at"]
                                    - active["started_at"], 4)
                                if active["first_candidate_at"] is not None
                                else None),
            **self._ids(active),
        )
        self._active = None

    @staticmethod
    def _ids(active):
        return {
            "intervention_id": active["intervention_id"],
            "intervention_index": active["intervention_index"],
            "episode_id": active["episode_id"],
        }
