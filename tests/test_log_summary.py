import csv
import json
import subprocess
import sys
from pathlib import Path


def test_summary_script_exports_episode_and_intervention_rows(tmp_path):
    input_dir = tmp_path / "runs"
    output_dir = tmp_path / "summary"
    input_dir.mkdir()
    log_path = input_dir / "session.jsonl"
    events = [
        {
            "event": "session_started", "session_id": "session",
            "condition": "posture_trigger", "metadata": {
                "participant_id": "P01", "move_enabled": True,
                "posture_trigger_sustain_sec": 3.0,
                "recovery_confirm_sec": 1.0,
            },
        },
        {
            "event": "posture_episode_started", "session_id": "session",
            "elapsed_sec": 1.0, "episode_id": "E0001", "episode_index": 1,
            "posture_label": "slouch", "torso_proxy": 22.0,
        },
        {
            "event": "trigger_threshold_reached", "session_id": "session",
            "elapsed_sec": 4.1, "episode_id": "E0001",
            "configured_delay_sec": 3.0,
            "observed_bad_duration_sec": 3.1,
        },
        {
            "event": "intervention", "session_id": "session",
            "elapsed_sec": 4.2, "intervention_id": "I0001-a1b2c3",
            "intervention_index": 1, "episode_id": "E0001",
            "delivery_status": "accepted_by_output_queue",
            "accepted_outputs": ["motor_control_queue"],
            "posture": "slouch", "behavior": "mimic_slouch",
        },
        {
            "event": "posture_recovery_candidate", "session_id": "session",
            "elapsed_sec": 6.0, "intervention_id": "I0001-a1b2c3",
            "response_onset_sec": 1.8,
        },
        {
            "event": "posture_recovered", "session_id": "session",
            "elapsed_sec": 7.0, "intervention_id": "I0001-a1b2c3",
            "confirmed_response_time_sec": 2.8,
            "confirmation_observed_sec": 1.0,
        },
        {
            "event": "posture_episode_ended", "session_id": "session",
            "elapsed_sec": 7.0, "episode_id": "E0001",
            "end_reason": "good_classification", "duration_sec": 6.0,
            "observed_duration_sec": 5.8, "observation_complete": True,
        },
    ]
    log_path.write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )

    script = Path(__file__).resolve().parents[1] / "scripts" / \
        "summarize_posture_logs.py"
    subprocess.run(
        [sys.executable, str(script), "--input", str(input_dir),
         "--output-dir", str(output_dir)],
        check=True, capture_output=True, text=True,
    )

    with (output_dir / "episodes.csv").open(encoding="utf-8-sig") as stream:
        episode = next(csv.DictReader(stream))
    with (output_dir / "interventions.csv").open(
            encoding="utf-8-sig") as stream:
        intervention = next(csv.DictReader(stream))

    assert episode["participant_id"] == "P01"
    assert episode["triggered"] == "True"
    assert episode["configured_delay_sec"] == "3.0"
    assert intervention["intervention_id"] == "I0001-a1b2c3"
    assert intervention["delivery_status"] == "accepted_by_output_queue"
    assert intervention["response_onset_sec"] == "1.8"
    assert intervention["confirmed_response_time_sec"] == "2.8"
    assert intervention["outcome"] == "recovered"
