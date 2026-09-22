#!/usr/bin/env python3
"""JSONL 세션 로그를 에피소드/개입 단위 CSV로 요약한다."""

import argparse
import csv
import json
from pathlib import Path


def _read_sessions(source):
    files = [source] if source.is_file() else sorted(source.glob("*.jsonl"))
    for path in files:
        rows = []
        with path.open(encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(
                        f"{path}:{line_number}: JSONL 파싱 오류: {exc}") from exc
        yield path, rows


def _base_fields(path, rows):
    started = next((row for row in rows
                    if row.get("event") == "session_started"), {})
    metadata = started.get("metadata") or {}
    return {
        "session_id": started.get("session_id", path.stem),
        "participant_id": metadata.get("participant_id", ""),
        "condition": started.get("condition", ""),
        "move_enabled": metadata.get("move_enabled", ""),
        "posture_trigger_sustain_sec": metadata.get(
            "posture_trigger_sustain_sec", ""),
        "recovery_confirm_sec": metadata.get("recovery_confirm_sec", ""),
        "camera_source": metadata.get("camera_source", ""),
        "camera_width": metadata.get("camera_width", ""),
        "camera_height": metadata.get("camera_height", ""),
    }


def summarize(source):
    episodes = []
    interventions = []
    for path, rows in _read_sessions(source):
        base = _base_fields(path, rows)
        episode_map = {}
        intervention_map = {}

        for row in rows:
            event = row.get("event")
            episode_id = row.get("episode_id")
            intervention_id = row.get("intervention_id")
            if event == "posture_episode_started" and episode_id:
                entry = {
                    **base,
                    "episode_id": episode_id,
                    "episode_index": row.get("episode_index", ""),
                    "start_elapsed_sec": row.get("elapsed_sec", ""),
                    "posture_label": row.get("posture_label", ""),
                    "torso_proxy": row.get("torso_proxy", ""),
                    "neck_proxy": row.get("neck_proxy", ""),
                    "lateral_proxy": row.get("lateral_proxy", ""),
                    "confidence": row.get("confidence", ""),
                    "triggered": False,
                }
                episode_map[episode_id] = entry
                episodes.append(entry)
            elif event == "trigger_threshold_reached" and episode_id:
                entry = episode_map.get(episode_id)
                if entry is not None:
                    entry["triggered"] = True
                    entry["trigger_elapsed_sec"] = row.get("elapsed_sec", "")
                    entry["configured_delay_sec"] = row.get(
                        "configured_delay_sec", "")
                    entry["observed_bad_duration_sec"] = row.get(
                        "observed_bad_duration_sec", "")
            elif event == "posture_episode_ended" and episode_id:
                entry = episode_map.get(episode_id)
                if entry is not None:
                    entry["end_elapsed_sec"] = row.get("elapsed_sec", "")
                    entry["end_reason"] = row.get("end_reason", "")
                    entry["duration_sec"] = row.get("duration_sec", "")
                    entry["observed_duration_sec"] = row.get(
                        "observed_duration_sec", "")
                    entry["observation_complete"] = row.get(
                        "observation_complete", "")
            elif event == "intervention" and intervention_id:
                entry = {
                    **base,
                    "intervention_id": intervention_id,
                    "intervention_index": row.get("intervention_index", ""),
                    "episode_id": episode_id or "",
                    "accepted_elapsed_sec": row.get("elapsed_sec", ""),
                    "delivery_status": row.get("delivery_status", ""),
                    "accepted_outputs": ",".join(
                        row.get("accepted_outputs", [])),
                    "posture_label": row.get("posture", ""),
                    "behavior": row.get("behavior", ""),
                }
                intervention_map[intervention_id] = entry
                interventions.append(entry)
            elif intervention_id and intervention_id in intervention_map:
                entry = intervention_map[intervention_id]
                if event == "posture_recovery_candidate":
                    if not entry.get("response_onset_sec"):
                        entry["response_onset_sec"] = row.get(
                            "response_onset_sec", "")
                elif event == "posture_recovered":
                    entry["confirmed_response_time_sec"] = row.get(
                        "confirmed_response_time_sec", "")
                    entry["confirmation_observed_sec"] = row.get(
                        "confirmation_observed_sec", "")
                    entry["outcome"] = "recovered"
                elif event == "posture_maintenance":
                    entry["good_duration_sec"] = row.get(
                        "good_duration_sec", "")
                    entry["observation_complete"] = row.get(
                        "observation_complete", "")
                elif event == "posture_relapsed":
                    entry["relapsed_elapsed_sec"] = row.get("elapsed_sec", "")
                    entry["time_since_recovery_sec"] = row.get(
                        "time_since_recovery_sec", "")
                elif event == "intervention_ignored":
                    entry["outcome"] = "ignored"
                    entry["outcome_reason"] = row.get("reason", "")
                elif event == "intervention_outcome_incomplete":
                    entry["outcome"] = "incomplete"
                    entry["outcome_reason"] = row.get("reason", "")
                    entry["response_onset_sec"] = row.get(
                        "response_onset_sec", entry.get("response_onset_sec", ""))

        for entry in intervention_map.values():
            entry.setdefault("outcome", "unresolved")
    return episodes, interventions


def _write_csv(path, rows, columns):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("data/runs"),
                        help="JSONL 로그 파일 또는 로그 폴더")
    parser.add_argument("--output-dir", type=Path,
                        default=Path("data/summaries"),
                        help="episodes.csv와 interventions.csv 저장 폴더")
    args = parser.parse_args()
    if not args.input.exists():
        parser.error(f"입력 경로가 없습니다: {args.input}")

    episodes, interventions = summarize(args.input)
    _write_csv(
        args.output_dir / "episodes.csv", episodes,
        ["session_id", "participant_id", "condition", "move_enabled",
         "posture_trigger_sustain_sec", "camera_source", "camera_width",
         "camera_height", "episode_id", "episode_index", "start_elapsed_sec",
         "end_elapsed_sec", "posture_label", "torso_proxy", "neck_proxy",
         "lateral_proxy", "confidence", "triggered", "trigger_elapsed_sec",
         "configured_delay_sec", "observed_bad_duration_sec", "end_reason",
         "duration_sec", "observed_duration_sec", "observation_complete"],
    )
    _write_csv(
        args.output_dir / "interventions.csv", interventions,
        ["session_id", "participant_id", "condition", "move_enabled",
         "posture_trigger_sustain_sec", "recovery_confirm_sec", "camera_source",
         "camera_width", "camera_height", "intervention_id",
         "intervention_index", "episode_id", "accepted_elapsed_sec",
         "delivery_status", "accepted_outputs", "posture_label", "behavior",
         "response_onset_sec",
         "confirmed_response_time_sec", "confirmation_observed_sec",
         "good_duration_sec", "relapsed_elapsed_sec",
         "time_since_recovery_sec", "observation_complete", "outcome",
         "outcome_reason"],
    )
    print(f"에피소드 {len(episodes)}건 → {args.output_dir / 'episodes.csv'}")
    print(f"개입 {len(interventions)}건 → {args.output_dir / 'interventions.csv'}")


if __name__ == "__main__":
    main()
