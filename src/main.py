#!/usr/bin/env python3
"""카메라로 본 나쁜 자세에만 미리 정한 동작으로 반응한다.

정상 자세에서는 로봇을 중립에 두고 움직이지 않는다. 나쁜 자세가 정책에
정해진 시간 이상 지속될 때만 미리 정의된 고정 포즈를 실행하고, 사용자가
정상 자세로 돌아오면 중립으로 복귀한다.
기본은 모터를 건드리지 않는다. 실제로 움직이려면 --move 를 붙인다.

    ~/dynamixel-venv/bin/python -m src.main              # 프리뷰만
    ~/dynamixel-venv/bin/python -m src.main --move       # 로봇 구동
"""

import argparse
import sys
import threading
import time
from contextlib import ExitStack
from pathlib import Path

import cv2
import yaml

from src.camera.camera_stream import CameraError, CameraStream  # noqa: E402
from src.perception.pose_estimator import (  # noqa: E402
    PoseEstimator, draw_angle_bar, draw_overlay, draw_skeleton,
)
from src.perception.posture_features import (  # noqa: E402
    MedianFilter, PostureAngles, clamp_angles, extract_angles, smooth,
)
from src.posture.pose_buffer import PoseBuffer  # noqa: E402
from src.posture.classifier import classify  # noqa: E402
from src.robot.dynamixel_driver import (  # noqa: E402
    BusError, PROJECT_ROOT, describe_hardware_error, install_signal_guards,
    load_joints, open_bus, ping_all, read_hardware_error,
)
from src.robot.joint_mapper import (  # noqa: E402
    JointMapper, MappingError, TICKS_PER_DEG,
)
from src.robot.joint_writer import JointWriter, WriterError  # noqa: E402
from src.behavior.behavior_manager import BehaviorManager
from src.behavior.executor import BehaviorExecutor  # noqa: E402
from src.safety.supervisor import (  # noqa: E402
    STATE_IDLE, STATE_SAFE_STOP, STATE_TRACKING, IdlePolicy,
)
from src.safety.gate import SafetyGate  # noqa: E402
from src.safety.health import HealthMonitor, SafeStopRequested  # noqa: E402
from src.telemetry.event_log import EventLogger  # noqa: E402
from src.telemetry.response_tracker import ResponseTracker  # noqa: E402
from src.telemetry.posture_observer import PostureEpisodeObserver  # noqa: E402

POSTURE_CONFIG = PROJECT_ROOT / "config" / "posture.yaml"


def load_posture_config(path=POSTURE_CONFIG):
    if not Path(path).exists():
        raise FileNotFoundError(f"설정 파일이 없습니다: {path}")
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


class ControlLoop(threading.Thread):
    """고정 주기로 중립 또는 고정 개입 포즈를 모터에 쓰는 스레드."""

    def __init__(self, buffer, mapper, writer, config, safety_gate=None,
                 condition="posture_trigger"):
        super().__init__(name="control", daemon=True)
        echo = config["echo"]
        motion = config["motion"]
        self.buffer = buffer
        self.mapper = mapper
        self.condition = condition
        self.motion_enabled = True
        self.writer = writer
        self.period = 1.0 / echo["control_hz"]
        self.safety_gate = safety_gate or SafetyGate(mapper, config)
        distance_fn = getattr(self.mapper, "position_delta", None)
        if distance_fn is None:
            distance_fn = lambda name, current, target: target - current
        self.policy = IdlePolicy(
            motion["return_to_neutral_sec"], motion["idle_release_sec"],
            grace_seconds=motion.get("person_lost_grace_sec", 0.5),
            distance_fn=distance_fn)
        self.neutral = mapper.neutral_targets()
        self._rest_targets = dict(self.neutral)
        self.stop_event = threading.Event()
        self.error = None

        self.state = "starting"
        self.commanded = dict(self.neutral)
        self.last_targets = dict(self.neutral)
        self.heartbeat_at = time.monotonic()
        self.safe_stop_reason = None
        self._behavior_lock = threading.Lock()
        self._behavior_action = None
        self._behavior_started_at = None
        self._behavior_returning = False
        self._behavior_return_started_at = None
        self._behavior_release_sec = motion["idle_release_sec"]
        self._behavior_released = False
        camera_gimbal = config.get("camera_gimbal") or {}
        self._camera_gimbal_enabled = bool(camera_gimbal.get("enabled", False))
        self._camera_gimbal_joint = camera_gimbal.get(
            "joint", getattr(mapper, "neck_joint", "neck_pitch"))

    def _targets_for_pose(self, pose):
        """고정 포즈를 만들고 탑재 카메라 관절은 역회전으로 보정한다."""
        targets = dict(self.mapper.to_targets(pose))
        if not self._camera_gimbal_enabled:
            return targets

        joint = self._camera_gimbal_joint
        joints = getattr(self.mapper, "joints", {})
        spec = joints.get(joint)
        if spec is None or joint not in targets:
            return targets
        reference = self._rest_targets.get(joint, self.neutral.get(joint))
        if reference is None:
            return targets

        torso_delta_deg = 0.0
        for name, body_spec in joints.items():
            if name == joint or name not in targets:
                continue
            previous = self._rest_targets.get(name)
            direction = float(body_spec.get("direction", 1))
            if previous is None or direction == 0:
                continue
            torso_delta_deg += (
                (targets[name] - previous) / (TICKS_PER_DEG * direction))

        neck_direction = float(spec.get("direction", 1))
        raw = reference - neck_direction * torso_delta_deg * TICKS_PER_DEG
        targets[joint] = int(round(max(
            spec["min_position"], min(spec["max_position"], raw))))
        return targets

    def submit_behavior(self, action):
        """고정된 행동을 다음 제어 tick부터 재생한다."""
        if action is None:
            return False
        with self._behavior_lock:
            self._behavior_action = action
            self._behavior_started_at = time.monotonic()
            self._behavior_returning = False
            self._behavior_return_started_at = None
            self._behavior_released = False
        return True

    def safe_stop(self, reason):
        """추적을 중지하고 finally의 중립 복귀 경로로 보낸다."""
        self.safe_stop_reason = str(reason)
        self.state = STATE_SAFE_STOP
        with self._behavior_lock:
            self._behavior_action = None
            self._behavior_started_at = None
            self._behavior_returning = False
            self._behavior_return_started_at = None
            self._behavior_released = False
        self.stop_event.set()

    def _behavior_pose(self, now):
        with self._behavior_lock:
            action = self._behavior_action
            started = self._behavior_started_at
            if action is None or started is None:
                return None
            # 과장 포즈는 사용자가 정상 자세로 돌아올 때까지 유지한다.
            if (not getattr(action, "hold_until_good", False)
                    and now - started >= action.duration_sec):
                self._behavior_action = None
                self._behavior_started_at = None
                self._behavior_returning = True
                self._behavior_return_started_at = now
                return None
            return action.pose

    def _cancel_behavior(self, now):
        """사람이 사라지거나 자세가 바뀌면 고정 포즈를 중단하고 복귀한다."""
        with self._behavior_lock:
            if self._behavior_action is not None:
                self._behavior_action = None
                self._behavior_started_at = None
                self._behavior_returning = True
                self._behavior_return_started_at = now

    def _at_neutral(self):
        distance_fn = getattr(self.mapper, "position_delta", None)
        if distance_fn is None:
            distance_fn = lambda name, current, target: target - current
        return all(
            abs(distance_fn(name, self.commanded.get(name, value), value)) <= 15
            for name, value in self.neutral.items())

    def _posture_trigger_output(self, now, person_visible, state,
                                posture_label="unknown"):
        """반응 모드의 목표와 토크 상태를 계산한다.

        대기 중에는 시작 시점의 기준 자세를 토크로 유지하고, 이벤트가 있을
        때만 고정 포즈로 이동한다. 이벤트가 끝나면 중립으로 복귀한 뒤 토크를
        푼다.
        """
        behavior_pose = self._behavior_pose(now)
        if behavior_pose is not None:
            if (person_visible and state == STATE_TRACKING
                    and posture_label != "good"):
                return self._targets_for_pose(behavior_pose), True
            # 사람 이탈·관측 불가·정상 자세 복귀 시 과장 포즈를 끝내고
            # 중립으로 천천히 돌아간다.
            self._cancel_behavior(now)

        with self._behavior_lock:
            returning = self._behavior_returning
            started = self._behavior_return_started_at

        if returning:
            if self._at_neutral():
                if (started is None
                        or now - started >= self._behavior_release_sec):
                    with self._behavior_lock:
                        self._behavior_returning = False
                        self._behavior_return_started_at = None
                        self._behavior_released = True
                    return dict(self.commanded), False
            return dict(self.neutral), True

        with self._behavior_lock:
            released = self._behavior_released
        if released:
            # 한 번 중립 복귀를 끝낸 뒤에는 다음 tick에서 시작 기준 자세로
            # 되돌아가지 않고, 중립을 유지한 채 토크를 해제한다.
            return dict(self.neutral), False

        if not person_visible:
            # 사람이 없으면 기존 IdlePolicy가 중립 복귀와 토크 해제를
            # 담당한다. 사람이 보이는 동안에는 현재 기준 자세를 유지한다.
            return dict(self.neutral), state != STATE_IDLE

        # 평상시에는 사용자의 자세를 따라가지 않고 시작 시점의 기준 자세를
        # 유지한다. 토크를 풀면 세로 컬럼이 자중으로 무너질 수 있다.
        return dict(self._rest_targets), True

    def run(self):
        try:
            self._loop()
        except Exception as exc:                      # 스레드 밖으로 전달
            self.error = exc
            self.safe_stop(f"control_error: {exc}")

    def _loop(self):
        if self.writer is not None and self.motion_enabled:
            self.writer.prepare()
            self.commanded = self.writer.read_positions()
            self._rest_targets = dict(self.commanded)

        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            self.heartbeat_at = now
            # 반응 모드는 지연된 샘플이 아니라 최신 관측값만 사용한다.
            played = self.buffer.latest()
            person_visible = played is not None and played.valid
            posture_label = (classify(played).label
                             if person_visible else "unknown")

            state, torque = self.policy.update(now, person_visible,
                                               self.commanded, self.neutral)
            self.state = state

            desired, torque_enabled = self._posture_trigger_output(
                now, person_visible, state, posture_label)

            # 일반 추적, 행동 재생, 중립 복귀 모두 같은 관문을 통과한다.
            limited = self.safety_gate.limit_targets(self.commanded, desired)
            self.commanded = limited
            self.last_targets = limited

            if self.writer is not None and self.motion_enabled:
                if not torque_enabled:
                    self.writer.set_torque(False)
                else:
                    self.writer.set_torque(True)
                    self.writer.write_targets(limited)

            next_tick += self.period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()


def build_writer(stack, mapper, config):
    """모터 버스를 열고 JointWriter 를 만든다. 실패하면 예외."""
    packet, port = stack
    present = ping_all(packet, port)
    joint_ids = mapper.joint_ids()
    missing = [i for i in joint_ids.values() if i not in present]
    if missing:
        raise BusError(f"ID {missing} 가 버스에 없습니다. 응답: {sorted(present)}")

    for name, motor_id in joint_ids.items():
        fault = read_hardware_error(packet, port, motor_id)
        if fault:
            raise BusError(
                f"{name} (ID {motor_id}) 하드웨어 에러 0x{fault:02X} "
                f"({describe_hardware_error(fault)}). "
                "scripts/torque_off.py --clear-errors 로 해제하세요.")

    motion = config["motion"]
    return JointWriter(packet, port, joint_ids,
                       profile_velocity=motion["profile_velocity"],
                       profile_acceleration=motion["profile_acceleration"])


def check_pose_within_limits(writer, mapper):
    """접힌 자세에서 토크를 걸면 아래 단이 과부하로 죽는다."""
    joints = mapper.joints
    positions = writer.read_positions()
    folded = []
    for name, spec in joints.items():
        position = positions.get(name)
        if not mapper.within_limits(name, position):
            folded.append(f"  {name} 현재 {position}, 운용 범위 "
                          f"zero 기준 상대 범위 확인 필요")
    if folded:
        raise BusError("현재 자세가 운용 범위를 벗어나 있습니다.\n"
                       + "\n".join(folded)
                       + "\n손으로 컬럼을 세운 뒤 다시 실행하세요.")


def print_status_line(now, status_mark, control, measured, played, fps):
    """헤드리스 모드에서 0.5초마다 상태를 한 줄로 찍는다."""
    if now - status_mark < 0.5:
        return status_mark
    measured_state = classify(measured) if measured else None
    measured_text = (
        "torso {:+5.1f} neck {:+5.1f} shoulder {:+5.1f} {}".format(
            measured.torso_pitch_deg, measured.neck_pitch_deg,
            measured.lateral_tilt_deg, measured_state.label)
        if measured else "사람 없음            ")
    played_text = (
        "torso {:+5.1f} neck {:+5.1f} shoulder {:+5.1f}".format(
            played.torso_pitch_deg, played.neck_pitch_deg,
            played.lateral_tilt_deg)
        if played and played.valid else "대기                 ")
    targets = control.last_targets
    print(f"[{control.state:<9}] fps {fps:4.1f} | "
          f"측정 {measured_text} | 관측 {played_text} | "
          f"목표 {targets}", flush=True)
    return now


def draw_preview(frame, control, measured, played, angles_config,
                 landmarks, writer, fps):
    """프리뷰 창에 스켈레톤과 오버레이를 그린다."""
    draw_skeleton(frame, landmarks)
    lines = [
        f"state {control.state}   fps {fps:4.1f}",
        ("측정  torso {:+6.1f}  neck {:+6.1f}".format(
            measured.torso_pitch_deg, measured.neck_pitch_deg)
         if measured else "측정  사람을 찾는 중"),
        ("어깨  lateral {:+6.1f}  상태 {}".format(
            measured.lateral_tilt_deg, classify(measured).label)
         if measured else "어깨  측정 대기"),
        (f"관측  torso {played.torso_pitch_deg:+6.1f}  "
         f"neck {played.neck_pitch_deg:+6.1f}"
         if played and played.valid else "관측  대기"),
        "모터  " + ("구동" if writer else "dry-run"),
    ]
    draw_overlay(frame, lines)
    if measured:
        draw_angle_bar(frame, "torso", measured.torso_pitch_deg,
                       angles_config["max_torso_pitch_deg"], 0)
        draw_angle_bar(frame, "neck", measured.neck_pitch_deg,
                       angles_config["max_neck_pitch_deg"], 1)


def run(args):
    install_signal_guards()

    config = load_posture_config()
    if args.no_camera_gimbal:
        config["camera_gimbal"] = {
            **(config.get("camera_gimbal") or {}),
            "enabled": False,
        }
    perception = config["perception"]
    angles_config = config["angles"]

    joints = load_joints()
    if not joints:
        sys.exit("config/joints.yaml 에 joints 가 없습니다.")
    try:
        mapper = JointMapper(joints, config["distribution"])
    except MappingError as exc:
        sys.exit(f"관절 매핑 설정 오류: {exc}")

    buffer = PoseBuffer()
    median = MedianFilter(angles_config["median_window"])
    experiment = config.get("experiment") or {}
    condition = args.condition or experiment.get("condition", "posture_trigger")
    writer = None
    control = None
    previous = None
    frames = 0
    fps = 0.0
    fps_mark = time.monotonic()
    status_mark = 0.0

    behavior = None
    safety_gate = SafetyGate(mapper, config)
    health = HealthMonitor(config)
    intervention_config = config.get("intervention") or {}
    max_event_age_sec = float(
        intervention_config.get("max_event_age_sec", 0.75))
    correction_config = config.get("correction") or {}
    telemetry_config = config.get("telemetry") or {}
    source = args.camera or perception.get("camera",
                                           perception.get("camera_index", 0))
    try:
        executor = BehaviorExecutor(config, safety_gate, condition)
    except ValueError as exc:
        sys.exit(str(exc))

    event_logger = None
    response_tracker = None
    posture_observer = None
    log_dir = Path(experiment.get("log_dir", "data/runs"))
    if not log_dir.is_absolute():
        log_dir = PROJECT_ROOT / log_dir
    recovery_confirm_sec = float(
        telemetry_config.get("recovery_confirm_sec", 1.0))
    session_metadata = {
        "participant_id": args.participant_id or None,
        "move_enabled": bool(args.move),
        "posture_trigger_sustain_sec": float(
            correction_config.get("sustain_seconds", 3.0)),
        "response_timeout_sec": float(
            experiment.get("response_timeout_sec", 15.0)),
        "recovery_confirm_sec": recovery_confirm_sec,
        "camera_source": str(source),
        "camera_width": int(perception["width"]),
        "camera_height": int(perception["height"]),
    }
    event_logger = EventLogger(log_dir, condition=condition,
                               metadata=session_metadata)
    response_tracker = ResponseTracker(
        event_logger,
        response_timeout_sec=float(
            experiment.get("response_timeout_sec", 15.0)),
        recovery_confirm_sec=recovery_confirm_sec,
    )
    posture_observer = PostureEpisodeObserver(
        event_logger,
        sustain_seconds=correction_config.get("sustain_seconds", 3.0),
    )
    stack = ExitStack()
    try:
        if args.move:
            packet, port = stack.enter_context(open_bus())
            writer = build_writer((packet, port), mapper, config)
            check_pose_within_limits(writer, mapper)

        control = ControlLoop(buffer, mapper, writer, config, safety_gate,
                              condition=condition)
        control.start()

        # 교정 판단 스레드 (--no-correction 이면 비활성)
        behavior = None
        if not args.no_correction:
            behavior = BehaviorManager(config, condition=condition)
            behavior.start()
            print(f"자세 반응 모드 활성. 조건={condition}, "
                  "나쁜 자세가 지속될 때만 고정 모션을 실행합니다 "
                  "(--no-correction 으로 끌 수 있음).")

        with CameraStream(source, perception["width"],
                          perception["height"]) as camera, \
                PoseEstimator(args.model
                              or perception.get("model")) as estimator:
            print("자세 반응 시작. 프리뷰 창에서 q 또는 Esc 로 종료합니다.")
            print("사용자별 캘리브레이션 없이 실행합니다. "
                  "카메라 위치를 고정하고 오검출을 확인하세요.")
            if writer is None:
                print("dry-run 입니다. 모터는 움직이지 않습니다 (--move 로 구동).")

            start = time.monotonic()
            health.start(start)
            while not control.stop_event.is_set():
                frame = camera.read()
                now = time.monotonic()
                if frame is None:
                    reason = health.check(now, control.heartbeat_at)
                    if reason:
                        control.safe_stop(reason)
                        raise SafeStopRequested(reason)
                    continue
                health.record_frame(now)
                reason = health.check(now, control.heartbeat_at)
                if reason:
                    control.safe_stop(reason)
                    raise SafeStopRequested(reason)
                landmarks, world = estimator.detect(frame,
                                                    (now - start) * 1000.0)

                measured = None
                if landmarks and world:
                    measured = extract_angles(
                        world, landmarks, now,
                        perception["min_visibility"],
                        perception["invert_torso"],
                        perception["invert_neck"])

                if measured is not None:
                    # 순서가 중요하다. 중앙값으로 튀는 값을 먼저 버리고,
                    # 운용 범위 안으로 자르고 부드럽게 한다.
                    measured = median.apply(measured)
                    measured = clamp_angles(measured,
                                            angles_config["max_torso_pitch_deg"],
                                            angles_config["max_neck_pitch_deg"])
                    measured = smooth(previous, measured,
                                      angles_config["smoothing_alpha"],
                                      angles_config.get("neck_smoothing_alpha"))
                    previous = measured
                    buffer.push(measured)
                else:
                    previous = None
                    median.reset()
                    buffer.push(PostureAngles(now, 0.0, 0.0, 0.0))

                # 교정 판단 스레드에는 인식 실패도 unknown으로 전달한다.
                # 실패 프레임을 생략하면 이전 나쁜 자세가 보이지 않은
                # 시간까지 지속된 것으로 오인할 수 있다.
                observed = measured or PostureAngles(now, 0.0, 0.0, 0.0)
                current_posture = classify(observed)
                if posture_observer is not None:
                    posture_observer.observe(now, current_posture, observed)
                if behavior is not None:
                    behavior.update_posture(observed)
                if response_tracker is not None:
                    response_tracker.update(now, current_posture.label)

                # 교정 이벤트 소비
                if behavior is not None:
                    event = behavior.poll_event()
                    if event:
                        action = None
                        event_now = time.monotonic()
                        age = event_now - event.timestamp
                        episode_id = (posture_observer.trigger(
                            event.timestamp, event.posture_label)
                            if posture_observer is not None else None)
                        event_is_current = (
                            observed.valid
                            and age >= 0.0
                            and age <= max_event_age_sec
                            and current_posture.label == event.posture_label
                        )
                        if event_is_current:
                            action = executor.build(event)
                            if action is not None:
                                delivered = False
                                simulated = False
                                accepted_outputs = []
                                if action.pose is not None:
                                    if writer is None:
                                        simulated = True
                                    else:
                                        delivered = control.submit_behavior(action)
                                        if delivered:
                                            accepted_outputs.append(
                                                "motor_control_queue")
                                if delivered and response_tracker is not None:
                                    applied_at = time.monotonic()
                                    response_tracker.intervention(
                                        applied_at, event.posture_label,
                                        action.behavior,
                                        episode_id=episode_id,
                                        accepted_outputs=accepted_outputs)
                                elif simulated and event_logger is not None:
                                    event_logger.record(
                                        "intervention_simulated",
                                        timestamp=time.monotonic(),
                                        posture=event.posture_label,
                                        behavior=action.behavior,
                                        episode_id=episode_id,
                                    )
                                elif not delivered and posture_observer is not None:
                                    posture_observer.intervention_skipped(
                                        time.monotonic(), "delivery_failed",
                                        posture=event.posture_label,
                                        behavior=action.behavior,
                                    )
                            else:
                                # 실행기가 행동을 만들지 못한 경우도 실제
                                # 개입이 아니므로 다음 감지를 허용한다.
                                if posture_observer is not None:
                                    posture_observer.intervention_skipped(
                                        time.monotonic(), "executor_returned_none",
                                        posture=event.posture_label,
                                    )
                                behavior.rearm_after_skipped_event()
                        else:
                            if posture_observer is not None:
                                reason = ("person_lost" if not observed.valid
                                          else "posture_changed" if
                                          current_posture.label != event.posture_label
                                          else "event_expired")
                                posture_observer.intervention_skipped(
                                    event_now, reason,
                                    posture=event.posture_label,
                                    event_age_sec=round(age, 4),
                                )
                            behavior.rearm_after_skipped_event()
                        d = event.decision
                        action_name = action.behavior if action else "ignore"
                        print(f"[자세반응] [{d.action}] {d.speech} "
                              f"(행동: {action_name}, 강도: {event.urgency})")

                frames += 1
                if now - fps_mark >= 1.0:
                    fps = frames / (now - fps_mark)
                    frames = 0
                    fps_mark = now

                played = measured

                if args.no_preview:
                    status_mark = print_status_line(
                        now, status_mark, control, measured, played, fps)
                else:
                    draw_preview(frame, control, measured, played,
                                 angles_config, landmarks, writer, fps)
                    cv2.imshow("Posture Robot", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break

                if control.error:
                    raise SafeStopRequested(
                        f"control_error: {control.error}")
        if control.error:
            raise SafeStopRequested(f"control_error: {control.error}")
        if control.safe_stop_reason:
            raise SafeStopRequested(control.safe_stop_reason)
        return 0
    except SafeStopRequested as exc:
        if event_logger is not None:
            event_logger.record("safe_stop", reason=str(exc))
        print(f"[SAFE_STOP] {exc}", file=sys.stderr)
        return 1
    except (BusError, CameraError, WriterError) as exc:
        sys.exit(str(exc))
    except KeyboardInterrupt:
        print("\n중단됨")
        return 0
    finally:
        if behavior is not None:
            behavior.stop()
            behavior.join(timeout=1.0)
        if control is not None:
            control.stop_event.set()
            control.join(timeout=2.0)
        if writer is not None and (control is None or control.motion_enabled):
            # 모터를 먼저 정리한 뒤 나머지 종료 작업을 수행한다.
            try:
                # 정리 경로도 운용 한계 검사를 건너뛰지 않는다.
                safe_neutral = safety_gate.clamp_targets(mapper.neutral_targets())
                writer.write_targets(safe_neutral)
                time.sleep(1.0)
                print("중립 복귀 후 토크 해제 완료")
            except Exception as exc:
                print(f"중립 복귀 중 오류: {exc}", file=sys.stderr)
            finally:
                # 중립 복귀 실패와 토크 해제를 묶지 않는다. 일부 관절에서
                # 통신이 실패해도 writer가 가능한 관절을 계속 시도한다.
                try:
                    writer.set_torque(False)
                    print("토크 해제 완료")
                except Exception as exc:
                    print(f"토크 해제 중 오류: {exc}", file=sys.stderr)
        close_at = time.monotonic()
        if response_tracker is not None:
            response_tracker.close(close_at)
        if posture_observer is not None:
            posture_observer.close(close_at)
        if event_logger is not None:
            event_logger.close()
        stack.close()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="자세 반응 로봇 루프")
    parser.add_argument("--move", action="store_true",
                        help="실제로 모터를 구동한다. 없으면 프리뷰만")
    parser.add_argument("--model",
                        help="lite / full / heavy 또는 .task 경로")
    parser.add_argument("--camera",
                        help="카메라 장치 경로 또는 인덱스. "
                             "생략하면 config/posture.yaml 의 perception.camera")
    parser.add_argument("--no-preview", action="store_true",
                        help="창을 띄우지 않는다 (헤드리스)")
    parser.add_argument("--no-camera-gimbal", action="store_true",
                        help="탑재 카메라 짐벌 보정을 끈다 (노트북 웹캠용)")
    parser.add_argument("--no-correction", action="store_true",
                        help="자세 반응/교정 판단을 끈다")
    parser.add_argument("--condition",
                        choices=("posture_trigger",),
                        help="실험 개입 조건 (현재 posture_trigger만 지원)")
    parser.add_argument("--participant-id",
                        help="로그용 익명 참가자 코드 (이름/이메일은 입력하지 마세요)")
    return run(parser.parse_args())


if __name__ == "__main__":
    sys.exit(main())
