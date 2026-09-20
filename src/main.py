#!/usr/bin/env python3
"""카메라로 본 자세를 로봇이 한 템포 늦게 따라한다.

메인 스레드가 카메라를 읽어 각도를 뽑고 타임스탬프와 함께 버퍼에 넣는다.
제어 스레드는 고정 주기로 "지금 - 지연" 시점의 값을 버퍼에서 보간해 꺼내
모터에 쓴다. 인식이 느려져도 재생 템포는 벽시계 기준으로 일정하다.

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
from src.perception.calibration import (  # noqa: E402
    CALIBRATION_PATH, average_reference, load_reference, save_reference,
)
from src.perception.posture_features import (  # noqa: E402
    MedianFilter, PostureAngles, apply_reference, clamp_angles, extract_angles,
    smooth,
)
from src.posture.pose_buffer import PoseBuffer  # noqa: E402
from src.posture.classifier import classify  # noqa: E402
from src.robot.dynamixel_driver import (  # noqa: E402
    BusError, PROJECT_ROOT, describe_hardware_error, install_signal_guards,
    load_joints, open_bus, ping_all, read_hardware_error,
)
from src.robot.joint_mapper import JointMapper, MappingError  # noqa: E402
from src.robot.joint_writer import JointWriter, WriterError  # noqa: E402
from src.behavior.behavior_manager import BehaviorManager
from src.behavior.executor import BehaviorExecutor  # noqa: E402
from src.safety.supervisor import (  # noqa: E402
    STATE_IDLE, STATE_SAFE_STOP, STATE_TRACKING, IdlePolicy,
)
from src.safety.gate import SafetyGate  # noqa: E402
from src.safety.health import HealthMonitor, SafeStopRequested  # noqa: E402
from src.audio.tts import SpeechQueue  # noqa: E402
from src.telemetry.event_log import EventLogger  # noqa: E402
from src.telemetry.response_tracker import ResponseTracker  # noqa: E402

POSTURE_CONFIG = PROJECT_ROOT / "config" / "posture.yaml"


def load_posture_config(path=POSTURE_CONFIG):
    if not Path(path).exists():
        raise FileNotFoundError(f"설정 파일이 없습니다: {path}")
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


class ControlLoop(threading.Thread):
    """고정 주기로 버퍼를 재생해 모터에 쓰는 스레드."""

    def __init__(self, buffer, mapper, writer, config, safety_gate=None,
                 condition="mirror"):
        super().__init__(name="control", daemon=True)
        echo = config["echo"]
        motion = config["motion"]
        self.buffer = buffer
        self.mapper = mapper
        self.condition = condition
        self.motion_enabled = condition == "mirror"
        # 조건이 음성이면 실수로 writer가 전달되어도 이 루프는 하드웨어
        # 초기화·토크·목표 쓰기를 수행하지 않는다.
        self.writer = writer
        self.delay = echo["delay_sec"]
        self.period = 1.0 / echo["control_hz"]
        self.safety_gate = safety_gate or SafetyGate(mapper, config)
        self.policy = IdlePolicy(
            motion["return_to_neutral_sec"], motion["idle_release_sec"],
            grace_seconds=motion.get("person_lost_grace_sec", 0.5))
        self.neutral = mapper.neutral_targets()
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

    def submit_behavior(self, action):
        """고정된 행동을 다음 제어 tick부터 재생한다."""
        if action is None:
            return
        with self._behavior_lock:
            self._behavior_action = action
            self._behavior_started_at = time.monotonic()

    def safe_stop(self, reason):
        """추적을 중지하고 finally의 중립 복귀 경로로 보낸다."""
        self.safe_stop_reason = str(reason)
        self.state = STATE_SAFE_STOP
        with self._behavior_lock:
            self._behavior_action = None
            self._behavior_started_at = None
        self.stop_event.set()

    def _behavior_pose(self, now):
        with self._behavior_lock:
            action = self._behavior_action
            started = self._behavior_started_at
            if action is None or started is None:
                return None
            if now - started >= action.duration_sec:
                self._behavior_action = None
                self._behavior_started_at = None
                return None
            return action.pose

    def run(self):
        try:
            self._loop()
        except Exception as exc:                      # 스레드 밖으로 전달
            self.error = exc
            self.safe_stop(f"control_error: {exc}")

    def _loop(self):
        if self.writer is not None and self.motion_enabled:
            self.writer.prepare()
            self.commanded = self.writer.read_positions() or dict(self.neutral)

        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            self.heartbeat_at = now
            played = self.buffer.sample(now - self.delay)
            person_visible = played is not None and played.valid

            state, torque = self.policy.update(now, person_visible,
                                               self.commanded, self.neutral)
            self.state = state

            if (self.motion_enabled and person_visible
                    and state == STATE_TRACKING):
                behavior_pose = self._behavior_pose(now)
                desired = self.mapper.to_targets(behavior_pose or played)
            else:
                desired = dict(self.neutral)

            # 일반 추적, 행동 재생, 중립 복귀 모두 같은 관문을 통과한다.
            limited = self.safety_gate.limit_targets(self.commanded, desired)
            self.commanded = limited
            self.last_targets = limited

            if self.writer is not None and self.motion_enabled:
                if state == STATE_IDLE:
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
        if position is None:
            continue
        if not spec["min_position"] <= position <= spec["max_position"]:
            folded.append(f"  {name} 현재 {position}, 운용 범위 "
                          f"{spec['min_position']}~{spec['max_position']}")
    if folded:
        raise BusError("현재 자세가 운용 범위를 벗어나 있습니다.\n"
                       + "\n".join(folded)
                       + "\n손으로 컬럼을 세운 뒤 다시 실행하세요.")


def print_status_line(now, status_mark, control, measured, played, fps):
    """헤드리스 모드에서 0.5초마다 상태를 한 줄로 찍는다."""
    if now - status_mark < 0.5:
        return status_mark
    measured_text = (
        "torso {:+6.1f} neck {:+6.1f}".format(
            measured.torso_pitch_deg, measured.neck_pitch_deg)
        if measured else "사람 없음            ")
    played_text = (
        "torso {:+6.1f} neck {:+6.1f}".format(
            played.torso_pitch_deg, played.neck_pitch_deg)
        if played and played.valid else "대기                 ")
    targets = control.last_targets
    print(f"[{control.state:<9}] fps {fps:4.1f} | "
          f"측정 {measured_text} | 재생 {played_text} | "
          f"목표 {targets}", flush=True)
    return now


def draw_preview(frame, control, measured, played, echo, angles_config,
                 landmarks, writer, fps):
    """프리뷰 창에 스켈레톤과 오버레이를 그린다."""
    draw_skeleton(frame, landmarks)
    lines = [
        f"state {control.state}   fps {fps:4.1f}   "
        f"delay {echo['delay_sec']:.1f}s",
        ("측정  torso {:+6.1f}  neck {:+6.1f}".format(
            measured.torso_pitch_deg, measured.neck_pitch_deg)
         if measured else "측정  사람을 찾는 중"),
        ("재생  torso {:+6.1f}  neck {:+6.1f}".format(
            played.torso_pitch_deg, played.neck_pitch_deg)
         if played and played.valid else "재생  대기"),
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
    perception = config["perception"]
    angles_config = config["angles"]
    echo = config["echo"]

    joints = load_joints()
    if not joints:
        sys.exit("config/joints.yaml 에 joints 가 없습니다.")
    try:
        mapper = JointMapper(joints, config["distribution"])
    except MappingError as exc:
        sys.exit(f"관절 매핑 설정 오류: {exc}")

    buffer = PoseBuffer(echo["buffer_seconds"])
    median = MedianFilter(angles_config["median_window"])
    reference = load_reference()

    experiment = config.get("experiment") or {}
    condition = args.condition or experiment.get("condition", "mirror")
    if args.move and condition == "voice":
        sys.exit("voice 조건에서는 모터를 구동할 수 없습니다. "
                 "--move 를 빼고 실행하세요.")

    if args.move and reference is None:
        sys.exit(
            "캘리브레이션이 없습니다. 정면 카메라에서는 골반을 추정으로 채우기\n"
            "때문에 사람과 자리마다 상체각에 일정한 치우침이 생깁니다. 그대로\n"
            "구동하면 로봇이 굽은 자세를 중립으로 착각합니다.\n"
            "  ~/dynamixel-venv/bin/python -m src.main --calibrate")

    calibration_samples = []
    calibration_deadline = None

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
    try:
        executor = BehaviorExecutor(config, safety_gate, condition)
    except ValueError as exc:
        sys.exit(str(exc))

    event_logger = None
    response_tracker = None
    speaker = None
    if not args.calibrate:
        log_dir = Path(experiment.get("log_dir", "data/runs"))
        if not log_dir.is_absolute():
            log_dir = PROJECT_ROOT / log_dir
        event_logger = EventLogger(log_dir, condition=condition)
        response_tracker = ResponseTracker(
            event_logger,
            response_timeout_sec=float(
                experiment.get("response_timeout_sec", 15.0)),
        )
        audio = config.get("audio") or {}
        speaker = SpeechQueue(audio.get("tts") or {})
        speaker.start()
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
        if not args.no_correction and not args.calibrate:
            behavior = BehaviorManager(config)
            behavior.start()
            print(f"교정 모드 활성. 조건={condition}, Gemini 가 자세를 판단합니다 "
                  "(--no-correction 으로 끌 수 있음).")

        source = args.camera or perception.get("camera",
                                               perception.get("camera_index", 0))
        with CameraStream(source, perception["width"],
                          perception["height"]) as camera, \
                PoseEstimator(args.model
                              or perception.get("model")) as estimator:
            if args.calibrate:
                print("바른 자세로 앉으세요. 3초 뒤부터 3초간 측정합니다.")
                time.sleep(3.0)
                print("측정 중, 움직이지 마세요...")
                calibration_deadline = time.monotonic() + 3.0
            else:
                print("자세 모방 시작. 프리뷰 창에서 q 또는 Esc 로 종료합니다.")
                if reference is None:
                    print("경고: 캘리브레이션 없음. 각도에 치우침이 남습니다 "
                          "(--calibrate 로 보정).")
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

                raw = measured
                if measured is not None:
                    # 순서가 중요하다. 중앙값으로 튀는 값을 먼저 버리고,
                    # 기준값을 빼서 편향을 없앤 뒤, 자르고 부드럽게 한다.
                    measured = median.apply(measured)
                    measured = apply_reference(measured, reference)
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

                # 교정 판단 스레드에 최신 자세 전달
                if behavior is not None and measured is not None:
                    behavior.update_posture(measured)
                if response_tracker is not None and measured is not None:
                    response_tracker.update(now, classify(measured).label)

                # 교정 이벤트 소비
                if behavior is not None:
                    event = behavior.poll_event()
                    if event:
                        action = executor.build(event)
                        if action is not None:
                            if action.pose is not None:
                                control.submit_behavior(action)
                            applied_at = time.monotonic()
                            if response_tracker is not None:
                                response_tracker.intervention(
                                    applied_at, event.posture_label,
                                    action.behavior)
                            if speaker is not None:
                                speaker.submit(action.speech, now=applied_at)
                        d = event.decision
                        action_name = action.behavior if action else "ignore"
                        print(f"[교정] [{d.action}] {d.speech} "
                              f"(행동: {action_name}, 강도: {event.urgency})")

                if calibration_deadline is not None:
                    if raw is not None:
                        calibration_samples.append(raw)
                    if now >= calibration_deadline:
                        if len(calibration_samples) < 10:
                            sys.exit("표본이 부족합니다. 카메라 앞에 앉아서 "
                                     "다시 시도하세요.")
                        reference = average_reference(calibration_samples)
                        target = save_reference(reference,
                                                len(calibration_samples))
                        print(f"기준값 저장: {target}")
                        print(f"  상체 {reference.torso_pitch_deg:+.2f}도  "
                              f"목 {reference.neck_pitch_deg:+.2f}도  "
                              f"({len(calibration_samples)} 표본)")
                        return 0

                frames += 1
                if now - fps_mark >= 1.0:
                    fps = frames / (now - fps_mark)
                    frames = 0
                    fps_mark = now

                played = buffer.sample(now - echo["delay_sec"])

                if args.no_preview:
                    status_mark = print_status_line(
                        now, status_mark, control, measured, played, fps)
                else:
                    draw_preview(frame, control, measured, played, echo,
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
            # 모터를 먼저 정리한다. TTS 종료는 외부 프로세스가 반환될 때까지
            # 기다릴 수 있으므로 하드웨어 정리보다 앞에 두면 안 된다.
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
        if response_tracker is not None:
            response_tracker.close()
        if speaker is not None:
            speaker.stop()
        if event_logger is not None:
            event_logger.close()
        stack.close()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="자세 모방 루프")
    parser.add_argument("--move", action="store_true",
                        help="실제로 모터를 구동한다. 없으면 프리뷰만")
    parser.add_argument("--calibrate", action="store_true",
                        help="바른 자세를 3초간 재서 기준값으로 저장하고 끝낸다")
    parser.add_argument("--model",
                        help="lite / full / heavy 또는 .task 경로")
    parser.add_argument("--camera",
                        help="카메라 장치 경로 또는 인덱스. "
                             "생략하면 config/posture.yaml 의 perception.camera")
    parser.add_argument("--no-preview", action="store_true",
                        help="창을 띄우지 않는다 (헤드리스)")
    parser.add_argument("--no-correction", action="store_true",
                        help="Gemini 교정 판단을 끈다 (에코만)")
    parser.add_argument("--condition",
                        choices=("voice", "mirror"),
                        help="실험 개입 조건. 생략하면 config/posture.yaml 사용")
    return run(parser.parse_args())


if __name__ == "__main__":
    sys.exit(main())
