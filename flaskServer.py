from flask import Flask, request, jsonify, render_template
from robot_control import RobotControl
from dialog_engine import DialogEngine
from action_runner import ActionRunner
from wall_follower import (
    WALL_FOLLOW_CONFIG_FIELDS,
    WALL_FOLLOW_PROFILES,
    WallFollower,
    get_wall_follow_profile,
)
try:
    from lidar_safety import LidarSafetyMonitor
    LIDAR_IMPORT_ERROR = None
except Exception as _lidar_import_ex:
    LidarSafetyMonitor = None
    LIDAR_IMPORT_ERROR = str(_lidar_import_ex)

import logging
from werkzeug.serving import WSGIRequestHandler
import threading
import subprocess
import re
import time
import os
import random
import argparse
from typing import Optional

class QuietHandler(WSGIRequestHandler):
    def log_request(self, code='-', size='-'):
        # Suppress heartbeat spam
        if (
            self.path.startswith("/api/heartbeat")
            or self.path.startswith("/api/final/status")
            or self.path.startswith("/api/dialog_last")
        ):
            return
        super().log_request(code, size)

app = Flask(__name__)

# One shared controller instance for the server
ctrl = RobotControl(port="/dev/ttyACM0", device=0x0C)

dialog_lock = threading.Lock()
dialog_engine = None
action_runner = None
dialog_state_override = None
lidar_monitor = None
wall_follower = None
LIDAR_CLEAR_SCANS_REQUIRED = 3
LIDAR_MOTION_GUARD_PERIOD_S = 0.1

FINAL_APPROACH_MIN_DROP_MM = 250
FINAL_APPROACH_MAX_MM = 1800
FINAL_APPROACH_IMMEDIATE_MM = 650
FINAL_APPROACH_MIN_WAIT_S = 0.75
FINAL_APPROACH_TIMEOUT_S = 60.0
FINAL_TURN_SPEED = 1000
FINAL_TURN_180_S = 1.60
FINAL_TURN_90_S = 0.80
FINAL_FORWARD_SPEED = 900
FINAL_SIDE_WALL_DETECT_MM = 1500
FINAL_ALIGN_MIN_DRIVE_S = 1.2
FINAL_ALIGN_CONFIRM_SCANS = 5
FINAL_ALIGN_TIMEOUT_S = 12.0
FINAL_T_INTERSECTION_TIMEOUT_S = 45.0
FINAL_FINISH_DRIVE_S = 5.0
FINAL_OBSTACLE_STOP_MM = 900
FINAL_OBSTACLE_CLEAR_MM = 1150
FINAL_ACK_PAUSE_S = 2.8

_motion_lock = threading.Lock()
_motion_left = 0
_motion_right = 0
_motion_active = False

_dialog_event_lock = threading.Lock()
_last_dialog_event = {
    "input": "",
    "matched": False,
    "reply": "",
    "actions": [],
    "state": "BOOT",
    "scope_depth": 0,
    "source": "none",
    "updated_at": 0.0,
}

_final_demo_lock = threading.Lock()
_final_demo_cancel = threading.Event()
_final_demo_thread = None
_final_demo_status = {
    "active": False,
    "state": "IDLE",
    "message": "Final demo idle.",
    "destination": None,
    "front_mm": None,
    "baseline_front_mm": None,
    "left_mm": None,
    "right_mm": None,
    "wall_state": None,
    "started_at": None,
    "updated_at": time.time(),
}


def set_dialog_state(value: Optional[str]):
    global dialog_state_override
    with dialog_lock:
        dialog_state_override = value


def get_dialog_state() -> str:
    with dialog_lock:
        if dialog_state_override is not None:
            return dialog_state_override
        if dialog_engine is None:
            return "BOOT"
        return dialog_engine.state


def configure_dialog_engine(script_path: str, seed: int | None):
    global dialog_engine, action_runner, dialog_state_override
    if action_runner is not None:
        action_runner.interrupt()
    dialog_engine = DialogEngine.from_file(script_path, seed=seed)
    for err in dialog_engine.errors:
        print(f"[DIALOG PARSE] {err}")
    if dialog_engine.has_fatal_errors():
        print("[DIALOG] fatal errors found; dialog engine will refuse to run")
    action_runner = ActionRunner(ctrl, on_state_change=set_dialog_state)
    dialog_state_override = None
    print(f"[DIALOG] loaded script={script_path} seed={seed}")

def bad(msg, code=400):
    return jsonify({"ok": False, "error": msg}), code


def _lidar_block_for_motion(left: int, right: int):
    """
    Returns (blocked: bool, direction: str|None, status: dict|None)
    direction is 'forward', 'backward', or None.
    """
    if lidar_monitor is None:
        return (False, None, None)

    status = lidar_monitor.status()
    linear = left + right
    if linear > 0:
        blocked = bool(status["front_blocked"])
        return (blocked, "forward", status)
    if linear < 0:
        blocked = bool(status["rear_blocked"])
        return (blocked, "backward", status)
    return (False, None, status)


def _set_motion_command(left: int, right: int):
    global _motion_left, _motion_right, _motion_active
    with _motion_lock:
        _motion_left = int(left)
        _motion_right = int(right)
        _motion_active = (_motion_left != 0 or _motion_right != 0)


def _clear_motion_command():
    _set_motion_command(0, 0)


def _get_motion_command():
    with _motion_lock:
        return _motion_left, _motion_right, _motion_active


def _stop_wall_follower_if_active(reason: str):
    global wall_follower
    if wall_follower is not None and wall_follower.active:
        print(f"[WALL] stopping due to {reason}")
        wall_follower.stop()


def _store_dialog_event(event: dict) -> None:
    global _last_dialog_event
    with _dialog_event_lock:
        merged = dict(_last_dialog_event)
        merged.update(event)
        merged["updated_at"] = time.time()
        _last_dialog_event = merged


def _get_dialog_event() -> dict:
    with _dialog_event_lock:
        return dict(_last_dialog_event)


def _set_final_demo_state(state: str, message: str, **extra) -> None:
    with _final_demo_lock:
        prev_state = _final_demo_status.get("state")
        prev_message = _final_demo_status.get("message")
        prev_log_at = float(_final_demo_status.get("_log_at") or 0.0)
        now = time.time()
        _final_demo_status.update(
            {
                "state": state,
                "message": message,
                "updated_at": now,
            }
        )
        _final_demo_status.update(extra)
        should_log = state != prev_state or message != prev_message or (now - prev_log_at) >= 3.0
        if should_log:
            _final_demo_status["_log_at"] = now
            print(f"[FINAL] {state}: {message}")


def _get_final_demo_status() -> dict:
    with _final_demo_lock:
        status = dict(_final_demo_status)
    status.pop("_log_at", None)
    return status


def _front_distance_from_lidar_status(status: dict | None) -> Optional[float]:
    if not status:
        return None
    candidates = []
    for key in ("front_min_mm",):
        value = status.get(key)
        if value is not None:
            candidates.append(float(value))
    zone_mins = status.get("zone_mins_mm") or {}
    for key in ("front_center", "front"):
        value = zone_mins.get(key)
        if value is not None:
            candidates.append(float(value))
    if not candidates:
        return None
    return min(candidates)


def _destination_from_text(text: str, explicit_destination: object = None) -> Optional[str]:
    hay = f"{explicit_destination or ''} {text or ''}".lower()
    if any(word in hay for word in ("bathroom", "restroom", "washroom")):
        return "bathroom"
    if "robotics lab" in hay or "robot lab" in hay or re.search(r"\blab\b", hay):
        return "robot lab"
    return None


def _final_demo_is_waiting_for_destination() -> bool:
    with _final_demo_lock:
        return bool(_final_demo_status["active"] and _final_demo_status["state"] == "WAIT_DESTINATION")


def _final_demo_is_active() -> bool:
    with _final_demo_lock:
        return bool(_final_demo_status["active"])


def _reset_final_demo_status(message: str = "Final demo idle.") -> None:
    with _final_demo_lock:
        _final_demo_status.update(
            {
                "active": False,
                "state": "IDLE",
                "message": message,
                "destination": None,
                "front_mm": None,
                "baseline_front_mm": None,
                "left_mm": None,
                "right_mm": None,
                "wall_state": None,
                "started_at": None,
                "updated_at": time.time(),
            }
        )


def _cancel_final_demo(reason: str) -> None:
    _final_demo_cancel.set()
    _set_final_demo_state("CANCELLED", reason)
    try:
        _stop_wall_follower_if_active("final demo cancel")
        ctrl.stop()
        _clear_motion_command()
    except Exception as ex:
        print(f"[FINAL] cancel stop failed: {ex}")
    _reset_final_demo_status(reason)


def _sleep_with_final_cancel(seconds: float) -> bool:
    end = time.time() + seconds
    while time.time() < end:
        if _final_demo_cancel.is_set():
            return False
        # The final demo intentionally runs autonomously for several seconds
        # between browser requests, so keep the server watchdog from treating
        # planned speech/turn/drive time as a lost-client emergency.
        touch_heartbeat()
        time.sleep(0.03)
    return True


def _final_turn_left(seconds: float, label: str) -> bool:
    _set_final_demo_state(label, f"Turning left for {seconds:.2f}s.")
    try:
        ctrl.turn_left(FINAL_TURN_SPEED)
        _set_motion_command(-FINAL_TURN_SPEED, FINAL_TURN_SPEED)
        return _sleep_with_final_cancel(seconds)
    finally:
        ctrl.stop()
        _clear_motion_command()


def _final_turn_right(seconds: float, label: str) -> bool:
    _set_final_demo_state(label, f"Turning right for {seconds:.2f}s.")
    try:
        ctrl.turn_right(FINAL_TURN_SPEED)
        _set_motion_command(FINAL_TURN_SPEED, -FINAL_TURN_SPEED)
        return _sleep_with_final_cancel(seconds)
    finally:
        ctrl.stop()
        _clear_motion_command()


def _final_front_obstacle(status: dict | None) -> bool:
    front_mm = _front_distance_from_lidar_status(status)
    return bool(status and status.get("front_blocked")) or (
        front_mm is not None and front_mm <= FINAL_OBSTACLE_STOP_MM
    )


def _final_front_clear(status: dict | None) -> bool:
    front_mm = _front_distance_from_lidar_status(status)
    return front_mm is None or front_mm >= FINAL_OBSTACLE_CLEAR_MM


def _final_wait_front_clear() -> bool:
    _set_final_demo_state("OBSTACLE_WAIT", "Obstacle detected in front. Waiting until clear.")
    while not _final_demo_cancel.is_set():
        status = lidar_monitor.status() if lidar_monitor is not None else {}
        front_mm = _front_distance_from_lidar_status(status)
        _set_final_demo_state(
            "OBSTACLE_WAIT",
            "Obstacle detected in front. Waiting until clear.",
            front_mm=front_mm,
        )
        if _final_front_clear(status):
            return True
        ctrl.stop()
        _clear_motion_command()
        time.sleep(0.15)
    return False


def _final_drive_forward_until_side_walls() -> bool:
    start = time.time()
    aligned_scans = 0
    _set_final_demo_state(
        "ALIGN_HALLWAY",
        f"Driving forward until both side walls are within {FINAL_SIDE_WALL_DETECT_MM} mm.",
    )
    try:
        while not _final_demo_cancel.is_set():
            if time.time() - start > FINAL_ALIGN_TIMEOUT_S:
                _set_final_demo_state("ALIGN_TIMEOUT", "Timed out before seeing both side walls.")
                return False

            status = lidar_monitor.status() if lidar_monitor is not None else {}
            if _final_front_obstacle(status):
                ctrl.stop()
                _clear_motion_command()
                if not _final_wait_front_clear():
                    return False
                aligned_scans = 0

            zone_mins = status.get("zone_mins_mm") or {}
            left_mm = zone_mins.get("left")
            right_mm = zone_mins.get("right")
            front_mm = _front_distance_from_lidar_status(status)
            elapsed = time.time() - start
            sides_detected = (
                left_mm is not None
                and right_mm is not None
                and float(left_mm) <= FINAL_SIDE_WALL_DETECT_MM
                and float(right_mm) <= FINAL_SIDE_WALL_DETECT_MM
            )
            if elapsed >= FINAL_ALIGN_MIN_DRIVE_S and sides_detected:
                aligned_scans += 1
            else:
                aligned_scans = 0
            _set_final_demo_state(
                "ALIGN_HALLWAY",
                "Driving forward until both side walls are detected.",
                front_mm=front_mm,
                left_mm=left_mm,
                right_mm=right_mm,
            )
            if aligned_scans >= FINAL_ALIGN_CONFIRM_SCANS:
                ctrl.stop_smooth()
                _clear_motion_command()
                _set_final_demo_state("HALLWAY_ALIGNED", "Both side walls detected.")
                return True

            ctrl.drive_autonomous(FINAL_FORWARD_SPEED, FINAL_FORWARD_SPEED)
            _set_motion_command(FINAL_FORWARD_SPEED, FINAL_FORWARD_SPEED)
            if not _sleep_with_final_cancel(0.15):
                return False
        return False
    finally:
        ctrl.stop()
        _clear_motion_command()


def _final_wall_follow_until_t_intersection() -> bool:
    start = time.time()
    _start_final_right_wall_follow()
    while not _final_demo_cancel.is_set():
        if time.time() - start > FINAL_T_INTERSECTION_TIMEOUT_S:
            _set_final_demo_state("T_TIMEOUT", "Timed out before detecting the T-intersection.")
            return False

        status = wall_follower.status() if wall_follower is not None else {}
        _set_final_demo_state(
            "WALL_FOLLOW_TO_T",
            "Wall following until T-intersection is detected.",
            wall_state=status.get("last_state"),
        )
        if status.get("last_intersection_detected") or status.get("last_state") == "T_INTERSECTION_DETECTED":
            _stop_wall_follower_if_active("final demo T-intersection detected")
            ctrl.stop()
            _clear_motion_command()
            _set_final_demo_state("T_INTERSECTION", "T-intersection detected.")
            return True
        time.sleep(0.2)
    return False


def _final_decision_turn(destination: str) -> bool:
    if destination == "bathroom":
        return _final_turn_right(FINAL_TURN_90_S, "TURN_RIGHT_TO_BATHROOM")
    return _final_turn_left(FINAL_TURN_90_S, "TURN_LEFT_TO_ROBOT_LAB")


def _final_drive_forward_for_finish() -> bool:
    elapsed_drive = 0.0
    last = time.time()
    _set_final_demo_state("FINAL_STRAIGHT", f"Driving straight for {FINAL_FINISH_DRIVE_S:.1f}s.")
    try:
        while not _final_demo_cancel.is_set() and elapsed_drive < FINAL_FINISH_DRIVE_S:
            now = time.time()
            dt = now - last
            last = now

            status = lidar_monitor.status() if lidar_monitor is not None else {}
            front_mm = _front_distance_from_lidar_status(status)
            if _final_front_obstacle(status):
                ctrl.stop()
                _clear_motion_command()
                if not _final_wait_front_clear():
                    return False
                last = time.time()
                continue

            elapsed_drive += dt
            _set_final_demo_state(
                "FINAL_STRAIGHT",
                f"Driving straight for {FINAL_FINISH_DRIVE_S:.1f}s.",
                front_mm=front_mm,
            )
            ctrl.drive_autonomous(FINAL_FORWARD_SPEED, FINAL_FORWARD_SPEED)
            _set_motion_command(FINAL_FORWARD_SPEED, FINAL_FORWARD_SPEED)
            if not _sleep_with_final_cancel(0.1):
                return False
        ctrl.stop_smooth()
        _clear_motion_command()
        return True
    finally:
        ctrl.stop()
        _clear_motion_command()


def _start_final_right_wall_follow() -> None:
    global wall_follower
    if lidar_monitor is None:
        raise RuntimeError("lidar monitor not configured")
    if wall_follower is None:
        wall_follower = WallFollower(ctrl, lidar_monitor)
    cfg = get_wall_follow_profile("final", side="right")
    start_kwargs = {}
    for key in WALL_FOLLOW_CONFIG_FIELDS:
        if key not in cfg:
            continue
        start_kwargs[key] = cfg[key]
    wall_follower.start(**start_kwargs)
    _set_final_demo_state(
        "WALL_FOLLOW_RIGHT",
        "Right wall following started.",
        active=True,
    )


def _final_destination_ack(destination: str) -> str:
    if destination == "robot lab":
        return "Sure, I will take you to the robot lab. Follow me."
    if destination == "bathroom":
        return "Sure, I will take you to the bathroom. Follow me."
    return f"Sure, I will take you to the {destination}. Follow me."


def _final_demo_wait_for_approach() -> bool:
    start = time.time()
    baseline_front = None
    _set_final_demo_state(
        "WAIT_APPROACH",
        "Waiting for a person to approach the front LIDAR.",
        active=True,
        started_at=start,
    )

    while not _final_demo_cancel.is_set():
        if time.time() - start > FINAL_APPROACH_TIMEOUT_S:
            _set_final_demo_state("TIMEOUT", "No approaching person detected.")
            return False
        if lidar_monitor is None:
            _set_final_demo_state("ERROR", "LIDAR monitor is not configured.")
            return False

        status = lidar_monitor.status()
        front_mm = _front_distance_from_lidar_status(status)
        if front_mm is not None:
            if baseline_front is None or front_mm > baseline_front:
                baseline_front = front_mm

            elapsed = time.time() - start
            drop = baseline_front - front_mm if baseline_front is not None else 0
            close_approach = (
                elapsed >= FINAL_APPROACH_MIN_WAIT_S
                and front_mm <= FINAL_APPROACH_MAX_MM
                and drop >= FINAL_APPROACH_MIN_DROP_MM
            )
            very_close_after_wait = (
                elapsed >= FINAL_APPROACH_MIN_WAIT_S
                and front_mm <= FINAL_APPROACH_IMMEDIATE_MM
            )
            _set_final_demo_state(
                "WAIT_APPROACH",
                "Waiting for front LIDAR distance to get smaller.",
                front_mm=front_mm,
                baseline_front_mm=baseline_front,
            )
            if close_approach or very_close_after_wait:
                return True

        time.sleep(0.2)
    return False


def _final_demo_greet_worker() -> None:
    try:
        if not _final_demo_wait_for_approach():
            _reset_final_demo_status(_get_final_demo_status().get("message", "Demo stopped."))
            return
        if _final_demo_cancel.is_set():
            return
        _set_final_demo_state(
            "GREET",
            "Person detected. Greeting and waiting for Vosk destination command.",
        )
        speak_async("Greetings. Where would you like to go?")
        _set_final_demo_state(
            "WAIT_DESTINATION",
            "Listening through local Vosk. Say bathroom or robot lab.",
            active=True,
        )
    except Exception as ex:
        _set_final_demo_state("ERROR", f"Final demo greeting failed: {ex}")
        _reset_final_demo_status(f"Final demo greeting failed: {ex}")


def _final_demo_navigation_worker(destination: str) -> None:
    try:
        _set_final_demo_state(
            "NAV_START",
            f"Starting route toward {destination}.",
            active=True,
            destination=destination,
        )
        speak_async(_final_destination_ack(destination))
        if not _sleep_with_final_cancel(FINAL_ACK_PAUSE_S):
            return
        if not _final_turn_left(FINAL_TURN_180_S, "TURN_180"):
            return
        if not _final_drive_forward_until_side_walls():
            return
        if not _final_wall_follow_until_t_intersection():
            return
        if not _final_decision_turn(destination):
            return
        if not _final_drive_forward_for_finish():
            return
        if _final_demo_cancel.is_set():
            return
        _set_final_demo_state("ARRIVED", f"Arrived at {destination}.", active=False)
        speak_async(f"We have arrived at the {destination}.")
    except Exception as ex:
        _set_final_demo_state("ERROR", f"Final demo navigation failed: {ex}")
        try:
            ctrl.stop()
            _clear_motion_command()
        except Exception:
            pass


# =========================
# Watchdog / Force Stop
# =========================

HEARTBEAT_TIMEOUT_S = 3.0   # allow browser mic prompts / Pi load without nuisance stops
WATCHDOG_PERIOD_S = 0.25    # how often watchdog checks

_last_heartbeat = time.time()
_has_seen_heartbeat = False
_force_stop_lock = threading.Lock()
_force_stop_running = False

def touch_heartbeat():
    global _last_heartbeat, _has_seen_heartbeat
    _last_heartbeat = time.time()
    _has_seen_heartbeat = True

def run_force_stop_async(reason: str):
    """
    Runs force_stop.py in the background, but prevents overlap.
    """
    global _force_stop_running

    with _force_stop_lock:
        if _force_stop_running:
            return
        _force_stop_running = True

    def worker():
        global _force_stop_running
        try:
            print(f"[WATCHDOG] FORCE STOP triggered: {reason}")
            if _final_demo_is_active():
                _final_demo_cancel.set()
                _reset_final_demo_status(f"Final demo stopped: {reason}")
            _stop_wall_follower_if_active("watchdog force stop")
            if action_runner is not None:
                action_runner.interrupt()
            if dialog_engine is not None:
                dialog_engine.reset_to_idle("watchdog force stop")
            try:
                ctrl.stop_smooth()
            except Exception as e:
                print(f"[WATCHDOG] ctrl.stop() failed: {e}")

        finally:
            with _force_stop_lock:
                _force_stop_running = False

    threading.Thread(target=worker, daemon=True).start()


def watchdog_loop():
    """
    Background thread: if heartbeat becomes stale -> force stop ONCE,
    then wait until heartbeat returns before allowing another trigger.
    """
    global _last_heartbeat, _has_seen_heartbeat
    timed_out = False  # local state: have we already triggered for the current outage?

    while True:
        time.sleep(WATCHDOG_PERIOD_S)
        if not _has_seen_heartbeat:
            # Do not watchdog-force-stop before the browser has ever connected.
            continue
        age = time.time() - _last_heartbeat

        if age > HEARTBEAT_TIMEOUT_S:
            # Only trigger once per outage
            if not timed_out:
                run_force_stop_async(f"heartbeat timeout ({age:.2f}s > {HEARTBEAT_TIMEOUT_S}s)")
                timed_out = True
        else:
            # Heartbeat is healthy again -> allow future triggers
            timed_out = False


def lidar_motion_guard_loop():
    """
    Background thread: if wheels are already running from a prior command,
    enforce lidar stop immediately when front/rear become blocked.
    """
    while True:
        time.sleep(LIDAR_MOTION_GUARD_PERIOD_S)
        if lidar_monitor is None:
            continue

        left, right, active = _get_motion_command()
        if not active:
            continue

        s = lidar_monitor.status()
        linear = left + right
        front_blocked = bool(s.get("front_blocked"))
        rear_blocked = bool(s.get("rear_blocked"))

        should_stop = (linear > 0 and front_blocked) or (linear < 0 and rear_blocked)
        if not should_stop:
            continue

        direction = "forward" if linear > 0 else "backward"
        try:
            ctrl.stop_smooth()
            _clear_motion_command()
            print(
                f"[LIDAR SAFETY] forced stop while moving direction={direction} "
                f"left={left} right={right}"
            )
        except Exception as e:
            print(f"[LIDAR SAFETY] forced stop failed: {e}")


# start watchdog thread
threading.Thread(target=watchdog_loop, daemon=True).start()
threading.Thread(target=lidar_motion_guard_loop, daemon=True).start()


# =========================
# TTS helpers
# =========================

def sanitize_tts(text: str) -> str:
    # remove control chars, collapse whitespace
    text = re.sub(r"[\x00-\x1F\x7F]", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text

def speak_async(text: str):
    def run():
        try:
            subprocess.run(["espeak-ng", "-s", "165", "-v", "en-us", text], check=False)
        except FileNotFoundError:
            print(f"[TTS WARN] espeak-ng not installed; cannot speak: {text}")
    threading.Thread(target=run, daemon=True).start()


DEFAULT_DIALOG_SCRIPT = os.path.join(os.path.dirname(__file__), "testDialogFileForPractice.txt")
configure_dialog_engine(DEFAULT_DIALOG_SCRIPT, seed=None)


@app.route("/")
def index():
    return render_template("index.html")


# =========================
# Heartbeat API
# =========================

@app.route("/api/heartbeat", methods=["POST"])
def api_heartbeat():
    touch_heartbeat()
    return jsonify({"ok": True, "t": time.time()})


# Optional: manual “panic button” endpoint (handy for testing)
@app.route("/api/force_stop", methods=["POST"])
def api_force_stop():
    if _final_demo_is_active():
        _cancel_final_demo("manual force stop")
    if action_runner is not None:
        action_runner.interrupt()
    if dialog_engine is not None:
        dialog_engine.reset_to_idle("manual force stop")
    run_force_stop_async("manual /api/force_stop")
    return jsonify({"ok": True})


# =========================
# DRIVE API
# =========================

@app.route("/api/drive", methods=["POST"])
def api_drive():
    touch_heartbeat()  # treat commands as “activity” too
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/drive")
    _stop_wall_follower_if_active("manual /api/drive")

    data = request.get_json(silent=True) or {}
    if "left" not in data or "right" not in data:
        return bad("Missing 'left' or 'right'")

    try:
        left = int(data["left"])
        right = int(data["right"])
    except (ValueError, TypeError):
        return bad("left/right must be integers")

    if abs(left) > 3000 or abs(right) > 3000:
        return bad("left/right out of allowed range")

    try:
        blocked, direction, lidar_status = _lidar_block_for_motion(left, right)
        if blocked:
            ctrl.stop_smooth()
            _clear_motion_command()
            print(f"[LIDAR SAFETY] blocked {direction} drive command left={left} right={right}")
            return jsonify(
                {
                    "ok": True,
                    "blocked": True,
                    "direction": direction,
                    "left": 0,
                    "right": 0,
                    "lidar": lidar_status,
                }
            )
        ctrl.drive(left, right)
        _set_motion_command(left, right)
    except Exception as e:
        run_force_stop_async(f"drive exception: {e}")
        return bad(f"drive failed: {e}", code=500)

    return jsonify({"ok": True, "blocked": False, "left": left, "right": right})


@app.route("/api/forward", methods=["POST"])
def api_forward():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/forward")
    _stop_wall_follower_if_active("manual /api/forward")
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        if lidar_monitor is not None:
            s = lidar_monitor.status()
            if s["front_blocked"]:
                ctrl.stop_smooth()
                _clear_motion_command()
                print(f"[LIDAR SAFETY] blocked forward speed={speed}")
                return jsonify({"ok": True, "blocked": True, "direction": "forward", "speed": 0, "lidar": s})
        ctrl.forward(speed)
        _set_motion_command(speed, speed)
    except Exception as e:
        run_force_stop_async(f"forward exception: {e}")
        return bad(f"forward failed: {e}", code=500)
    return jsonify({"ok": True, "blocked": False, "speed": speed})


@app.route("/api/backward", methods=["POST"])
def api_backward():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/backward")
    _stop_wall_follower_if_active("manual /api/backward")
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        if lidar_monitor is not None:
            s = lidar_monitor.status()
            if s["rear_blocked"]:
                ctrl.stop_smooth()
                _clear_motion_command()
                print(f"[LIDAR SAFETY] blocked backward speed={speed}")
                return jsonify({"ok": True, "blocked": True, "direction": "backward", "speed": 0, "lidar": s})
        ctrl.backward(speed)
        _set_motion_command(-speed, -speed)
    except Exception as e:
        run_force_stop_async(f"backward exception: {e}")
        return bad(f"backward failed: {e}", code=500)
    return jsonify({"ok": True, "blocked": False, "speed": speed})


@app.route("/api/turn_left", methods=["POST"])
def api_turn_left():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/turn_left")
    _stop_wall_follower_if_active("manual /api/turn_left")
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.turn_left(speed)
        _set_motion_command(-speed, speed)
    except Exception as e:
        run_force_stop_async(f"turn_left exception: {e}")
        return bad(f"turn_left failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/turn_right", methods=["POST"])
def api_turn_right():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/turn_right")
    _stop_wall_follower_if_active("manual /api/turn_right")
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.turn_right(speed)
        _set_motion_command(speed, -speed)
    except Exception as e:
        run_force_stop_async(f"turn_right exception: {e}")
        return bad(f"turn_right failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual /api/stop")
    _stop_wall_follower_if_active("manual /api/stop")
    try:
        if action_runner is not None:
            action_runner.interrupt()
        if dialog_engine is not None:
            dialog_engine.reset_to_idle("manual stop")
        ctrl.stop()
        _clear_motion_command()
    except Exception as e:
        run_force_stop_async(f"stop exception: {e}")
        return bad(f"stop failed: {e}", code=500)
    return jsonify({"ok": True})


@app.route("/api/center", methods=["POST"])
def api_center():
    touch_heartbeat()
    _stop_wall_follower_if_active("manual /api/center")
    try:
        ctrl.center_pose()
        _clear_motion_command()
    except Exception as e:
        run_force_stop_async(f"center exception: {e}")
        return bad(f"center failed: {e}", code=500)
    return jsonify({"ok": True})


@app.route("/api/geeked", methods=["POST"])
def api_geeked():
    touch_heartbeat()
    _stop_wall_follower_if_active("manual /api/geeked")
    try:
        _clear_motion_command()
        ctrl.stop()

        limits = getattr(ctrl, "SERVO_LIMITS", {})

        def rand_joint(name, move_fn):
            lo, hi = limits.get(name, (2000, 8000))
            # Keep values on coarse steps to avoid jittery tiny moves.
            value = random.randrange(lo, hi + 1, 50)
            move_fn(value)
            return value

        values = {
            "head_pan": rand_joint("head_pan", ctrl.head_pan),
            "head_tilt": rand_joint("head_tilt", ctrl.head_tilt),
            "waist": rand_joint("waist", ctrl.waist),
            "right_shoulder_ud": rand_joint("right_shoulder_ud", ctrl.right_shoulder_ud),
            "right_shoulder_yaw": rand_joint("right_shoulder_yaw", ctrl.right_shoulder_yaw),
            "right_elbow_ud": rand_joint("right_elbow_ud", ctrl.right_elbow_ud),
            "right_wrist_ud": rand_joint("right_wrist_ud", ctrl.right_wrist_ud),
            "right_wrist_rot": rand_joint("right_wrist_rot", ctrl.right_wrist_rot),
            "right_hand_pinch": rand_joint("right_hand_pinch", ctrl.right_hand_pinch),
            "left_shoulder_ud": rand_joint("left_shoulder_ud", ctrl.left_shoulder_ud),
            "left_shoulder_yaw": rand_joint("left_shoulder_yaw", ctrl.left_shoulder_yaw),
            "left_elbow_ud": rand_joint("left_elbow_ud", ctrl.left_elbow_ud),
            "left_wrist_ud": rand_joint("left_wrist_ud", ctrl.left_wrist_ud),
            "left_wrist_rot": rand_joint("left_wrist_rot", ctrl.left_wrist_rot),
            "left_hand_pinch": rand_joint("left_hand_pinch", ctrl.left_hand_pinch),
        }
    except Exception as e:
        run_force_stop_async(f"geeked exception: {e}")
        return bad(f"geeked failed: {e}", code=500)
    return jsonify({"ok": True, "values": values})


@app.route("/api/all_neutral", methods=["POST"])
def api_all_neutral():
    touch_heartbeat()
    _stop_wall_follower_if_active("manual /api/all_neutral")
    try:
        ctrl.all_servos_neutral()
        _clear_motion_command()
        values = dict(ctrl.robot.SERVO_NEUTRALS)
    except Exception as e:
        run_force_stop_async(f"all_neutral exception: {e}")
        return bad(f"all_neutral failed: {e}", code=500)
    return jsonify({"ok": True, "values": values})


@app.route("/api/six_seven", methods=["POST"])
def api_six_seven():
    touch_heartbeat()
    _stop_wall_follower_if_active("manual /api/six_seven")
    try:
        ctrl.all_servos_neutral()
        _clear_motion_command()
        phrase = "six seven six seven six seven six seven six seven"
        speak_async(phrase)

        l0 = ctrl.robot.servo_neutral("left_elbow_ud")
        r0 = ctrl.robot.servo_neutral("right_elbow_ud")
        l_up = ctrl._servo_clamp("left_elbow_ud", l0 + 700)
        r_up = ctrl._servo_clamp("right_elbow_ud", r0 + 700)

        hold_s = 0.22
        for _ in range(11):
            # Left up, right down.
            ctrl.left_elbow_ud(l_up)
            ctrl.right_elbow_ud(r0)
            time.sleep(hold_s)

            # Right up, left down.
            ctrl.left_elbow_ud(l0)
            ctrl.right_elbow_ud(r_up)
            time.sleep(hold_s)

        # Return to neutral.
        ctrl.left_elbow_ud(l0)
        ctrl.right_elbow_ud(r0)
    except Exception as e:
        run_force_stop_async(f"six_seven exception: {e}")
        return bad(f"six_seven failed: {e}", code=500)
    return jsonify({"ok": True})


# =========================
# HEAD / WAIST API
# =========================

@app.route("/api/head_pan", methods=["POST"])
def api_head_pan():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    try:
        ctrl.head_pan(v)
    except Exception as e:
        run_force_stop_async(f"head_pan exception: {e}")
        return bad(f"head_pan failed: {e}", code=500)
    return jsonify({"ok": True, "value": v})


@app.route("/api/head_tilt", methods=["POST"])
def api_head_tilt():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    try:
        ctrl.head_tilt(v)
    except Exception as e:
        run_force_stop_async(f"head_tilt exception: {e}")
        return bad(f"head_tilt failed: {e}", code=500)
    return jsonify({"ok": True, "value": v})


@app.route("/api/waist", methods=["POST"])
def api_waist():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    try:
        ctrl.waist(v)
    except Exception as e:
        run_force_stop_async(f"waist exception: {e}")
        return bad(f"waist failed: {e}", code=500)
    return jsonify({"ok": True, "value": v})


def _api_arm_joint(move_fn, joint_name):
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    try:
        move_fn(v)
    except Exception as e:
        run_force_stop_async(f"{joint_name} exception: {e}")
        return bad(f"{joint_name} failed: {e}", code=500)
    return jsonify({"ok": True, "joint": joint_name, "value": v})


@app.route("/api/right_shoulder_ud", methods=["POST"])
def api_right_shoulder_ud():
    return _api_arm_joint(ctrl.right_shoulder_ud, "right_shoulder_ud")


@app.route("/api/right_shoulder_yaw", methods=["POST"])
def api_right_shoulder_yaw():
    return _api_arm_joint(ctrl.right_shoulder_yaw, "right_shoulder_yaw")


@app.route("/api/right_elbow_ud", methods=["POST"])
def api_right_elbow_ud():
    return _api_arm_joint(ctrl.right_elbow_ud, "right_elbow_ud")


@app.route("/api/right_wrist_ud", methods=["POST"])
def api_right_wrist_ud():
    return _api_arm_joint(ctrl.right_wrist_ud, "right_wrist_ud")


@app.route("/api/right_wrist_rot", methods=["POST"])
def api_right_wrist_rot():
    return _api_arm_joint(ctrl.right_wrist_rot, "right_wrist_rot")


@app.route("/api/right_hand_pinch", methods=["POST"])
def api_right_hand_pinch():
    return _api_arm_joint(ctrl.right_hand_pinch, "right_hand_pinch")


@app.route("/api/left_wrist_rot", methods=["POST"])
def api_left_wrist_rot():
    return _api_arm_joint(ctrl.left_wrist_rot, "left_wrist_rot")


@app.route("/api/left_shoulder_ud", methods=["POST"])
def api_left_shoulder_ud():
    return _api_arm_joint(ctrl.left_shoulder_ud, "left_shoulder_ud")


@app.route("/api/left_shoulder_yaw", methods=["POST"])
def api_left_shoulder_yaw():
    return _api_arm_joint(ctrl.left_shoulder_yaw, "left_shoulder_yaw")


@app.route("/api/left_elbow_ud", methods=["POST"])
def api_left_elbow_ud():
    return _api_arm_joint(ctrl.left_elbow_ud, "left_elbow_ud")


@app.route("/api/left_wrist_ud", methods=["POST"])
def api_left_wrist_ud():
    return _api_arm_joint(ctrl.left_wrist_ud, "left_wrist_ud")


@app.route("/api/left_hand_pinch", methods=["POST"])
def api_left_hand_pinch():
    return _api_arm_joint(ctrl.left_hand_pinch, "left_hand_pinch")


# =========================
# VOICE / TTS API
# =========================

@app.route("/api/speak_text", methods=["POST"])
def api_speak_text():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    text = data.get("text", "")

    if not isinstance(text, str):
        return bad("text must be a string")

    text = sanitize_tts(text)
    if not text:
        return bad("text is empty")

    if len(text) > 140:
        return bad("text too long (max 140 characters)")

    speak_async(text)
    return jsonify({"ok": True, "text": text})


@app.route("/api/dialog_state", methods=["GET"])
def api_dialog_state():
    if dialog_engine is None:
        return jsonify({"ok": False, "error": "dialog engine not configured"}), 500
    return jsonify(
        {
            "ok": True,
            "state": get_dialog_state(),
            "scope_depth": dialog_engine.current_scope_depth(),
            "unmatched_in_scope": dialog_engine.unmatched_in_scope,
            "fatal_errors": dialog_engine.has_fatal_errors(),
            "error_count": len(dialog_engine.errors),
        }
    )


@app.route("/api/dialog_last", methods=["GET"])
def api_dialog_last():
    return jsonify({"ok": True, "dialog": _get_dialog_event()})


@app.route("/api/lidar_status", methods=["GET"])
def api_lidar_status():
    if lidar_monitor is None:
        return jsonify(
            {
                "ok": False,
                "error": "lidar monitor not configured",
                "import_error": LIDAR_IMPORT_ERROR,
            }
        ), 500
    return jsonify({"ok": True, "lidar": lidar_monitor.status()})


@app.route("/api/final/start", methods=["POST"])
def api_final_start():
    touch_heartbeat()
    global _final_demo_thread
    if lidar_monitor is None:
        return bad("lidar monitor not configured", code=500)

    if _final_demo_is_active():
        _cancel_final_demo("restarting final demo")

    _final_demo_cancel.clear()
    try:
        _stop_wall_follower_if_active("final demo start")
        if action_runner is not None:
            action_runner.interrupt()
        ctrl.stop()
        _clear_motion_command()
    except Exception as ex:
        return bad(f"final demo start failed: {ex}", code=500)

    _reset_final_demo_status("Final demo starting.")
    with _final_demo_lock:
        _final_demo_status.update(
            {
                "active": True,
                "state": "STARTING",
                "message": "Final demo starting.",
                "started_at": time.time(),
                "updated_at": time.time(),
            }
        )
    _final_demo_thread = threading.Thread(target=_final_demo_greet_worker, daemon=True)
    _final_demo_thread.start()
    return jsonify({"ok": True, "final": _get_final_demo_status()})


@app.route("/api/final/stop", methods=["POST"])
def api_final_stop():
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("final demo stopped")
    else:
        _reset_final_demo_status("Final demo stopped.")
    return jsonify({"ok": True, "final": _get_final_demo_status()})


@app.route("/api/final/status", methods=["GET"])
def api_final_status():
    lidar_status = lidar_monitor.status() if lidar_monitor is not None else None
    wall_status = wall_follower.status() if wall_follower is not None else {"active": False}
    return jsonify(
        {
            "ok": True,
            "final": _get_final_demo_status(),
            "lidar": lidar_status,
            "wall_follow": wall_status,
        }
    )


@app.route("/api/wall_follow/start", methods=["POST"])
def api_wall_follow_start():
    global wall_follower
    touch_heartbeat()
    if _final_demo_is_active():
        _cancel_final_demo("manual wall follow start")
    if lidar_monitor is None:
        return bad("lidar monitor not configured", code=500)
    if wall_follower is None:
        wall_follower = WallFollower(ctrl, lidar_monitor)

    try:
        data = request.get_json(silent=True) or {}
        side = str(data.get("side", "right")).lower().strip()
        profile = str(data.get("profile", "final")).lower().strip()
        cfg = get_wall_follow_profile(profile, side=side)

        if "set_speed" in data and "base_speed" not in data:
            data["base_speed"] = data["set_speed"]
        if "delta" in data and "correction_band" not in data:
            data["correction_band"] = data["delta"]

        start_kwargs = {}
        for key in WALL_FOLLOW_CONFIG_FIELDS:
            if key not in cfg:
                continue
            raw = data.get(key, cfg[key])
            default = cfg[key]
            if key == "side":
                start_kwargs[key] = str(raw).lower().strip()
            elif isinstance(default, float):
                start_kwargs[key] = float(raw)
            else:
                start_kwargs[key] = int(raw)

        start_kwargs["base_speed"] = max(1000, int(start_kwargs["base_speed"]))
        _clear_motion_command()
        wall_follower.start(**start_kwargs)
    except Exception as e:
        return bad(f"wall follow start failed: {e}", code=400)

    return jsonify({"ok": True, "wall_follow": wall_follower.status()})


@app.route("/api/wall_follow/profiles", methods=["GET"])
def api_wall_follow_profiles():
    return jsonify({"ok": True, "profiles": WALL_FOLLOW_PROFILES})


@app.route("/api/wall_follow/stop", methods=["POST"])
def api_wall_follow_stop():
    touch_heartbeat()
    global wall_follower
    try:
        if wall_follower is not None:
            wall_follower.stop()
        _clear_motion_command()
    except Exception as e:
        return bad(f"wall follow stop failed: {e}", code=500)
    return jsonify({"ok": True, "wall_follow": wall_follower.status() if wall_follower else {"active": False}})


@app.route("/api/wall_follow/status", methods=["GET"])
def api_wall_follow_status():
    if wall_follower is None:
        return jsonify({"ok": True, "wall_follow": {"active": False, "last_state": "NOT_STARTED"}})
    return jsonify({"ok": True, "wall_follow": wall_follower.status()})


@app.route("/api/dialog_input", methods=["POST"])
def api_dialog_input():
    touch_heartbeat()
    if dialog_engine is None:
        return bad("dialog engine not configured", code=500)

    data = request.get_json(silent=True) or {}
    text = data.get("text", "")
    source = data.get("source", "browser")
    if not isinstance(text, str):
        return bad("text must be a string")
    text = sanitize_tts(text)
    if not text:
        return bad("text is empty")
    destination = _destination_from_text(text, data.get("destination"))
    final_destination_waiting = bool(destination and _final_demo_is_waiting_for_destination())

    if _final_demo_is_active() and not final_destination_waiting:
        response = {
            "ok": True,
            "input": text,
            "matched": False,
            "reply": "",
            "actions": [],
            "state": get_dialog_state(),
            "scope_depth": dialog_engine.current_scope_depth(),
            "source": source,
            "destination": destination,
            "final_triggered": False,
            "ignored": True,
            "ignore_reason": "final demo already navigating",
        }
        _store_dialog_event(response)
        print(f"[FINAL] ignored speech while navigating: {text!r}")
        return jsonify(response)

    _stop_wall_follower_if_active("dialog input")

    # Wheel deadman: any non-final dialog input immediately stops wheel motion,
    # even if wheels were started by manual drive controls.
    try:
        ctrl.stop()
        _clear_motion_command()
    except Exception as ex:
        print(f"[DIALOG] deadman stop failed: {ex}")

    with dialog_lock:
        result = dialog_engine.handle_input(text)

    if not result.get("ok", False):
        return jsonify(result), 400

    if result.get("interrupt", False):
        if action_runner is not None:
            action_runner.interrupt()
        try:
            ctrl.stop()
            _clear_motion_command()
        except Exception as ex:
            print(f"[DIALOG] stop failed on interrupt: {ex}")

    speak_text = result.get("speak_text", "")
    if isinstance(speak_text, str) and speak_text and not final_destination_waiting:
        speak_async(speak_text)

    actions = result.get("actions", [])
    if isinstance(actions, list) and actions and action_runner is not None:
        # Set immediately so API/UI shows EXEC_ACTIONS without worker timing delay.
        set_dialog_state("EXEC_ACTIONS")
        action_runner.enqueue(actions)

    final_triggered = False
    if final_destination_waiting:
        final_triggered = True
        _set_final_demo_state(
            "NAV_QUEUED",
            f"Destination heard: {destination}. Starting navigation.",
            destination=destination,
            active=True,
        )
        _final_demo_cancel.clear()
        threading.Thread(
            target=_final_demo_navigation_worker,
            args=(destination,),
            daemon=True,
        ).start()

    response = {
        "ok": True,
        "input": text,
        "matched": result.get("matched", False),
        "reply": speak_text,
        "actions": actions,
        "state": get_dialog_state(),
        "scope_depth": dialog_engine.current_scope_depth(),
        "source": source,
        "destination": destination,
        "final_triggered": final_triggered,
    }
    _store_dialog_event(response)
    return jsonify(response)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CSCI 455 Robot Flask Server + Dialog Engine")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument(
        "--dialog-script",
        default=os.path.join(os.path.dirname(__file__), "testDialogFileForPractice.txt"),
        help="Path to dialog script file",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for deterministic dialog output choices",
    )
    parser.add_argument(
        "--lidar-port",
        default=os.getenv("LIDAR_PORT", "auto"),
        help="Lidar serial device path or 'auto'",
    )
    parser.add_argument(
        "--lidar-stop-mm",
        type=int,
        default=int(os.getenv("LIDAR_STOP_MM", "800")),
        help="Stop distance threshold in mm",
    )
    args = parser.parse_args()

    # Start lidar monitor before serving requests (if module is available).
    if LidarSafetyMonitor is not None:
        lidar_monitor = LidarSafetyMonitor(
            port=args.lidar_port,
            stop_mm=args.lidar_stop_mm,
            clear_scans_required=LIDAR_CLEAR_SCANS_REQUIRED,
        )
        lidar_monitor.start()
        wall_follower = WallFollower(ctrl, lidar_monitor)
        print(
            f"[LIDAR] monitor started port={args.lidar_port} stop_mm={args.lidar_stop_mm} "
            f"clear_scans_required={LIDAR_CLEAR_SCANS_REQUIRED}"
        )
    else:
        print(f"[LIDAR WARN] lidar monitor disabled (import failed): {LIDAR_IMPORT_ERROR}")

    configure_dialog_engine(args.dialog_script, args.seed)
    PORT = args.port
    print(f"[FLASK] starting on 0.0.0.0:{PORT}")
    print(f"[FLASK] open http://<robot-ip>:{PORT}/ from your laptop")
    app.run(host="0.0.0.0", port=PORT, debug=False, request_handler=QuietHandler)
