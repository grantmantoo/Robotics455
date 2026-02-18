from flask import Flask, request, jsonify, render_template
from robot_control import RobotControl

import logging
from werkzeug.serving import WSGIRequestHandler
import threading
import subprocess
import re
import time
import os

class QuietHandler(WSGIRequestHandler):
    def log_request(self, code='-', size='-'):
        # Suppress heartbeat spam
        if self.path.startswith("/api/heartbeat"):
            return
        super().log_request(code, size)

app = Flask(__name__)

# One shared controller instance for the server
ctrl = RobotControl(port="/dev/ttyACM0", device=0x0C)

def bad(msg, code=400):
    return jsonify({"ok": False, "error": msg}), code


# =========================
# Watchdog / Force Stop
# =========================

HEARTBEAT_TIMEOUT_S = 1.0   # if we haven't heard from browser in this many seconds -> force stop
WATCHDOG_PERIOD_S = 0.1     # how often watchdog checks

_last_heartbeat = time.time()
_force_stop_lock = threading.Lock()
_force_stop_running = False

def touch_heartbeat():
    global _last_heartbeat
    _last_heartbeat = time.time()

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

            # Prefer your dedicated script (exactly what you asked for)
            script_path = os.path.join(os.path.dirname(__file__), "force_stop.py")
            if os.path.exists(script_path):
                subprocess.run(["python3", script_path], check=False)
            else:
                print("[WATCHDOG] force_stop.py not found рядом с flaskServer.py; falling back to ctrl.stop()")
                try:
                    ctrl.stop()
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
    global _last_heartbeat
    timed_out = False  # local state: have we already triggered for the current outage?

    while True:
        time.sleep(WATCHDOG_PERIOD_S)
        age = time.time() - _last_heartbeat

        if age > HEARTBEAT_TIMEOUT_S:
            # Only trigger once per outage
            if not timed_out:
                run_force_stop_async(f"heartbeat timeout ({age:.2f}s > {HEARTBEAT_TIMEOUT_S}s)")
                timed_out = True
        else:
            # Heartbeat is healthy again -> allow future triggers
            timed_out = False


# start watchdog thread
threading.Thread(target=watchdog_loop, daemon=True).start()


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
        subprocess.run(["espeak-ng", "-s", "165", "-v", "en-us", text], check=False)
    threading.Thread(target=run, daemon=True).start()


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
    run_force_stop_async("manual /api/force_stop")
    return jsonify({"ok": True})


# =========================
# DRIVE API
# =========================

@app.route("/api/drive", methods=["POST"])
def api_drive():
    touch_heartbeat()  # treat commands as “activity” too

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
        ctrl.drive(left, right)
    except Exception as e:
        run_force_stop_async(f"drive exception: {e}")
        return bad(f"drive failed: {e}", code=500)

    return jsonify({"ok": True, "left": left, "right": right})


@app.route("/api/forward", methods=["POST"])
def api_forward():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.forward(speed)
    except Exception as e:
        run_force_stop_async(f"forward exception: {e}")
        return bad(f"forward failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/backward", methods=["POST"])
def api_backward():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.backward(speed)
    except Exception as e:
        run_force_stop_async(f"backward exception: {e}")
        return bad(f"backward failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/turn_left", methods=["POST"])
def api_turn_left():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.turn_left(speed)
    except Exception as e:
        run_force_stop_async(f"turn_left exception: {e}")
        return bad(f"turn_left failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/turn_right", methods=["POST"])
def api_turn_right():
    touch_heartbeat()
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    try:
        ctrl.turn_right(speed)
    except Exception as e:
        run_force_stop_async(f"turn_right exception: {e}")
        return bad(f"turn_right failed: {e}", code=500)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    touch_heartbeat()
    try:
        ctrl.stop()
    except Exception as e:
        run_force_stop_async(f"stop exception: {e}")
        return bad(f"stop failed: {e}", code=500)
    return jsonify({"ok": True})


@app.route("/api/center", methods=["POST"])
def api_center():
    touch_heartbeat()
    try:
        ctrl.stop()
        ctrl.head_pan(5000)
        ctrl.head_tilt(5000)
        ctrl.waist(5000)
    except Exception as e:
        run_force_stop_async(f"center exception: {e}")
        return bad(f"center failed: {e}", code=500)
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


if __name__ == "__main__":
    PORT = 5000
    print(f"[FLASK] starting on 0.0.0.0:{PORT}")
    print(f"[FLASK] open http://<robot-ip>:{PORT}/ from your laptop")
    app.run(host="0.0.0.0", port=PORT, debug=False, request_handler=QuietHandler)