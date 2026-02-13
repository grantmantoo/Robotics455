# flaskServer.py
from flask import Flask, request, jsonify, render_template
from robot_control import RobotControl

import threading
import subprocess
import re

app = Flask(__name__)

# One shared controller instance for the server
ctrl = RobotControl(port="/dev/ttyACM0", device=0x0C)


def bad(msg, code=400):
    return jsonify({"ok": False, "error": msg}), code


# -------- TTS helpers --------
def sanitize_tts(text: str) -> str:
    # remove control chars, collapse whitespace
    text = re.sub(r"[\x00-\x1F\x7F]", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text


def speak_async(text: str):
    def run():
        # Offline TTS on Pi
        # -s = speed (words per minute)
        # -v = voice
        subprocess.run(["espeak-ng", "-s", "165", "-v", "en-us", text], check=False)

    threading.Thread(target=run, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


# ---------- DRIVE API ----------
@app.route("/api/drive", methods=["POST"])
def api_drive():
    data = request.get_json(silent=True) or {}
    if "left" not in data or "right" not in data:
        return bad("Missing 'left' or 'right'")

    try:
        left = int(data["left"])
        right = int(data["right"])
    except (ValueError, TypeError):
        return bad("left/right must be integers")

    # sanity limits (RobotControl also clamps, but this rejects obviously bad input)
    if abs(left) > 3000 or abs(right) > 3000:
        return bad("left/right out of allowed range")

    ctrl.drive(left, right)
    return jsonify({"ok": True, "left": left, "right": right})


@app.route("/api/forward", methods=["POST"])
def api_forward():
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    ctrl.forward(speed)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/backward", methods=["POST"])
def api_backward():
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    ctrl.backward(speed)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/turn_left", methods=["POST"])
def api_turn_left():
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    ctrl.turn_left(speed)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/turn_right", methods=["POST"])
def api_turn_right():
    data = request.get_json(silent=True) or {}
    try:
        speed = int(data.get("speed", 800))
    except (ValueError, TypeError):
        return bad("speed must be int")
    ctrl.turn_right(speed)
    return jsonify({"ok": True, "speed": speed})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    ctrl.stop()
    return jsonify({"ok": True})


@app.route("/api/center", methods=["POST"])
def api_center():
    ctrl.stop()
    ctrl.head_pan(5000)
    ctrl.head_tilt(5000)
    ctrl.waist(5000)
    return jsonify({"ok": True})


# ---------- HEAD / WAIST API ----------
@app.route("/api/head_pan", methods=["POST"])
def api_head_pan():
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    ctrl.head_pan(v)
    return jsonify({"ok": True, "value": v})


@app.route("/api/head_tilt", methods=["POST"])
def api_head_tilt():
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    ctrl.head_tilt(v)
    return jsonify({"ok": True, "value": v})


@app.route("/api/waist", methods=["POST"])
def api_waist():
    data = request.get_json(silent=True) or {}
    if "value" not in data:
        return bad("Missing 'value'")
    try:
        v = int(data["value"])
    except (ValueError, TypeError):
        return bad("value must be int")
    ctrl.waist(v)
    return jsonify({"ok": True, "value": v})


# ---------- VOICE / TTS API ----------
@app.route("/api/speak_text", methods=["POST"])
def api_speak_text():
    data = request.get_json(silent=True) or {}
    text = data.get("text", "")

    if not isinstance(text, str):
        return bad("text must be a string")

    text = sanitize_tts(text)

    if not text:
        return bad("text is empty")

    # Prevent abuse / long blocking speech
    if len(text) > 140:
        return bad("text too long (max 140 characters)")

    speak_async(text)
    return jsonify({"ok": True, "text": text})


if __name__ == "__main__":
    PORT = 5000
    print(f"[FLASK] starting on 0.0.0.0:{PORT}")
    print(f"[FLASK] open http://<robot-ip>:{PORT}/ from your laptop")
    app.run(host="0.0.0.0", port=PORT, debug=False)