"""Flask control dashboard for the desky arm.

Enter a target (x, y, z) position or a per-joint servo degree in the browser
and drive the real actuators through hardware.util.ArmController. If no
hardware is connected — no serial device, missing/misconfigured .env, or even
the dynamixel_sdk/python-dotenv packages not installed — the dashboard still
starts. It just reports "no hardware connected" instead of controlling
anything.

Also serves /mobile: a page meant to be opened on the phone mounted on the
arm's end-effector. It asks for camera+microphone permission and streams JPEG
camera frames to the server over a WebSocket (/ws/camera). The main dashboard
(/) polls the latest frame back over HTTP to preview it. The same WebSocket
also carries recorded voice-question clips (WebM/Opus) from the mic button —
the server tells the two apart by content (JPEG's SOI marker vs WebM's EBML
header), transcribes voice clips via Gemini, and sends the transcript back
over that same connection.

Also exposes /api/ask: a Gemini-backed chat/summary endpoint. If GEMINI_API_KEY
isn't set (or the google-genai package isn't installed), that route reports
"not configured" instead of crashing the app, same as the hardware fallback.

Run from the repository root:
    python -m webapp.app
"""

import json
import os
import numpy as np
import threading
import time
import cv2

from flask import Flask, Response, jsonify, render_template, request
from flask_sock import Sock

from kinematics.urdf_loader import load_arm
from logger import Logger

try:
    # Optional: only needed to read GEMINI_API_KEY (and hardware's .env vars)
    # from a .env file. Without it, those must already be real env vars.
    from dotenv import load_dotenv

    load_dotenv()
except ImportError:
    pass

Logger.enabled = True

app = Flask(__name__)
sock = Sock(app)

arm_ctrl = None
hardware_error = None
try:
    # Imported inside the try block: hardware.controller imports dynamixel_sdk
    # at module load time, so even that missing package must count as "no
    # hardware connected" rather than crashing the whole app.
    from hardware.controller import Controller
    from hardware.actuator import Actuator, ArmController

    controller = Controller()
    actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
    arm_ctrl = ArmController(actuators)
    Logger.log("WEBAPP", "Hardware connected")
except Exception as e:
    hardware_error = str(e)
    Logger.log("WEBAPP", f"No hardware connected: {hardware_error}")

# Joint list is needed to render the per-joint controls even with no hardware.
arm = arm_ctrl.arm if arm_ctrl is not None else load_arm()

GEMINI_MODEL = "gemini-flash-latest"  # fast, generous free tier
GEMINI_CHAT_INSTRUCTION = "너는 친절한 한국어 AI 비서야. 자연스럽게 대화해줘."
GEMINI_SUMMARY_INSTRUCTION = "너는 문서를 간결하게 요약해주는 비서야. 핵심만 한국어로 정리해줘."

gemini_client = None
gemini_error = None
try:
    from google import genai
    from google.genai import types as genai_types

    gemini_client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])
    Logger.log("GEMINI", f"Gemini client configured (model={GEMINI_MODEL})")
except Exception as e:
    gemini_error = str(e)
    Logger.log("GEMINI", f"Gemini not configured: {gemini_error}")

# Latest JPEG frame received from the phone's camera over /ws/camera, plus a
# few counters for the dashboard's status display. Guarded by camera_lock
# since the WebSocket handler runs on its own request thread.
camera_lock = threading.Lock()
camera_state = {"frame": None, "frame_time": None, "frame_count": 0, "clients": 0}


@app.route("/")
def index():
    return render_template(
        "index.html",
        joint_ids=[joint.id for joint in arm.joints],
        hardware_connected=arm_ctrl is not None,
        hardware_error=hardware_error,
    )

@app.route("/mobile")
def mobile():
    return render_template(
        "mobile.html",
        joint_ids=[joint.id for joint in arm.joints],
        hardware_connected=arm_ctrl is not None,
        hardware_error=hardware_error,
        gemini_configured=gemini_client is not None,
        gemini_error=gemini_error,
    )

@app.route("/api/status")
def status():
    if arm_ctrl is None:
        return jsonify({"connected": False, "position": None, "error": hardware_error})
    return jsonify({"connected": True, "position": arm_ctrl.get_position()})


@app.route("/api/ask", methods=["POST"])
def ask():
    if gemini_client is None:
        return jsonify({"error": f"Gemini not configured: {gemini_error}"}), 503

    data = request.get_json(force=True)
    text = data.get("text") if data else None
    mode = data.get("mode", "chat") if data else "chat"
    if not text:
        return jsonify({"error": "text is required"}), 400

    system = GEMINI_SUMMARY_INSTRUCTION if mode == "summary" else GEMINI_CHAT_INSTRUCTION

    Logger.log("GEMINI", f"ask request: mode={mode}, text length={len(text)}")
    try:
        response = gemini_client.models.generate_content(
            model=GEMINI_MODEL,
            contents=text,
            config=genai_types.GenerateContentConfig(
                system_instruction=system,
                max_output_tokens=1024,
            ),
        )
        return jsonify({"answer": response.text})
    except Exception as e:
        Logger.log("GEMINI", f"ask failed: {e}")
        return jsonify({"error": str(e)}), 502


WEBM_EBML_HEADER = b"\x1a\x45\xdf\xa3"

STT_INSTRUCTION = "이 오디오를 한국어 텍스트로 정확히 받아써줘. 설명 없이 텍스트만 출력해."


def _transcribe(audio_bytes):
    """Run one recorded voice clip through Gemini and return the transcript
    text. Raises on failure — callers turn that into a {"type": "error"} reply."""
    audio_part = genai_types.Part.from_bytes(data=audio_bytes, mime_type="audio/webm")
    response = gemini_client.models.generate_content(
        model=GEMINI_MODEL,
        contents=[audio_part, STT_INSTRUCTION],
    )
    return response.text


@sock.route("/ws/camera")
def ws_camera(ws):
    """Receives JPEG camera frames and recorded voice-question clips from
    /mobile, both over this one connection.

    Each incoming binary message is either one complete JPEG frame (canvas
    snapshot — see mobile.html's captureLoop) or one complete WebM/Opus clip
    (MediaRecorder output from the mic button). They're told apart by their
    magic bytes rather than a custom header, so the client doesn't need any
    extra framing beyond "send the whole blob".
    """
    with camera_lock:
        camera_state["clients"] += 1
    Logger.log("CAMERA", "Mobile client connected")
    try:
        while True:
            data = ws.receive()
            if data is None:
                break
            if not isinstance(data, (bytes, bytearray)):
                continue
            data = bytes(data)

            if data[:4] == WEBM_EBML_HEADER:
                Logger.log("STT", f"Received voice clip ({len(data)} bytes)")
                if gemini_client is None:
                    ws.send(json.dumps({"type": "error", "error": f"Gemini not configured: {gemini_error}"}))
                    continue
                try:
                    transcript = _transcribe(data)
                    Logger.log("STT", f"Transcript: {transcript!r}")
                    ws.send(json.dumps({"type": "transcript", "text": transcript}))
                except Exception as e:
                    Logger.log("STT", f"Transcription failed: {e}")
                    ws.send(json.dumps({"type": "error", "error": str(e)}))
                continue

            with camera_lock:
                camera_state["frame"] = data
                camera_state["frame_time"] = time.time()
                camera_state["frame_count"] += 1
    finally:
        with camera_lock:
            camera_state["clients"] -= 1
        Logger.log("CAMERA", "Mobile client disconnected")


@app.route("/api/camera/latest.jpg")
def camera_latest_frame():
    with camera_lock:
        frame = camera_state["frame"]
    if frame is None:
        return jsonify({"error": "no camera frame received yet"}), 404
    return Response(frame, mimetype="image/jpeg")


@app.route("/api/camera/status")
def camera_status():
    with camera_lock:
        frame_time = camera_state["frame_time"]
        frame_count = camera_state["frame_count"]
        clients = camera_state["clients"]
    return jsonify({
        "streaming": frame_time is not None,
        "clients": clients,
        "frame_count": frame_count,
        "age_seconds": (time.time() - frame_time) if frame_time is not None else None,
    })


@app.route("/api/goto_position", methods=["POST"])
def goto_position():
    if arm_ctrl is None:
        return jsonify({"error": "no hardware connected"}), 503

    data = request.get_json(force=True)
    try:
        target = (float(data["x"]), float(data["y"]), float(data["z"]))
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "x, y, z must be numbers"}), 400

    Logger.log("WEBAPP", f"goto_position request: target={target}")
    q, converged = arm_ctrl.goto_position(target)
    servo_deg = arm_ctrl.arm.q_to_servo_deg(q) if converged else None
    return jsonify({"converged": converged, "servo_deg": servo_deg})


@app.route("/api/goto_joint", methods=["POST"])
def goto_joint():
    if arm_ctrl is None:
        return jsonify({"error": "no hardware connected"}), 503

    data = request.get_json(force=True)
    try:
        joint_id = int(data["id"])
        degree = float(data["degree"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "id and degree must be numbers"}), 400

    actuator = next((a for a in arm_ctrl.actuators if a.id == joint_id), None)
    if actuator is None:
        return jsonify({"error": f"no actuator with id {joint_id}"}), 404

    Logger.log("WEBAPP", f"goto_joint request: id={joint_id} degree={degree}")
    try:
        actuator.goto(degree)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    return jsonify({"ok": True})


def run():
    """Serve the dashboard and show a local preview window of the phone's
    camera feed. Shared by `python -m webapp.app` and `python main.py` so
    both behave identically.

    debug=False: the Flask reloader re-imports this module in a subprocess,
    which would open the serial port twice.
    threaded=True: the /ws/camera WebSocket connection stays open for the
    whole mobile session, so the dev server needs a thread per request to
    keep serving the dashboard's HTTP polling at the same time.
    ssl_context="adhoc": getUserMedia (camera/mic) only works in a secure
    context. A phone loading /mobile over the LAN IP needs HTTPS — a
    self-signed cert is generated on the fly (pyOpenSSL). The browser will
    show an untrusted-certificate warning once; accept it to proceed.

    app.run() blocks forever, so it runs on a background thread here —
    otherwise the cv2.imshow loop below would never execute. cv2's window
    calls stay on the main thread since OpenCV's HighGUI requires that on
    macOS (imshow/waitKey from a non-main thread silently do nothing there).
    """
    server_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=8000, debug=False, threaded=True, ssl_context="adhoc"),
        daemon=True,
    )
    server_thread.start()

    Logger.log("CAMERA", "Press 'q' in the camera window (or Ctrl+C here) to quit")
    last_frame_count = 0
    try:
        while True:
            with camera_lock:
                frame_bytes = camera_state["frame"]
                frame_count = camera_state["frame_count"]
            # Only decode+show on a genuinely new frame — the phone only
            # sends ~5 fps, but this loop spins far faster (waitKey(1) is a
            # ~1ms cap, not a guarantee), so without this check it would
            # needlessly re-decode and re-display the same bytes every spin.
            if frame_bytes is not None and frame_count != last_frame_count:
                last_frame_count = frame_count
                frame = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    cv2.imshow("Mobile camera", frame)
            # waitKey both drives HighGUI's event loop (without it the window
            # never actually renders/refreshes) and lets 'q'/Esc quit.
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
