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

import mediapipe as mp
import numpy as np
import threading
import cv2
import io
import math
import time

import matplotlib
matplotlib.use("Agg")  # headless: render arm previews to PNG, no display/window
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from flask import Flask, Response, jsonify, render_template, request
from flask_sock import Sock

from kinematics.urdf_loader import load_arm, _DEFAULT_URDF_PATH
from kinematics.simulate import parse_urdf, draw_pose, draw_points, workspace_bounds
from logger import Logger
from src.api.gemini import Gemini
from src.api.camera import Camera

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

# Geometry for the server-side 3D preview (/api/render) and the local 3D scene
# window in run(). Parsed once; the workspace bounds keep the preview's framing
# stable across poses.
_root_link, _chain, _visuals = parse_urdf(_DEFAULT_URDF_PATH)
_render_bounds = workspace_bounds(arm, _root_link, _chain, _visuals)


_Q_CACHE_TTL = 0.5  # seconds
_q_cache = {"q": None, "t": 0.0}


def _current_q():
    """Current joint angles (radians), throttled to _Q_CACHE_TTL.

    Both the /api/status poll and the run() 3D-scene overlay (which redraws on
    every mobile camera frame, ~5/s) need the current pose, but each actuator
    readback is its own DYNAMIXEL serial round trip x5 joints. Without this
    shared cache the two call sites would independently re-read all 5
    actuators far more often than the arm's pose actually changes, flooding
    the serial bus. Returns None if there's no hardware, or the very first
    read fails (a later failure just keeps serving the stale cached pose).
    """
    if arm_ctrl is None:
        return None
    now = time.monotonic()
    if _q_cache["q"] is not None and now - _q_cache["t"] < _Q_CACHE_TTL:
        return _q_cache["q"]
    servo_degs = [a.get_position() for a in arm_ctrl.actuators]
    if any(d is None for d in servo_degs):
        return _q_cache["q"]
    _q_cache["q"] = arm.servo_deg_to_q(servo_degs)
    _q_cache["t"] = now
    return _q_cache["q"]


# Assumed pinhole focal length in image-WIDTH-normalized units (i.e. a point at
# distance `d` with real-world offset `o` perpendicular to the optical axis
# lands at normalized offset o / (d / _CAMERA_FOCAL_NORM) from center). ~1.0
# corresponds to a ~53 deg horizontal FOV, a reasonable phone-camera ballpark.
# There's no real calibration here, so hand distances below are an estimate,
# not a measurement.
_CAMERA_FOCAL_NORM = 1.0
_HAND_WRIST_TO_THUMB_CMC_M = 0.035  # landmark 0 (wrist) -> landmark 1 (thumb CMC)


def _hand_depth_m(landmarks, frame_width, frame_height):
    """Estimate one hand's distance from the camera (meters) from how large it
    appears on screen: a hand that looks smaller is farther away, not shrunk.
    Assumes the real wrist(0)->thumb-CMC(1) span is _HAND_WRIST_TO_THUMB_CMC_M
    and inverts the pinhole projection (apparent_size = focal * real_size /
    distance) to solve for distance.
    """
    wrist, thumb_cmc = landmarks[0], landmarks[1]
    dx = (thumb_cmc.x - wrist.x) * frame_width
    dy = (thumb_cmc.y - wrist.y) * frame_height
    apparent = math.hypot(dx, dy) / frame_width
    apparent = max(apparent, 1e-4)  # guard against a degenerate (near-zero) detection
    return _CAMERA_FOCAL_NORM * _HAND_WRIST_TO_THUMB_CMC_M / apparent


def _hand_landmark_to_world(landmark, T_ee, depth):
    """Place one mediapipe hand landmark in world coordinates, assuming the
    phone's camera sits at the end-effector and looks along its local +X (the
    same direction as the URDF tool offset). `depth` is the whole hand's
    estimated distance from the camera (see _hand_depth_m) — at that distance
    a normalized image offset of `o` corresponds to a real offset of
    `o * depth / _CAMERA_FOCAL_NORM`, so as the hand moves away it gets placed
    farther back along the optical axis while its reconstructed size stays
    ~_HAND_WRIST_TO_THUMB_CMC_M instead of shrinking the way it does in the raw
    2D image. landmark.z (mediapipe's relative depth within the hand) is
    scaled the same way for finger-to-finger depth (e.g. a curled finger).
    """
    scale = depth / _CAMERA_FOCAL_NORM
    right = (landmark.x - 0.5) * scale
    down = (landmark.y - 0.5) * scale
    forward = depth + landmark.z * scale
    # Points came out rotated 90deg clockwise; rotate the (right, down) plane
    # 90deg counter-clockwise to correct it: (right, down) -> (down, -right).
    right, down = down, -right
    x, y, z = forward, -right, -down
    return (T_ee[0][0] * x + T_ee[0][1] * y + T_ee[0][2] * z + T_ee[0][3],
            T_ee[1][0] * x + T_ee[1][1] * y + T_ee[1][2] * z + T_ee[1][3],
            T_ee[2][0] * x + T_ee[2][1] * y + T_ee[2][2] * z + T_ee[2][3])

gemini = Gemini()
camera = Camera(gemini)


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
        gemini_configured=gemini.configured,
        gemini_error=gemini.error,
    )

@app.route("/api/status")
def status():
    if arm_ctrl is None:
        return jsonify({"connected": False, "position": None, "error": hardware_error})
    q = _current_q()
    return jsonify({"connected": True, "position": list(arm.fk(q)) if q is not None else None})


app.route("/api/ask", methods=["POST"], endpoint="ask")(gemini.ask)
sock.route("/ws/camera")(camera.ws_camera)
app.route("/api/camera/latest.jpg", endpoint="camera_latest_frame")(camera.latest_frame)
app.route("/api/camera/status", endpoint="camera_status")(camera.status)


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


def _parse_degrees(data):
    """Validate a request body carrying one servo degree per joint (in joint
    order). Returns (degrees, error_response). Exactly one is non-None."""
    degs = data.get("degrees") if data else None
    if not isinstance(degs, list) or len(degs) != len(arm.joints):
        return None, (jsonify({"error": f"degrees must be a list of {len(arm.joints)} numbers"}), 400)
    try:
        return [float(d) for d in degs], None
    except (TypeError, ValueError):
        return None, (jsonify({"error": "degrees must be numbers"}), 400)


@app.route("/api/fk", methods=["POST"])
def fk():
    """Forward kinematics only — compute the end-effector position for a set of
    servo angles WITHOUT moving anything. Works even with no hardware, so the
    dashboard can preview an FK pose before committing to it."""
    degs, err = _parse_degrees(request.get_json(force=True))
    if err:
        return err
    q = arm.servo_deg_to_q(degs)
    return jsonify({"position": list(arm.fk(q))})


@app.route("/api/render", methods=["POST"])
def render():
    """Render the arm at the given servo angles to a PNG (server-side matplotlib,
    same drawing code as the desktop simulation). Powers the web 3D preview and
    needs no hardware."""
    degs, err = _parse_degrees(request.get_json(force=True))
    if err:
        return err
    q = arm.servo_deg_to_q(degs)

    fig = Figure(figsize=(6, 6))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    ax.view_init(elev=22, azim=-55)
    draw_pose(ax, arm, _root_link, _chain, _visuals, q, _render_bounds)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=90)
    return Response(buf.getvalue(), mimetype="image/png")


@app.route("/api/ik", methods=["POST"])
def ik():
    """Solve IK for a target (x, y, z) and return the per-joint servo degrees.
    Does NOT move anything — the web sim uses it to preview a solution. `seed`
    (optional) is the current servo degrees, used as the IK starting guess."""
    data = request.get_json(force=True)
    try:
        target = (float(data["x"]), float(data["y"]), float(data["z"]))
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "x, y, z must be numbers"}), 400

    seed = None
    raw_seed = data.get("seed") if data else None
    if isinstance(raw_seed, list) and len(raw_seed) == len(arm.joints):
        try:
            seed = arm.servo_deg_to_q([float(d) for d in raw_seed])
        except (TypeError, ValueError):
            seed = None

    q, converged = arm.ik(target, seed=seed)
    return jsonify({"converged": converged, "servo_deg": arm.q_to_servo_deg(q)})


@app.route("/api/goto_joints", methods=["POST"])
def goto_joints():
    """FK control of ALL actuators at once: set every joint's servo angle and
    drive all five servos, then report the resulting FK position."""
    if arm_ctrl is None:
        return jsonify({"error": "no hardware connected"}), 503

    degs, err = _parse_degrees(request.get_json(force=True))
    if err:
        return err

    Logger.log("WEBAPP", f"goto_joints request: degrees={degs}")
    try:
        pos = arm_ctrl.goto_joints(degs)
    except ValueError as e:  # e.g. a servo degree outside 0..300
        return jsonify({"error": str(e)}), 400
    return jsonify({"position": list(pos)})


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


def initialize_position():
    arm_ctrl.goto_position([0,0,0.3])
    arm_ctrl.actuators[0].goto(180)
    arm_ctrl.actuators[1].goto(180)
    


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
    
    initialize_position()
    
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands

    # Persistent 3D scene (robot + hand overlay), rendered off-screen (Agg,
    # same as /api/render) and shown via cv2 so it doesn't fight the module's
    # Agg backend or need a second GUI event loop on the main thread.
    fig3d = Figure(figsize=(6, 6))
    canvas3d = FigureCanvasAgg(fig3d)
    ax3d = fig3d.add_subplot(111, projection="3d", computed_zorder=False)
    ax3d.view_init(elev=22, azim=-55)

    try:
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:

            while True:
                frame_bytes, frame_count = camera.snapshot()
                # Only decode+show on a genuinely new frame — the phone only
                # sends ~5 fps, but this loop spins far faster (waitKey(1) is a
                # ~1ms cap, not a guarantee), so without this check it would
                # needlessly re-decode and re-display the same bytes every spin.
                if frame_bytes is not None and frame_count != last_frame_count:
                    last_frame_count = frame_count
                    frame = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
                    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)


                    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = hands.process(image)

                    q = _current_q() or [0.0] * len(arm.joints)
                    T_ee = arm.fk_matrix(q)

                    hand_world_point_sets = []
                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            mp_drawing.draw_landmarks(
                                frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                            # 엄지와 검지 손 끝 연결 후 거리 측정
                            point_index = [0,1,5,17]
                            first_point = hand_landmarks.landmark[0]
                            points =  [ hand_landmarks.landmark[i] for i in point_index ]
                            coords = [(int(i.x * frame.shape[1]), int(i.y * frame.shape[0])) for i in points]
                            cv2.line(frame, coords[0], coords[1], (0, 255, 0), 2)
                            cv2.line(frame, coords[1], coords[2], (0, 255, 0), 2)
                            cv2.line(frame, coords[2], coords[3], (0, 255, 0), 2)
                            cv2.line(frame, coords[3], coords[0], (0, 255, 0), 2)


                            # point = tuple((a + b) / 2 for a, b in zip(coords[0], coords[2]))

                            # cv2.circle(frame, point, radius=2, color=(0, 255, 0), thickness=-1)

                            # Bigger on screen -> closer, not "bigger" -- recover a real
                            # distance from apparent size (see _hand_depth_m) so the hand
                            # moves back in 3D as it shrinks on screen, instead of just
                            # shrinking in place.
                            depth = _hand_depth_m(hand_landmarks.landmark, frame.shape[1], frame.shape[0])
                            hand_world_point_sets.append([
                                _hand_landmark_to_world(lm, T_ee, depth)
                                for lm in hand_landmarks.landmark
                            ])

                    if frame is not None:
                        cv2.imshow("Mobile camera", frame)

                    # 3D scene: current robot pose plus any detected hand(s),
                    # placed relative to the end-effector (the phone/camera
                    # mount) via forward kinematics.
                    draw_pose(ax3d, arm, _root_link, _chain, _visuals, q, _render_bounds)
                    for world_points in hand_world_point_sets:
                        draw_points(ax3d, world_points, mp_hands.HAND_CONNECTIONS)
                    canvas3d.draw()
                    scene = cv2.cvtColor(np.asarray(canvas3d.buffer_rgba()), cv2.COLOR_RGBA2BGR)
                    cv2.imshow("3D scene (robot + hand)", scene)
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
