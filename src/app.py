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

Also exposes /api/ask: a Gemini-backed chat/summary endpoint, and the
/api/scan/* routes (perception.document_scanner.DocumentScanner + Gemini) that
back /mobile's document-scan section: detect paper documents in the latest
camera frame, freeze a snapshot, and read the selected one aloud/verbatim. If
GEMINI_API_KEY isn't set (or the google-genai package isn't installed), those
routes report "not configured" instead of crashing the app, same as the
hardware fallback.

Hand tracking (perception.hand_tracker.HandTracker/HandFollower) and face
tracking (perception.face_tracker.FaceTracker/FaceFollower) both run in
run()'s local preview loop, not as Flask routes — same detect → overlay →
place-in-3D-scene → (optionally) compute a follow command pipeline for each.
Hand-follow is currently disabled (HAND_FOLLOW_ENABLED = False below) — the
objects are still built and hands are still drawn, but nothing commands the
arm from them; re-enable the flag to bring it back. Face-follow
(FaceFollower) is the active one: it tracks the primary face's screen-center
offset and depth, and, once the offset passes a dead-zone threshold, picks
one of two responses depending on distance — closer than
FaceFollower.NEAR_DISTANCE_M, a full 3D end-effector target (fully re-centered
sideways/up-down, partially corrected in depth, via arm_ctrl.goto_position);
at or beyond it (the normal desk-distance case), the arm instead holds the
IK-solved pose for FaceFollower.HOME_POSITION and only joint 1 (yaw) is
nudged to keep facing the face, via arm_ctrl.goto_joints. run() is the only
place that actually commands hardware from either follower's output — neither
follower itself has a hardware dependency.

Run from the repository root:
    python -m webapp.app
"""

import numpy as np
import threading
import cv2
import io
import time

import matplotlib
matplotlib.use("Agg")  # headless: render arm previews to PNG, no display/window
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from flask import Flask, Response, jsonify, render_template, request
from flask_sock import Sock

from kinematics.urdf_loader import load_arm, _DEFAULT_URDF_PATH
from kinematics.simulate import parse_urdf, draw_pose, workspace_bounds
from logger import Logger
from perception.document_scanner import DocumentScanner
from perception.face_tracker import FaceTracker, FaceFollower
from perception.hand_tracker import HandTracker, HandFollower
from src.api.camera import Camera
from src.api.gemini import Gemini
from src.api.scan import ScanAPI

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


_Q_CACHE_TTL = 3.0  # seconds
_q_cache = {"q": None, "t": 0.0}


def _current_q():
    """Current joint angles (radians), throttled to _Q_CACHE_TTL.

    Both the /api/status poll and the run() 3D-scene overlay (which redraws on
    every mobile camera frame, ~5/s) need the current pose, but each actuator
    readback is its own DYNAMIXEL serial round trip x5 joints. Without this
    shared cache the two call sites would independently re-read all 5
    actuators far more often than the arm's pose actually changes, flooding
    the serial bus. TTL is a few seconds (not sub-second) on purpose: the 3D
    overlay and HandFollower's "which way am I currently facing" direction
    don't need fresher-than-that pose data, and every cache miss is 5 blocking
    serial round trips, so a short TTL was making this the hot path. Returns
    None if there's no hardware, or the very first read fails (a later
    failure just keeps serving the stale cached pose).
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


gemini = Gemini()
camera = Camera(gemini)
scanner = DocumentScanner()
scan_api = ScanAPI(camera, scanner, gemini)


@app.route("/")
def index():
    return render_template(
        "index.html",
        joint_ids=[joint.id for joint in arm.joints],
        joint_ranges={joint.id: _joint_slider_range(joint) for joint in arm.joints},
        coupled_joints={joint.id: joint.coupled_with for joint in arm.joints
                        if joint.coupled_with is not None},
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
app.route("/api/scan/preview.jpg", endpoint="scan_preview")(scan_api.preview)
app.route("/api/scan/detect", endpoint="scan_detect")(scan_api.detect)
app.route("/api/scan/parse", methods=["POST"], endpoint="scan_parse")(scan_api.parse)


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


def _joint_slider_range(joint):
    """(min_deg, max_deg, home_deg) for this joint's UI slider, derived from
    its URDF <limit> (kinematics/find_joint_limits.py — self-collision-aware,
    not just the servo's 0-300 deg physical range). For a coupled joint
    (currently just joint2, see Joint.coupled_table) this is the
    conservative bound that holds no matter what its partner joint is doing;
    _servo_degs_within_limits does the live, coupling-aware check server-side
    before anything actually moves."""
    lo_deg, hi_deg = sorted((joint.servo_deg(joint.q_min), joint.servo_deg(joint.q_max)))
    return lo_deg, hi_deg, joint.home_deg


def _servo_degs_within_limits(servo_degs):
    """Check servo_degs (one per joint, arm.joints order) against each
    joint's self-collision-aware bounds, resolving joint2's coupling to
    joint3 using the angles in this SAME servo_degs vector (not whatever the
    arm is currently doing) — the values that would actually be commanded
    together. Returns None if all are within range, else a message naming
    the first joint that isn't."""
    q = arm.servo_deg_to_q(servo_degs)
    for i, joint in enumerate(arm.joints):
        q_other = q[arm.id_to_index[joint.coupled_with]] if joint.coupled_with is not None else None
        lo, hi = joint.bounds(q_other=q_other)
        if lo - 1e-9 <= q[i] <= hi + 1e-9:
            continue
        lo_deg, hi_deg = sorted((joint.servo_deg(lo), joint.servo_deg(hi)))
        coupling_note = f" (depends on joint{joint.coupled_with}'s current angle)" \
            if joint.coupled_with is not None else ""
        return (f"joint{joint.id} servo={servo_degs[i]:.1f} deg is outside its "
                f"self-collision-safe range [{lo_deg:.1f}, {hi_deg:.1f}] deg{coupling_note}")
    return None


@app.route("/api/fk", methods=["POST"])
def fk():
    """Forward kinematics only — compute the end-effector position for a set of
    servo angles WITHOUT moving anything. Works even with no hardware, so the
    dashboard can preview an FK pose before committing to it."""
    degs, err = _parse_degrees(request.get_json(force=True))
    if err:
        return err
    q = arm.servo_deg_to_q(degs)
    return jsonify({
        "position": list(arm.fk(q)),
        "within_limits": _servo_degs_within_limits(degs) is None,
    })


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

    limit_error = _servo_degs_within_limits(degs)
    if limit_error:
        return jsonify({"error": limit_error}), 400

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
    if joint_id not in arm.id_to_index:
        return jsonify({"error": f"no joint with id {joint_id}"}), 404

    # This joint moves alone, but a coupled joint's (e.g. joint2's) safe range
    # depends on where every OTHER joint currently is — so check against the
    # rest of the arm's live pose (or home, if it isn't known yet) with just
    # this one joint's degree swapped in.
    current_q = _current_q() or [0.0] * len(arm.joints)
    servo_degs = arm.q_to_servo_deg(current_q)
    servo_degs[arm.id_to_index[joint_id]] = degree
    limit_error = _servo_degs_within_limits(servo_degs)
    if limit_error:
        return jsonify({"error": limit_error}), 400

    Logger.log("WEBAPP", f"goto_joint request: id={joint_id} degree={degree}")
    try:
        actuator.goto(degree)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    return jsonify({"ok": True})


def initialize_position():
    # arm_ctrl.goto_position([0,0,0.3])
    # arm_ctrl.actuators[0].goto(180)
    # arm_ctrl.actuators[1].goto(180)
    pass


# Hand tracking/following is parked for now in favor of face tracking/
# following (see run()) — the HandTracker/HandFollower objects are still
# built (so the whole feature stays intact as a unit), but the per-frame
# detect/draw/follow pipeline is skipped entirely while this is False. Flip
# it back on to resume hand-based following.
HAND_FOLLOW_ENABLED = False


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

    # perception.hand_tracker.HandTracker wraps mediapipe: if mediapipe isn't
    # installed, `tracker.available` is False and process()/draw_overlay() are
    # no-ops, so the camera window still runs without hand detection. Built
    # regardless of HAND_FOLLOW_ENABLED so the feature stays intact as an
    # object even while parked (see the flag's comment above).
    tracker = HandTracker(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    follower = HandFollower()

    # perception.face_tracker.FaceTracker/FaceFollower — same pattern as the
    # hand tracker, currently the active follow behavior (see run()'s
    # docstring for the near/far distance split).
    face_tracker = FaceTracker(max_num_faces=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
    face_follower = FaceFollower(arm=arm)

    # Persistent 3D scene (robot + hand/face overlay), rendered off-screen
    # (Agg, same as /api/render) and shown via cv2 so it doesn't fight the
    # module's Agg backend or need a second GUI event loop on the main thread.
    fig3d = Figure(figsize=(6, 6))
    canvas3d = FigureCanvasAgg(fig3d)
    ax3d = fig3d.add_subplot(111, projection="3d", computed_zorder=False)
    ax3d.view_init(elev=22, azim=-55)

    try:
        while True:
            frame_bytes, frame_count = camera.snapshot()
            # Only decode+show on a genuinely new frame — the phone only
            # sends ~5 fps, but this loop spins far faster (waitKey(1) is a
            # ~1ms cap, not a guarantee), so without this check it would
            # needlessly re-decode and re-display the same bytes every spin.
            if frame_bytes is not None and frame_count != last_frame_count:
                last_frame_count = frame_count
                # /mobile now corrects orientation itself (screen.orientation
                # angle) before sending, so the frame arrives already upright —
                # no server-side rotate needed here anymore.
                frame = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)

                q = _current_q() or [0.0] * len(arm.joints)
                T_ee = arm.fk_matrix(q)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                hands = []
                if HAND_FOLLOW_ENABLED:
                    hands = tracker.process(rgb, T_ee, frame.shape)
                    tracker.draw_overlay(frame, hands)

                faces = face_tracker.process(rgb, T_ee, frame.shape)
                face_tracker.draw_overlay(frame, faces)

                if frame is not None:
                    cv2.imshow("Mobile camera", frame)

                # 3D scene: current robot pose plus any detected hand(s)/face,
                # placed relative to the end-effector (the phone/camera
                # mount) via forward kinematics.
                draw_pose(ax3d, arm, _root_link, _chain, _visuals, q, _render_bounds)
                tracker.draw_forward_axis_debug(ax3d, T_ee, _render_bounds[3] * 0.25)
                if HAND_FOLLOW_ENABLED:
                    tracker.draw_hands_3d(ax3d, hands)
                face_tracker.draw_faces_3d(ax3d, faces)
                canvas3d.draw()
                scene = cv2.cvtColor(np.asarray(canvas3d.buffer_rgba()), cv2.COLOR_RGBA2BGR)
                cv2.imshow("3D scene (robot + hand/face)", scene)

                if arm_ctrl is not None:
                    # Hand-follow: parked (see HAND_FOLLOW_ENABLED above).
                    if HAND_FOLLOW_ENABLED:
                        target = follower.next_ee_target(hands, T_ee)
                        if target is not None:
                            arm_ctrl.goto_position(target)

                    # Face-follow: FaceFollower decides WHERE, WHEN, and HOW
                    # (full 3D reposition vs. yaw-only) — see its docstring.
                    command = face_follower.next_command(faces, T_ee, q)
                    if command is not None:
                        kind, payload = command
                        if kind == "position":
                            arm_ctrl.goto_position(payload)
                        elif kind == "joints":
                            arm_ctrl.goto_joints(payload)
            # waitKey both drives HighGUI's event loop (without it the window
            # never actually renders/refreshes) and lets 'q'/Esc quit.
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        tracker.close()
        face_tracker.close()


if __name__ == "__main__":
    run()
