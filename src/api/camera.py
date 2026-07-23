import json
import threading
import time

from flask import Response, jsonify

from fundamental.const import CameraConst
from fundamental.logger import Logger

WEBM_EBML_HEADER = CameraConst.WEBM_EBML_HEADER


class Camera:
    """Holds the latest JPEG frame streamed from /mobile over /ws/camera plus
    the routes that expose it. The same WebSocket also carries recorded
    voice-question clips (WebM/Opus) from the mic button (binary, told apart
    from JPEG by content — SOI marker vs WebM's EBML header) and hand-landmark
    JSON messages (text) from the phone's own MediaPipe Tasks Vision
    HandLandmarker (perception.hand_tracker no longer runs hand detection on
    the server — see its module docstring for why). Voice clips are handed
    off to `gemini` for transcription and the transcript is sent back over
    that same connection.

    [병합] 이 소켓은 이제 양방향이다. 서버 → 폰 방향으로도 명령을 밀어 넣는다
    (`broadcast()`): 가위바위보 제스처는 서버의 카메라 루프에서 확정되는데
    정작 카메라·마이크·화면은 폰에 있으므로, 확정된 순간 폰에게 "스캔해라 /
    대화를 시작해라 / 카메라를 꺼라"를 되돌려 보내야 한다. transcript를 폰으로
    보내는 것과 정확히 같은 경로를 쓴다."""

    def __init__(self, gemini):
        self.gemini = gemini
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = None
        self.frame_count = 0
        self.clients = 0
        # Latest hand-landmark message from the phone: a list of hands, each
        # 21 [x, y, z] points (mediapipe-normalized), or [] if the phone is
        # currently seeing no hands. None until the first message arrives.
        self.hand_landmarks = None
        # 서버 → 폰 방향으로 명령을 밀어 넣기 위해 열려 있는 소켓을 들고 있는다
        # (제스처 확정은 카메라 루프에서 일어나는데 실행은 폰이 해야 하므로).
        self.sockets = []
        # ws.receive()로 블로킹된 핸들러 스레드와 broadcast()를 부르는 카메라
        # 루프 스레드가 서로 다르므로, 프레임이 섞여 나가지 않도록 전송만 따로
        # 직렬화한다. self.lock(상태 보호)과 별개인 이유: 전송은 네트워크
        # 대기가 있어 오래 걸릴 수 있는데, 그동안 snapshot() 같은 짧은 상태
        # 읽기까지 막으면 미리보기 루프가 통째로 멈춘다.
        self.send_lock = threading.Lock()

    def broadcast(self, payload):
        """연결된 모든 /mobile 클라이언트에 JSON 한 건을 보내고 성공 건수를 반환.

        카메라 루프(다른 스레드)에서 호출된다. 끊긴 소켓은 조용히 걷어낸다 —
        폰이 페이지를 새로고침하면 예전 소켓이 목록에 남아 있을 수 있는데,
        그걸로 send를 시도하다 예외가 나도 루프가 죽으면 안 되기 때문.
        """
        message = json.dumps(payload)
        with self.lock:
            targets = list(self.sockets)
        sent = 0
        for ws in targets:
            if self._send(ws, message):
                sent += 1
        return sent

    def _send(self, ws, message):
        """소켓 하나에 문자열 한 건을 보낸다(실패하면 목록에서 제거).

        모든 전송(transcript, 오류, 제스처 명령)이 이 한 곳을 지나므로
        send_lock을 여기서만 잡으면 된다.
        """
        try:
            with self.send_lock:
                ws.send(message)
            return True
        except Exception as e:
            Logger.log("CAMERA", f"send failed, dropping client: {e}")
            with self.lock:
                if ws in self.sockets:
                    self.sockets.remove(ws)
            return False

    def ws_camera(self, ws):
        """/ws/camera websocket handler."""
        with self.lock:
            self.clients += 1
            self.sockets.append(ws)
        Logger.log("CAMERA", "Mobile client connected")
        try:
            while True:
                data = ws.receive()
                if data is None:
                    break

                if isinstance(data, str):
                    self._handle_text_message(data)
                    continue
                data = bytes(data)

                if data[:4] == WEBM_EBML_HEADER:
                    Logger.log("STT", f"Received voice clip ({len(data)} bytes)")
                    if not self.gemini.configured:
                        self._send(ws, json.dumps({
                            "type": "error",
                            "error": f"Gemini not configured: {self.gemini.error}",
                        }))
                        continue
                    try:
                        transcript = self.gemini.transcribe(data)
                        Logger.log("STT", f"Transcript: {transcript!r}")
                        self._send(ws, json.dumps({"type": "transcript", "text": transcript}))
                    except Exception as e:
                        Logger.log("STT", f"Transcription failed: {e}")
                        self._send(ws, json.dumps({"type": "error", "error": str(e)}))
                    continue

                with self.lock:
                    self.frame = data
                    self.frame_time = time.time()
                    self.frame_count += 1
        finally:
            with self.lock:
                self.clients -= 1
                if ws in self.sockets:
                    self.sockets.remove(ws)
            Logger.log("CAMERA", "Mobile client disconnected")

    def _handle_text_message(self, text):
        """Parse a text WebSocket message. Only "hand_landmarks" is defined
        today (mobile.html's phone-side HandLandmarker results); anything
        else (malformed JSON, unknown type) is logged and ignored rather than
        killing the connection — a bad client message must never crash the
        loop that also carries the JPEG stream and voice clips."""
        try:
            msg = json.loads(text)
        except (ValueError, TypeError) as e:
            Logger.log("CAMERA", f"Ignoring malformed text message: {e}")
            return
        if not isinstance(msg, dict) or msg.get("type") != "hand_landmarks":
            return
        landmarks = msg.get("landmarks")
        if not isinstance(landmarks, list):
            return
        with self.lock:
            self.hand_landmarks = landmarks

    def latest_frame(self):
        """GET /api/camera/latest.jpg"""
        with self.lock:
            frame = self.frame
        if frame is None:
            return jsonify({"error": "no camera frame received yet"}), 404
        return Response(frame, mimetype="image/jpeg")

    def status(self):
        """GET /api/camera/status"""
        with self.lock:
            frame_time = self.frame_time
            frame_count = self.frame_count
            clients = self.clients
        return jsonify({
            "streaming": frame_time is not None,
            "clients": clients,
            "frame_count": frame_count,
            "age_seconds": (time.time() - frame_time) if frame_time is not None else None,
        })

    def snapshot(self):
        """Thread-safe read of (latest frame bytes, frame counter) for the
        server-local cv2 preview loop in webapp.app.run."""
        with self.lock:
            return self.frame, self.frame_count

    def latest_hand_landmarks(self):
        """Thread-safe read of the latest phone-reported hand landmarks for
        run()'s preview loop. Returns None until the first message arrives;
        after that, [] means "phone is looking but sees no hands right now"
        (the phone sends every detection tick, empty or not — see
        mobile.html — so there's no separate staleness timeout to track:
        a stale value only happens if the phone stops sending entirely, in
        which case the whole camera feed has already stalled too)."""
        with self.lock:
            return self.hand_landmarks

    def register(self, app, sock):
        """이 객체가 담당하는 라우트를 Flask 앱/Sock에 붙인다.

        WS   /ws/camera               ws_camera    (폰 → 서버: JPEG·음성·손 랜드마크,
                                                    서버 → 폰: transcript·제스처 명령)
        GET  /api/camera/latest.jpg   latest_frame (대시보드 미리보기)
        GET  /api/camera/status       status
        """
        sock.route("/ws/camera")(self.ws_camera)
        app.route("/api/camera/latest.jpg", endpoint="camera_latest_frame")(self.latest_frame)
        app.route("/api/camera/status", endpoint="camera_status")(self.status)
