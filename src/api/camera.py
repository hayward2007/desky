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
    voice-question clips (WebM/Opus) from the mic button — the two are told
    apart by content (JPEG's SOI marker vs WebM's EBML header) rather than a
    custom framing header. Voice clips are handed off to `gemini` for
    transcription and the transcript is sent back over that same connection."""

    def __init__(self, gemini):
        self.gemini = gemini
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = None
        self.frame_count = 0
        self.clients = 0

    def ws_camera(self, ws):
        """/ws/camera websocket handler."""
        with self.lock:
            self.clients += 1
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
                    if not self.gemini.configured:
                        ws.send(json.dumps({
                            "type": "error",
                            "error": f"Gemini not configured: {self.gemini.error}",
                        }))
                        continue
                    try:
                        transcript = self.gemini.transcribe(data)
                        Logger.log("STT", f"Transcript: {transcript!r}")
                        ws.send(json.dumps({"type": "transcript", "text": transcript}))
                    except Exception as e:
                        Logger.log("STT", f"Transcription failed: {e}")
                        ws.send(json.dumps({"type": "error", "error": str(e)}))
                    continue

                with self.lock:
                    self.frame = data
                    self.frame_time = time.time()
                    self.frame_count += 1
        finally:
            with self.lock:
                self.clients -= 1
            Logger.log("CAMERA", "Mobile client disconnected")

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
