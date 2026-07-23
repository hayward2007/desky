"""문서 스캔 API — `/mobile`의 "문서 스캔" 섹션을 뒷받침하는 라우트 묶음.

흐름:
  1. 실시간 미리보기 — 폰이 보내는 최신 프레임에서 문서를 찾아 초록 사각형으로 표시
  2. "스캔하기" — 그 순간의 원본 프레임 + 문서 좌표를 고정(freeze)해서 반환
  3. 사용자가 문서를 하나 고르면 그 영역만 잘라 Gemini에게 글자를 읽힌다
"""

import base64
import threading

import cv2
import numpy as np
from flask import Response, jsonify, request

from fundamental.logger import Logger


class ScanAPI:
    """카메라 프레임 → 문서 검출 → Gemini 글자 읽기까지를 연결하는 라우트 핸들러 묶음."""

    def __init__(self, camera, scanner, gemini):
        """camera: 최신 프레임 공급자(`Camera`), scanner: `DocumentScanner`,
        gemini: 글자 읽기를 담당할 `Gemini` 인스턴스."""
        self.camera = camera
        self.scanner = scanner
        self.gemini = gemini

        # "스캔하기"를 누른 순간의 프레임/문서 목록. 이후 파싱 요청이 이 스냅샷을
        # 기준으로 동작해야 하므로(그 사이에 카메라가 움직여도) 따로 붙잡아 둔다.
        self.lock = threading.Lock()
        self.frame = None
        self.documents = []

    def _decode_latest(self):
        """카메라의 최신 JPEG 프레임을 OpenCV BGR 배열로 디코드한다.
        아직 프레임이 없으면 None."""
        frame_bytes, _ = self.camera.snapshot()
        if frame_bytes is None:
            return None
        return cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)

    def preview(self):
        """GET /api/scan/preview.jpg — 실시간 미리보기.

        최신 프레임에서 문서를 검출해 초록 사각형 + 번호를 그려 JPEG로 반환한다.
        `/mobile`이 이 URL을 주기적으로 갱신해서 "지금 몇 번 문서가 잡히는지"를 보여준다.
        """
        frame = self._decode_latest()
        if frame is None:
            return jsonify({"error": "no camera frame yet"}), 404
        documents = self.scanner.detect(frame)
        _, buf = cv2.imencode(".jpg", self.scanner.draw(frame, documents))
        return Response(buf.tobytes(), mimetype="image/jpeg")

    def detect(self):
        """GET /api/scan/detect — "스캔하기" 버튼.

        그 순간의 원본 프레임과 검출된 문서 좌표를 스냅샷으로 저장한 뒤,
        오버레이 없는 원본 이미지(base64)와 문서 좌표를 함께 반환한다.
        선택 화면은 이 좌표로 사각형을 직접 그리고, 탭한 문서를 골라 파싱을 요청한다.
        """
        frame = self._decode_latest()
        if frame is None:
            return jsonify({"error": "no camera frame yet"}), 404

        documents = self.scanner.detect(frame)
        with self.lock:
            self.frame = frame
            self.documents = documents

        _, buf = cv2.imencode(".jpg", frame)
        img_b64 = base64.b64encode(buf.tobytes()).decode()
        Logger.log("SCAN", f"detect: {len(documents)} document(s)")
        return jsonify({
            "image": "data:image/jpeg;base64," + img_b64,
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "documents": [doc.to_dict() for doc in documents],
        })

    def parse(self):
        """POST /api/scan/parse — 바디 {"id": 문서번호|null, "mode": "text"|"summary"}.

        저장된 스냅샷에서 해당 문서 영역만 원근 보정해 잘라낸 뒤 Gemini에게 읽힌다.
        `id`가 없으면 프레임 전체를 그대로 넘긴다("전체 파싱").
        mode="summary"는 음성으로 읽어주기 좋은 짧은 요약을 돌려준다.
        """
        if not self.gemini.configured:
            return jsonify({"error": f"Gemini not configured: {self.gemini.error}"}), 503

        data = request.get_json(force=True) or {}
        doc_id = data.get("id")
        mode = data.get("mode", "text")

        with self.lock:
            frame = self.frame
            documents = self.documents
        if frame is None:
            return jsonify({"error": "먼저 스캔하세요"}), 400

        quad = next((d.quad for d in documents if d.id == doc_id), None)
        crop = self.scanner.four_point_transform(frame, quad) if quad is not None else frame
        _, buf = cv2.imencode(".jpg", crop)
        Logger.log("SCAN", f"parse: id={doc_id}, mode={mode}, crop bytes={len(buf)}")

        try:
            return jsonify({"text": self.gemini.parse_document(buf.tobytes(), mode)})
        except Exception as e:
            Logger.log("SCAN", f"parse failed: {e}")
            return jsonify({"error": str(e)}), 502
