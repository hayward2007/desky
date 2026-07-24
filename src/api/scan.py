"""문서 스캔 API — `/mobile`의 "문서 스캔" 동작을 뒷받침하는 라우트.

흐름(단순화됨): "스캔"을 누르면(버튼·음성·제스처) 지금 라이브로 송출되고
있는 카메라 프레임을 통째로 Gemini에게 보내, 화면 속 문서·글자 덩어리(본문)의
텍스트를 읽어 온다. 별도의 문서 윤곽 검출·선택 단계는 없다 — Gemini가
배경의 잡다한 글자는 무시하고 본문만 파싱하도록 프롬프트
(`GeminiConst.DOC_PARSE_INSTRUCTION`)에서 지시한다.

원래는 `perception.document_scanner.DocumentScanner`로 문서 사각형을
검출/원근 보정한 뒤 선택한 영역만 잘라 Gemini에 보내는 3단계
(preview.jpg/detect/parse) 흐름이었지만, 실사용에서는 라이브 프레임을 그냥
통째로 보내는 것으로 충분해 그 검출·선택 단계를 걷어냈다 — `DocumentScanner`는
이제 이 프로젝트 어디서도 쓰지 않는다.
"""

import cv2
import numpy as np
from flask import jsonify, request

from fundamental.logger import Logger


class ScanAPI:
    """카메라 최신 프레임 → Gemini 글자 읽기를 연결하는 라우트 핸들러."""

    def __init__(self, camera, gemini):
        """camera: 최신 프레임 공급자(`Camera`), gemini: 글자 읽기를 담당할 `Gemini`."""
        self.camera = camera
        self.gemini = gemini

    def _decode_latest(self):
        """카메라의 최신 JPEG 프레임을 OpenCV BGR 배열로 디코드한다.
        아직 프레임이 없으면 None."""
        frame_bytes, _ = self.camera.snapshot()
        if frame_bytes is None:
            return None
        return cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)

    def parse(self):
        """POST /api/scan/parse — 바디 {"mode": "text"|"summary"}.

        지금 라이브로 들어오고 있는 최신 프레임을 그대로 Gemini에 보내 글자를 읽는다.
        mode="text"    → 본문 텍스트를 그대로 출력
        mode="summary" → 음성으로 읽어주기 좋은 짧은 요약
        """
        if not self.gemini.configured:
            return jsonify({"error": f"Gemini not configured: {self.gemini.error}"}), 503

        data = request.get_json(force=True) or {}
        mode = data.get("mode", "text")

        frame = self._decode_latest()
        if frame is None:
            return jsonify({"error": "카메라 프레임이 아직 없습니다"}), 400

        _, buf = cv2.imencode(".jpg", frame)
        Logger.log("SCAN", f"parse: mode={mode}, frame bytes={len(buf)}")
        try:
            return jsonify({"text": self.gemini.parse_document(buf.tobytes(), mode)})
        except Exception as e:
            Logger.log("SCAN", f"parse failed: {e}")
            return jsonify({"error": str(e)}), 502

    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다.

        POST /api/scan/parse   parse   {mode} 라이브 프레임을 읽어 텍스트 반환
        """
        app.route("/api/scan/parse", methods=["POST"], endpoint="scan_parse")(self.parse)
