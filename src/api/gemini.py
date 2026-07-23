"""Gemini API 래퍼 — 채팅/요약(`/api/ask`), 음성 인식(STT), 문서 글자 읽기를 담당.

GEMINI_API_KEY가 없거나 `google-genai` 패키지가 없으면 `configured`가 False가
되고, 라우트는 크래시 대신 503 "not configured"를 돌려준다 — 하드웨어 미연결
때와 같은 패턴.
"""

import os

from flask import jsonify, request

from fundamental.const import GeminiConst
from fundamental.logger import Logger


class Gemini:
    """google-genai 클라이언트 한 개를 감싸고, Gemini를 쓰는 모든 기능을 모아 둔 객체."""

    # 모델명 + 시스템 프롬프트 + 토큰 한도 설명은 fundamental.const.GeminiConst 참고.
    MODEL = GeminiConst.MODEL
    CHAT_INSTRUCTION = GeminiConst.CHAT_INSTRUCTION
    SUMMARY_INSTRUCTION = GeminiConst.SUMMARY_INSTRUCTION
    STT_INSTRUCTION = GeminiConst.STT_INSTRUCTION
    DOC_PARSE_INSTRUCTION = GeminiConst.DOC_PARSE_INSTRUCTION
    DOC_READ_INSTRUCTION = GeminiConst.DOC_READ_INSTRUCTION
    MAX_OUTPUT_TOKENS = GeminiConst.MAX_OUTPUT_TOKENS

    def __init__(self):
        self.client = None
        self.error = None
        self._types = None

        try:
            from google import genai
            from google.genai import types

            self.client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])
            self._types = types
            Logger.log("GEMINI", f"Gemini client configured (model={self.MODEL})")
        except Exception as e:
            self.error = str(e)
            Logger.log("GEMINI", f"Gemini not configured: {self.error}")

    @property
    def configured(self):
        return self.client is not None

    # ------------------------------------------------------------------
    # 응답 파싱
    # ------------------------------------------------------------------
    @staticmethod
    def answer_text(response):
        """응답에서 '사고(thought)' 파트를 빼고 실제 답변 텍스트만 이어 붙인다.

        사고 내용이 파트로 섞여 와도 사용자에게 노출되지 않게 하는 안전장치.
        파트 구조가 예상과 다르면 `response.text`로 폴백한다.
        """
        chunks = []
        for cand in (getattr(response, "candidates", None) or []):
            content = getattr(cand, "content", None)
            for part in (getattr(content, "parts", None) or []):
                if getattr(part, "thought", False):
                    continue  # 사고 파트는 제외
                text = getattr(part, "text", None)
                if text:
                    chunks.append(text)
        text = "".join(chunks).strip()
        if not text:
            text = (getattr(response, "text", "") or "").strip()
        return text

    def ask(self):
        """POST /api/ask — body {"text": "...", "mode": "chat"|"summary"}."""
        if self.client is None:
            return jsonify({"error": f"Gemini not configured: {self.error}"}), 503

        data = request.get_json(force=True)
        text = data.get("text") if data else None
        mode = data.get("mode", "chat") if data else "chat"
        if not text:
            return jsonify({"error": "text is required"}), 400

        system = self.SUMMARY_INSTRUCTION if mode == "summary" else self.CHAT_INSTRUCTION

        Logger.log("GEMINI", f"ask request: mode={mode}, text length={len(text)}")
        try:
            response = self.client.models.generate_content(
                model=self.MODEL,
                contents=text,
                config=self._types.GenerateContentConfig(
                    system_instruction=system,
                    max_output_tokens=self.MAX_OUTPUT_TOKENS,
                ),
            )
            answer = self.answer_text(response) or "죄송해요, 답을 만들지 못했어요. 다시 말해줄래요?"
            return jsonify({"answer": answer})
        except Exception as e:
            Logger.log("GEMINI", f"ask failed: {e}")
            return jsonify({"error": str(e)}), 502

    def transcribe(self, audio_bytes):
        """Run one recorded voice clip (WebM/Opus bytes) through Gemini and
        return the transcript text. Raises on failure — callers turn that
        into a {"type": "error"} reply."""
        audio_part = self._types.Part.from_bytes(data=audio_bytes, mime_type="audio/webm")
        response = self.client.models.generate_content(
            model=self.MODEL,
            contents=[audio_part, self.STT_INSTRUCTION],
        )
        return self.answer_text(response)

    def parse_document(self, image_bytes, mode="text"):
        """문서 이미지(JPEG 바이트)에서 글자를 읽는다.

        mode="text"    → 문서의 모든 텍스트를 원본 줄바꿈까지 최대한 살려 그대로 출력
        mode="summary" → 음성으로 읽어주기 좋은 3문장 이내 요약
        """
        image_part = self._types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg")
        instruction = self.DOC_READ_INSTRUCTION if mode == "summary" else self.DOC_PARSE_INSTRUCTION
        response = self.client.models.generate_content(
            model=self.MODEL,
            contents=[image_part, instruction],
        )
        return self.answer_text(response) or "(텍스트 없음)"

    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다.

        POST /api/ask  ask  {question, mode: chat|summary}
        (음성 클립의 STT는 라우트가 아니라 src.api.camera.Camera가 웹소켓으로
        받은 오디오를 `transcribe()`에 직접 넘기는 방식이다.)
        """
        app.route("/api/ask", methods=["POST"], endpoint="ask")(self.ask)
