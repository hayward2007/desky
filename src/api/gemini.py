"""Gemini API 래퍼 — 채팅/요약(`/api/ask`), 음성 인식(STT), 문서 글자 읽기를 담당.

GEMINI_API_KEY가 없거나 `google-genai` 패키지가 없으면 `configured`가 False가
되고, 라우트는 크래시 대신 503 "not configured"를 돌려준다 — 하드웨어 미연결
때와 같은 패턴.
"""

import os

from flask import jsonify, request

from logger import Logger


class Gemini:
    """google-genai 클라이언트 한 개를 감싸고, Gemini를 쓰는 모든 기능을 모아 둔 객체."""

    MODEL = "gemini-flash-latest"  # 빠르고 무료 한도가 넉넉함

    # 답이 TTS로 그대로 읽히므로 짧고 말하듯이. 목록/기호/마크다운은 소리로 읽으면
    # 이상하니 금지한다.
    CHAT_INSTRUCTION = (
        "너는 음성으로 대답하는 한국어 AI 비서야. 대답은 그대로 소리 내어 읽히니까, "
        "최대한 짧게, 100자 이내로 꼭 필요한 핵심만 자연스러운 구어체로 말해. 덧붙이는 설명은 생략해. "
        "목록·번호·기호·마크다운·이모지는 쓰지 말고 말하듯이 이어서 답해."
    )
    SUMMARY_INSTRUCTION = (
        "너는 문서를 음성으로 요약해주는 한국어 비서야. 핵심만 3문장 이내로, "
        "목록·기호 없이 말하듯 간결하게 정리해."
    )
    STT_INSTRUCTION = "이 오디오를 한국어 텍스트로 정확히 받아써줘. 설명 없이 텍스트만 출력해."

    # 문서 전체 텍스트를 그대로 뽑을 때 (화면에 표시용)
    DOC_PARSE_INSTRUCTION = (
        "이 이미지는 종이 문서를 촬영한 것이다. 문서에 적힌 모든 텍스트를 정확히 "
        "읽어서 그대로 출력해라. 원본의 줄바꿈을 최대한 유지하고, 설명이나 요약 없이 "
        "텍스트만 출력해라. 손글씨도 최대한 읽어라. 문서에 글자가 없으면 '(텍스트 없음)'"
        "이라고만 답해라."
    )
    # 문서를 음성으로 읽어줄 때 (짧은 요약, TTS 친화적)
    DOC_READ_INSTRUCTION = (
        "이 이미지는 사용자가 가리킨 종이 문서다. 내용을 읽고 핵심만 한국어로 "
        "간결하게 3문장 이내로, 목록·기호·마크다운 없이 말하듯이 설명해라. "
        "글자가 없으면 '문서에서 글자를 못 찾았어요'라고만 답해라."
    )

    # 사고를 억지로 누르면 이 모델은 사고 과정을 답변 본문에 적어버린다(페르소나·
    # 포맷 체크리스트가 새어 나옴). 그래서 사고 수준은 기본값에 맡기고 천장만
    # 넉넉히 줘서 '사고 + 답'이 잘리지 않게 한다. 천장을 올려도 답이 길어지진 않는다.
    MAX_OUTPUT_TOKENS = 8192

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
