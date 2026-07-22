import os

from flask import jsonify, request

from logger import Logger


class Gemini:
    """Wraps the google-genai client used by /api/ask (chat/summary) and by
    Camera's voice-clip transcription. If GEMINI_API_KEY isn't set (or
    google-genai isn't installed), `configured` is False and callers get a
    "not configured" error instead of a crash."""

    MODEL = "gemini-flash-latest"  # fast, generous free tier
    CHAT_INSTRUCTION = "너는 친절한 한국어 AI 비서야. 자연스럽게 대화해줘."
    SUMMARY_INSTRUCTION = "너는 문서를 간결하게 요약해주는 비서야. 핵심만 한국어로 정리해줘."
    STT_INSTRUCTION = "이 오디오를 한국어 텍스트로 정확히 받아써줘. 설명 없이 텍스트만 출력해."

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
                    max_output_tokens=1024,
                ),
            )
            return jsonify({"answer": response.text})
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
        return response.text
