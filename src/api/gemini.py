from logger import Logger
from flask import jsonify, request
import os

class Gemini :
    
    GEMINI_MODEL = "gemini-flash-latest"  # fast, generous free tier
    GEMINI_CHAT_INSTRUCTION = "너는 친절한 한국어 AI 비서야. 자연스럽게 대화해줘."
    GEMINI_SUMMARY_INSTRUCTION = "너는 문서를 간결하게 요약해주는 비서야. 핵심만 한국어로 정리해줘."
            
    def __init__(self):
        self.gemini_client = None
        self.gemini_error = None
        
        try:
            from google import genai
            from google.genai import types as genai_types

            gemini_client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])
            Logger.log("GEMINI", f"Gemini client configured (model={self.GEMINI_MODEL})")
        except Exception as e:
            gemini_error = str(e)
            Logger.log("GEMINI", f"Gemini not configured: {gemini_error}")
            
            
    def ask(self):
        if self.gemini_client is None:
            return jsonify({"error": f"Gemini not configured: {self.gemini_error}"}), 503
    
        data = request.get_json(force=True)
        text = data.get("text") if data else None
        mode = data.get("mode", "chat") if data else "chat"
        if not text:
            return jsonify({"error": "text is required"}), 400
    
        system = self.GEMINI_SUMMARY_INSTRUCTION if mode == "summary" else self.GEMINI_CHAT_INSTRUCTION
    
        Logger.log("GEMINI", f"ask request: mode={mode}, text length={len(text)}")
        try:
            response = self.gemini_client.models.generate_content(
                model=self.GEMINI_MODEL,
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