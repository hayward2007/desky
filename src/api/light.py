"""조명 제어 — 메인 서버가 라즈베리파이의 간단 서보 서버를 HTTP로 호출한다.

왜 폰이 파이를 직접 부르지 않는가:
메인 앱은 ssl_context="adhoc"으로 HTTPS에서 돌아간다. 그 페이지의 JS가
http://<파이>:5000 으로 fetch하면 브라우저가 혼합 콘텐츠(mixed content)로
차단한다 — Chrome에서는 사용자가 허용할 방법조차 없고 조용히 실패한다.
페이지를 HTTP로 내리는 것도 답이 아니다(getUserMedia가 보안 컨텍스트를 요구).

그래서 폰은 이 서버를 부르고, 이 서버가 파이를 부른다. 서버 간 통신은
브라우저를 거치지 않으므로 혼합 콘텐츠도 CORS도 적용되지 않는다.

    폰 --HTTPS--> 메인 서버 --HTTP--> 파이 --PWM--> 서보
"""

import threading

import requests
from flask import jsonify, request

from fundamental.const import LightConst
from fundamental.logger import Logger


class Light:
    """벽 스위치에 붙은 서보를 눌러 조명을 켜고 끈다.

    라우트 등록은 `register()`로 한다(Calendar/Camera와 같은 관례). 파이가
    꺼져 있어도 앱은 그대로 뜨고, 조명 요청만 502로 실패한다 — 하드웨어/
    Gemini 미구성 때와 같은 "기능만 빠지고 앱은 계속 뜬다" 패턴.
    """

    # 값 설명은 fundamental.const.LightConst 참고.
    PI_URL = LightConst.PI_URL
    ON_ANGLE = LightConst.ON_ANGLE
    OFF_ANGLE = LightConst.OFF_ANGLE
    REST_ANGLE = LightConst.REST_ANGLE
    HOLD_MS = LightConst.HOLD_MS
    TIMEOUT_S = LightConst.TIMEOUT_S

    def __init__(self):
        self.state = None          # "on" | "off" | None(모름)
        self.lock = threading.Lock()

    def _post(self, angle):
        url = f"{self.PI_URL}/api/press"
        payload = {"angle": angle, "rest": self.REST_ANGLE, "hold_ms": self.HOLD_MS}
        response = requests.post(url, json=payload, timeout=self.TIMEOUT_S)
        response.raise_for_status()
        return response.json()

    def set(self, state):
        """조명을 켜거나 끈다. (성공여부, 에러메시지)를 돌려준다."""
        if state == "toggle":
            state = "off" if self.state == "on" else "on"
        if state not in ("on", "off"):
            return False, "state must be on/off/toggle"

        angle = self.ON_ANGLE if state == "on" else self.OFF_ANGLE
        # 서보가 누르고 되돌아오는 동안(~0.6초) 파이 서버가 응답을 잡고 있으므로,
        # 그 사이 두 번째 요청이 겹치지 않도록 잠근다.
        with self.lock:
            try:
                self._post(angle)
            except requests.exceptions.Timeout:
                return False, f"파이 응답 없음 ({self.PI_URL})"
            except requests.exceptions.ConnectionError:
                return False, f"파이에 연결 실패 ({self.PI_URL})"
            except Exception as e:
                return False, f"파이 오류: {e}"
            self.state = state
        Logger.log("LIGHT", f"→ {state} ({angle:.0f}deg)")
        return True, None

    def set_async(self, state):
        """카메라 루프에서 부를 때 쓴다. 네트워크 대기로 영상이 멈추지 않도록
        별도 스레드에서 보내고 즉시 돌아온다."""
        def _run():
            ok, error = self.set(state)
            if not ok:
                Logger.log("LIGHT", f"실패: {error}")
        threading.Thread(target=_run, daemon=True).start()

    # ------------------------------------------------------------------
    # HTTP 라우트
    # ------------------------------------------------------------------
    def command(self):
        """POST /api/light — 바디 {"state": "on"|"off"|"toggle"}."""
        data = request.get_json(force=True, silent=True) or {}
        ok, error = self.set(data.get("state", "toggle"))
        if not ok:
            return jsonify({"error": error}), 502
        return jsonify({"ok": True, "state": self.state})

    def status(self):
        """GET /api/light/status — 서보는 위치를 되읽을 수 없으므로 이 값은
        '서버가 마지막으로 명령한 상태'다. 손으로 스위치를 누르면 어긋난다."""
        return jsonify({"state": self.state, "pi_url": self.PI_URL})

    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다.

        폰이 파이(http)를 직접 부르면 혼합 콘텐츠로 차단되므로, 폰은 항상 같은
        출처의 아래 라우트를 부르고 서버가 대신 파이를 호출한다.

        POST /api/light         command {state: on|off|toggle}
        GET  /api/light/status  status  (마지막으로 명령한 상태)
        """
        app.route("/api/light", methods=["POST"], endpoint="light")(self.command)
        app.route("/api/light/status", endpoint="light_status")(self.status)
