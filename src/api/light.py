"""조명 제어 — 메인 서버가 라즈베리파이의 간단 서보 서버를 HTTP로 호출한다.

왜 폰이 파이를 직접 부르지 않는가:
메인 앱은 ssl_context="adhoc"으로 HTTPS에서 돌아간다. 그 페이지의 JS가
http://<파이>:5000 으로 fetch하면 브라우저가 혼합 콘텐츠(mixed content)로
차단한다 — Chrome에서는 사용자가 허용할 방법조차 없고 조용히 실패한다.
페이지를 HTTP로 내리는 것도 답이 아니다(getUserMedia가 보안 컨텍스트를 요구).

그래서 폰은 이 서버를 부르고, 이 서버가 파이에 명령을 전달한다. 서버 간
통신은 브라우저를 거치지 않으므로 혼합 콘텐츠도 CORS도 적용되지 않는다.

파이에 전달하는 경로는 둘뿐이고, 설정에 따라 자동으로 갈린다.

  ① 인터넷 중계 — 서로 다른 망에 있어도 된다
        PC 서버 ──(나가는 HTTP)──▶ 중계소 ◀──(나가는 HTTP)── 파이 ──▶ 서보
     양쪽 다 바깥으로 나가 만나므로 포트포워딩도, 터널도, 서로의 IP도 필요
     없다. `DESKY_RELAY_TOPIC`을 양쪽에 같은 값으로 넣으면 이 경로를 쓴다.
     자세한 내용은 src/api/relay.py 참고.

  ② 직접 호출 — 같은 공유기에 있을 때
        폰 --HTTPS--> PC 서버 --HTTP--> 파이(192.168.x.x) --PWM--> 서보
     중계 주제를 설정하지 않았을 때의 기본 동작. `PI_URL`만 맞으면 된다.
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

    def __init__(self, relay=None):
        """relay: src.api.relay.Relay (설정돼 있지 않으면 ② 직접 호출만 쓴다)."""
        self.state = None          # "on" | "off" | None(모름)
        self.relay = relay
        self.lock = threading.Lock()

    def _payload(self, angle):
        """두 경로가 똑같이 쓰는 명령 내용 — 눌렀다 되돌아오는 동작 한 번."""
        return {"angle": angle, "rest": self.REST_ANGLE, "hold_ms": self.HOLD_MS}

    def _press(self, angle):
        """파이에 '눌러라'를 전달한다. (성공여부, 에러메시지) 반환.

        파이가 서버에 접속해 있으면 그 연결로 보내고(①), 아니면 예전처럼
        PI_URL로 직접 HTTP를 쏜다(②). 둘 다 결과 형태가 같아서 호출부는
        어느 경로였는지 몰라도 된다.
        """
        if self.relay is not None and self.relay.enabled:
            return self.relay.press(angle, self.REST_ANGLE, self.HOLD_MS)

        try:
            response = requests.post(f"{self.PI_URL}/api/press",
                                     json=self._payload(angle), timeout=self.TIMEOUT_S)
            response.raise_for_status()
            return True, None
        except requests.exceptions.Timeout:
            return False, f"파이 응답 없음 ({self.PI_URL})"
        except requests.exceptions.ConnectionError:
            # 가장 흔한 실패다 — 서버와 파이가 다른 망에 있는 경우.
            # 역방향 연결(①)을 쓰라는 힌트를 에러에 함께 담는다.
            return False, (f"파이에 연결 실패 ({self.PI_URL}). 파이가 다른 공유기에 "
                           f"있다면 양쪽에 DESKY_RELAY_TOPIC을 같은 값으로 넣어 "
                           f"인터넷 중계를 쓰세요")
        except Exception as e:
            return False, f"파이 오류: {e}"

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
            ok, error = self._press(angle)
            if not ok:
                return False, error
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
        via_relay = self.relay is not None and self.relay.enabled
        return jsonify({
            "state": self.state,
            # 지금 어느 경로로 나가는지 + 파이가 살아 있는지 — 시연 전 점검용.
            "route": "relay" if via_relay else "http",
            "pi_alive": self.relay.pi_alive if via_relay else None,
            "pi_url": None if via_relay else self.PI_URL,
        })

    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다.

        폰이 파이(http)를 직접 부르면 혼합 콘텐츠로 차단되므로, 폰은 항상 같은
        출처의 아래 라우트를 부르고 서버가 대신 파이를 호출한다.

        POST /api/light         command {state: on|off|toggle}
        GET  /api/light/status  status  (마지막으로 명령한 상태)
        """
        app.route("/api/light", methods=["POST"], endpoint="light")(self.command)
        app.route("/api/light/status", endpoint="light_status")(self.status)
