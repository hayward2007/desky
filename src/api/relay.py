"""조명 명령을 인터넷 중계소를 통해 파이에 전달한다.

## 왜 중계소인가

두 기계를 인터넷으로 연결하려면 **둘 중 하나는 바깥에서 접근 가능해야** 한다.
그런데 PC도 파이도 공유기 뒤에 있어서 둘 다 접근이 안 된다. 지금까지 이걸
풀려고 한 방법들은 전부 뭔가를 설정해야 했다 — 포트포워딩(공유기를 만져야
함), 터널(계정과 도구가 필요), 고정 IP(통신사에 신청).

중계소 방식은 그걸 뒤집는다. **양쪽 다 바깥으로 나가서** 공용 서버 한 곳에서
만난다. 나가는 연결은 웹 브라우징과 같아서 아무 설정 없이 통과한다.

    PC 서버 ──(나가는 HTTP)──▶ ntfy.sh ◀──(나가는 HTTP)── 파이
                              (중계소)

그래서 필요한 설정이 **딱 하나**다: 양쪽이 같은 "주제 이름"을 쓰는 것.
포트포워딩도, 터널도, IP 주소도, 계정 가입도 없다. PC가 학교 와이파이에 있고
파이가 집에 있어도 그대로 동작한다.

## 중계소는 ntfy.sh

무료 공개 pub/sub 서비스다. 가입도 API 키도 없고, 주제 이름을 URL에 넣어
POST하면 발행, GET하면 구독이다. HTTP만 쓰므로 **파이에 설치할 패키지가 없다**
(파이썬 표준 라이브러리로 충분하다). 직접 띄운 ntfy 서버가 있으면
`DESKY_RELAY`로 주소만 바꾸면 된다.

## 보안 — 솔직하게

**주제 이름이 곧 비밀번호다.** 그 이름을 아는 사람은 누구나 조명을 켤 수 있다.
그래서 짧고 예쁜 이름 대신 길고 무작위한 이름을 쓴다:

    python -c "import secrets; print('desky-' + secrets.token_urlsafe(16))"

오가는 내용은 "각도 140으로 눌러라"뿐이고 TLS로 암호화되지만, 중계소 운영자는
그 사실을 볼 수 있다. 조명 스위치 하나에는 충분하지만, 나중에 문 잠금장치처럼
민감한 걸 붙일 거라면 그때는 자체 중계 서버를 쓸 것.

## 주고받는 내용

    <주제>       PC → 파이   {"cmd":"press","angle":140,"rest":90,"hold_ms":250,"id":3}
    <주제>-ack   파이 → PC   {"ack":3,"ok":true}   /   {"hello":"pi"} (접속·생존 신호)

주제를 둘로 나눈 이유: 한 주제를 같이 쓰면 자기가 보낸 메시지를 자기가 다시
받아서 걸러내야 한다. 나누면 그럴 일이 없다.
"""

import json
import threading
import time
import urllib.request

import requests

from fundamental.const import LightConst
from fundamental.logger import Logger


class Relay:
    """중계소를 통해 파이에 명령을 보내고, 파이가 살아 있는지 지켜본다."""

    URL = LightConst.RELAY_URL
    TOPIC = LightConst.RELAY_TOPIC
    TIMEOUT_S = LightConst.RELAY_TIMEOUT_S
    ALIVE_S = LightConst.RELAY_ALIVE_S

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = {}          # id -> {"event", "ok", "error"}
        self._next_id = 0
        self._last_seen = 0.0       # 파이의 마지막 신호 시각(monotonic)
        if self.enabled:
            # 파이가 보내는 응답·생존신호를 계속 듣는다. 데몬 스레드라 앱이
            # 끝나면 같이 사라진다.
            threading.Thread(target=self._listen_forever, daemon=True).start()
            Logger.log("RELAY", f"중계소 사용: {self.URL}/{self.TOPIC}")

    @property
    def enabled(self) -> bool:
        """주제가 설정돼 있으면 중계 경로를 쓴다. 없으면 Light가 같은 공유기
        직접 호출로 넘어간다."""
        return bool(self.TOPIC)

    @property
    def pi_alive(self) -> bool:
        """최근에 파이의 신호를 받았는지. 시연 전 점검용."""
        return bool(self._last_seen) and (time.monotonic() - self._last_seen) < self.ALIVE_S

    # ------------------------------------------------------------------
    # 보내기
    # ------------------------------------------------------------------
    def press(self, angle, rest, hold_ms):
        """파이에 "눌러라"를 보내고 응답을 기다린다. (성공여부, 에러메시지).

        응답을 기다리는 이유: 그냥 던지고 끝내면 파이가 꺼져 있어도 화면에는
        "켰습니다"라고 뜬다. 시연 중에 왜 안 되는지 모르는 게 제일 나쁘다.
        """
        with self._lock:
            self._next_id += 1
            command_id = self._next_id
            slot = {"event": threading.Event(), "ok": False, "error": None}
            self._pending[command_id] = slot

        body = json.dumps({"cmd": "press", "angle": angle, "rest": rest,
                           "hold_ms": hold_ms, "id": command_id})
        try:
            requests.post(f"{self.URL}/{self.TOPIC}", data=body.encode("utf-8"),
                          timeout=self.TIMEOUT_S).raise_for_status()
        except Exception as e:
            with self._lock:
                self._pending.pop(command_id, None)
            return False, f"중계소에 보내지 못했습니다: {e}"

        if not slot["event"].wait(self.TIMEOUT_S):
            with self._lock:
                self._pending.pop(command_id, None)
            return False, ("파이가 응답하지 않습니다 — 파이에서 server.py가 "
                           "같은 주제로 실행 중인지 확인하세요")
        return slot["ok"], slot["error"]

    # ------------------------------------------------------------------
    # 받기
    # ------------------------------------------------------------------
    def _listen_forever(self):
        """파이의 응답 주제를 계속 구독한다. 끊기면 다시 붙는다.

        중계소는 유휴 연결을 주기적으로 끊고, 네트워크도 언제든 끊긴다.
        재접속을 사람이 신경 쓸 일이 아니므로 여기서 알아서 반복한다.
        """
        url = f"{self.URL}/{self.TOPIC}-ack/json"
        while True:
            try:
                with urllib.request.urlopen(url, timeout=None) as stream:
                    for line in stream:
                        self._handle(line)
            except Exception:
                pass          # 끊김은 정상 — 잠시 후 다시 붙는다
            time.sleep(3)

    def _handle(self, line):
        """중계소가 흘려보내는 JSON 한 줄을 처리한다.

        ntfy는 실제 메시지 외에 연결 유지용 신호도 같이 보내므로, 우리가 만든
        메시지(event=message)만 골라낸다.
        """
        try:
            event = json.loads(line.decode("utf-8"))
        except Exception:
            return
        if event.get("event") != "message":
            return
        try:
            msg = json.loads(event.get("message", ""))
        except Exception:
            return

        self._last_seen = time.monotonic()      # 무슨 메시지든 = 파이가 살아 있다
        command_id = msg.get("ack")
        if command_id is None:
            return
        with self._lock:
            slot = self._pending.pop(command_id, None)
        if slot is None:
            return                               # 이미 시간이 지난 명령
        slot["ok"] = bool(msg.get("ok"))
        slot["error"] = None if slot["ok"] else msg.get("error", "파이가 실패를 알림")
        slot["event"].set()
