#!/usr/bin/env python3
"""라즈베리파이 서보 컨트롤 — 벽 스위치를 눌러 조명을 켜고 끈다.

동작 모드가 둘이고 **둘은 동시에 켜진다.**

  ① 인터넷 중계 — PC와 다른 공유기에 있어도 동작 (설치할 패키지 없음)
     환경변수 DESKY_RELAY_TOPIC이 있으면 공용 중계소를 구독하며 명령을
     기다린다. PC도 파이도 **바깥으로 나가서** 중계소에서 만나므로 포트
     포워딩도, 터널도, 서로의 IP도 필요 없다. 구독은 그냥 긴 HTTP GET이라
     파이썬 표준 라이브러리만으로 되고, pip/apt로 설치할 것이 없다.

  ② 로컬 HTTP 서버 — 같은 공유기에서
     http://<파이IP>:5000 에 접속하면 버튼으로 직접 돌려볼 수 있다. 각도를
     맞추거나 배선을 확인할 때 쓴다. PC가 같은 공유기에 있으면 이쪽으로도
     명령이 들어온다(PC의 PI_URL 설정).

설치:
    sudo apt install -y python3-flask
    sudo apt install -y python3-lgpio      # GPIO — 파이 5·파이 4 공통

    # GPIO는 lgpio 하나면 된다. 예전 안내의 pigpio는 파이 5에서 GPIO가 RP1
    # 칩으로 옮겨가며 동작하지 않고, 최근 라즈베리파이 OS에는 패키지 자체가
    # 없어 "Package 'pigpio' has no installation candidate" 오류가 난다.
    # 이 스크립트는 pigpio가 있으면 쓰고 없으면 자동으로 lgpio를 쓴다.

실행:
    # ①+② (PC와 다른 망에 있어도 동작)
    DESKY_RELAY_TOPIC=<PC와 같은 값> python3 server.py

    # ②만 (PC와 같은 공유기)
    python3 server.py
    # PC의 PI_URL에 넣을 주소 확인: hostname -I

주제 이름이 곧 비밀번호다. 그 이름을 아는 사람은 누구나 이 조명을 켤 수 있으므로
길고 무작위한 이름을 쓸 것:
    python3 -c "import secrets; print('desky-' + secrets.token_urlsafe(16))"

GPIO 라이브러리가 없어도 실행된다(모의 모드). 하드웨어 없이 중계 연결과 명령
경로를 먼저 확인할 수 있으므로, 배선 문제와 네트워크 문제를 분리해서 디버깅할
수 있다.
"""

import json
import os
import threading
import urllib.request

from flask import Flask, jsonify, render_template, request


PIN = 18                # BCM 번호 (12번 물리 핀)
PULSE_MIN_US = 500      # 0도
PULSE_MAX_US = 2500     # 180도
DETACH_AFTER_S = 0.5    # 이동 후 신호를 끊기까지의 시간

# --- 인터넷 중계 설정 (환경변수) ----------------------------------------
# PC와 같은 값을 넣으면 서로 다른 공유기에 있어도 조명이 동작한다.
# 비워 두면 로컬 HTTP 서버만 뜬다(같은 공유기 전용).
#   만드는 법: python3 -c "import secrets; print('desky-' + secrets.token_urlsafe(16))"
RELAY_TOPIC = os.environ.get("DESKY_RELAY_TOPIC", "").strip()
RELAY_URL = os.environ.get("DESKY_RELAY", "https://ntfy.sh")
# 로컬 확인용 웹 화면 포트. 이미 쓰는 프로그램이 있으면 PI_PORT로 바꾼다.
LOCAL_PORT = int(os.environ.get("PI_PORT", "5000"))


def angle_to_pulse(angle):
    angle = max(0.0, min(180.0, float(angle)))
    return int(PULSE_MIN_US + (angle / 180.0) * (PULSE_MAX_US - PULSE_MIN_US))


class Servo:
    """pigpio(파이 4 이하) → lgpio(파이 5) → 모의 모드 순으로 자동 선택.

    파이 5는 GPIO가 RP1 칩으로 옮겨가면서 pigpio가 동작하지 않는다. 두 라이브러리
    모두 '펄스 폭을 마이크로초로 주는' 방식이라 함수 이름만 다르고 값은 같다.
    """

    def __init__(self, pin):
        self.pin = pin
        self.angle = 90.0
        self.timer = None
        self.backend, self.handle = self._pick_backend()

    def _pick_backend(self):
        try:
            import pigpio
            pi = pigpio.pi()
            if pi.connected:
                print("[gpio] pigpio 사용")
                return "pigpio", pi
            print("[gpio] pigpio 데몬 미실행 — `sudo systemctl start pigpiod`")
        except ImportError:
            pass

        try:
            import lgpio
            # 칩 번호는 기기/펌웨어에 따라 다르다 — 파이 5는 GPIO가 RP1 칩으로
            # 옮겨가면서 한동안 gpiochip4였고 최근 펌웨어에서는 gpiochip0이다.
            # 파이 4 이하는 gpiochip0. 문제는 **번호만 맞으면 열리는 게 아니라**,
            # 엉뚱한 칩도 열기 자체는 성공한다는 것이다(그 뒤 서보만 조용히 안
            # 움직인다). 그래서 여는 것으로 끝내지 말고 실제로 이 핀을 점유해
            # 보고, 점유되는 칩을 진짜 헤더 칩으로 판정한다.
            for chip in (0, 4, 1, 2, 3):
                try:
                    handle = lgpio.gpiochip_open(chip)
                except Exception:
                    continue
                if self._claim_output(lgpio, handle):
                    print(f"[gpio] lgpio 사용 (chip {chip}, GPIO{self.pin})")
                    return "lgpio", handle
                try:
                    lgpio.gpiochip_close(handle)
                except Exception:
                    pass
            print("[gpio] lgpio는 있으나 GPIO를 점유하지 못했습니다 — "
                  "다른 프로그램이 쓰고 있거나(pigpiod 등) 권한이 없을 수 있습니다")
        except ImportError:
            pass

        print("[gpio] 라이브러리 없음 — 모의 모드로 실행합니다(서보는 안 움직임)")
        return "mock", None

    def _claim_output(self, lgpio, handle):
        """이 칩에서 서보 핀을 출력으로 점유해 본다. 성공하면 True.

        lgpio는 `tx_servo`를 부르기 전에 핀을 출력으로 점유해 둬야 한다 —
        점유하지 않으면 "GPIO not allocated" 오류가 나거나 아무 일도 일어나지
        않는다(pigpio는 이 단계가 필요 없어서 옮겨올 때 놓치기 쉬운 부분이다).

        동시에 이 시도가 **칩이 맞는지 확인하는 수단**이기도 하다: 헤더 핀을
        갖고 있지 않은 칩에서는 점유가 실패하므로, 성공한 칩이 곧 우리가 찾던
        칩이다.
        """
        try:
            lgpio.gpio_claim_output(handle, self.pin)
            return True
        except Exception as e:
            # 이미 우리가 점유한 상태라면 그대로 써도 된다(재시작 등).
            if "busy" in str(e).lower():
                return True
            return False

    def _write(self, pulse):
        if self.backend == "pigpio":
            self.handle.set_servo_pulsewidth(self.pin, pulse)
        elif self.backend == "lgpio":
            import lgpio
            lgpio.tx_servo(self.handle, self.pin, pulse)
        else:
            print(f"[mock] GPIO{self.pin} ← {pulse}us")

    def goto(self, angle):
        angle = max(0.0, min(180.0, float(angle)))
        self._write(angle_to_pulse(angle))
        self.angle = angle

        # 이동이 끝나면 신호를 끊는다. 계속 주면 서보가 위치를 붙들려고 미세하게
        # 떨면서 전류를 먹는데, 파이 5V 핀에서 전원을 뽑는 경우 이게 전압 강하와
        # 리부팅의 원인이 된다. 이전 예약은 취소해야 연속 명령에서 오래된 타이머가
        # 먼저 터져 이동 중에 신호를 끊는 일이 없다.
        if self.timer:
            self.timer.cancel()
        self.timer = threading.Timer(DETACH_AFTER_S, lambda: self._write(0))
        self.timer.daemon = True
        self.timer.start()
        return angle

    def press(self, angle, rest=90.0, hold_ms=250):
        """스위치를 밀었다가 중립으로 되돌아온다.

        벽 스위치는 각도를 유지하는 게 아니라 한 번 밀면 끝이다. 밀어붙인 채로
        두면 서보가 스위치를 계속 붙들고 전류를 먹는다.
        """
        import time
        if self.timer:
            self.timer.cancel()
        self._write(angle_to_pulse(angle))
        time.sleep(hold_ms / 1000.0)
        self._write(angle_to_pulse(rest))
        time.sleep(0.35)
        self._write(0)
        self.angle = rest
        return angle

    def close(self):
        if self.timer:
            self.timer.cancel()
        self._write(0)
        if self.backend == "pigpio":
            self.handle.stop()
        elif self.backend == "lgpio":
            import lgpio
            lgpio.gpiochip_close(self.handle)


class RelayClient:
    """인터넷 중계소를 구독해 조명 명령을 받는다. **설치할 패키지가 없다.**

    왜 중계소인가: PC와 파이가 서로 다른 공유기에 있으면 둘 다 NAT 뒤라
    서로에게 직접 갈 수 없다. 그래서 양쪽이 **바깥으로 나가서** 공용 중계소
    한 곳에서 만난다 — 나가는 연결은 웹 브라우징과 같아서 공유기 설정이
    필요 없다. 포트포워딩도, 터널도, 고정 IP도 없다.

    구독은 그냥 긴 HTTP GET이다(서버가 메시지를 한 줄씩 흘려보낸다). 그래서
    파이썬 표준 라이브러리(urllib)만으로 되고, pip이나 apt로 뭔가를 깔 필요가
    없다 — 라즈베리파이 OS의 PEP 668 제한("externally-managed-environment")에
    걸릴 일 자체가 없다.

    서버 쪽 짝은 `src/api/relay.py`이고 메시지 형식도 거기 적혀 있다.
    """

    def __init__(self, servo, url, topic):
        self.servo = servo
        self.url = url.rstrip("/")
        self.topic = topic
        self.stop_flag = threading.Event()

    def start(self):
        """구독 루프와 생존 신호를 백그라운드로 돌린다(로컬 HTTP 서버와 병행)."""
        threading.Thread(target=self._subscribe_forever, daemon=True).start()
        threading.Thread(target=self._heartbeat_forever, daemon=True).start()
        print(f"[relay] 중계소 구독: {self.url}/{self.topic}")

    # ------------------------------------------------------------------
    def _publish(self, payload):
        """응답 주제로 한 줄 보낸다(명령 결과, 생존 신호).

        명령을 받는 주제와 분리해 두어야 자기가 보낸 메시지를 자기가 다시
        받아 걸러내는 일이 없다.
        """
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(f"{self.url}/{self.topic}-ack", data=data)
        try:
            urllib.request.urlopen(request, timeout=10).close()
        except Exception as e:
            print(f"[relay] 응답 전송 실패: {e}")

    def _heartbeat_forever(self):
        """살아 있다고 주기적으로 알린다.

        이게 있어야 PC 대시보드에서 조명 버튼을 누르기 **전에** 파이가 켜져
        있는지 확인할 수 있다. 눌러 보고 나서야 아는 것보다 시연에 안전하다.
        """
        while not self.stop_flag.is_set():
            self._publish({"hello": "pi", "mode": self.servo.backend})
            self.stop_flag.wait(30)

    def _subscribe_forever(self):
        """명령 주제를 계속 구독한다. 끊기면 다시 붙는다.

        중계소는 유휴 연결을 주기적으로 끊고 와이파이도 언제든 끊긴다.
        사람이 신경 쓸 일이 아니므로 여기서 알아서 다시 연결한다.
        """
        url = f"{self.url}/{self.topic}/json"
        while not self.stop_flag.is_set():
            try:
                with urllib.request.urlopen(url, timeout=None) as stream:
                    print("[relay] 연결됨 — 명령을 기다립니다")
                    for line in stream:
                        if self.stop_flag.is_set():
                            return
                        self._handle(line)
            except Exception as e:
                print(f"[relay] 연결 끊김({e}) — 3초 후 재시도")
            self.stop_flag.wait(3)

    def _handle(self, line):
        """중계소가 흘려보내는 JSON 한 줄을 처리한다.

        중계소는 실제 메시지 외에 연결 유지용 신호도 보내므로 우리가 만든
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
        if msg.get("cmd") != "press":
            return

        command_id = msg.get("id")
        try:
            angle = float(msg.get("angle", 90))
            self.servo.press(angle, float(msg.get("rest", 90)), int(msg.get("hold_ms", 250)))
            print(f"[relay] 눌렀습니다 — {angle:.0f}도")
            self._publish({"ack": command_id, "ok": True, "mode": self.servo.backend})
        except Exception as e:
            # 실패도 반드시 알린다 — 안 보내면 PC 쪽은 "응답 없음"이라는 덜
            # 정확한 이유만 사용자에게 보여주게 된다.
            print(f"[relay] 실패: {e}")
            self._publish({"ack": command_id, "ok": False, "error": str(e)})


app = Flask(__name__)
servo = Servo(PIN)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/servo", methods=["POST"])
def move():
    """{"angle": 90} 을 받아 서보를 움직인다."""
    data = request.get_json(force=True, silent=True) or {}
    try:
        angle = float(data["angle"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "angle이 필요합니다 (0~180)"}), 400
    return jsonify({"ok": True, "angle": servo.goto(angle), "mode": servo.backend})


@app.route("/api/press", methods=["POST"])
def press():
    """{"angle": 140} — 밀었다 떼기. 벽 스위치용."""
    data = request.get_json(force=True, silent=True) or {}
    try:
        angle = float(data["angle"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "angle이 필요합니다 (0~180)"}), 400
    servo.press(angle, float(data.get("rest", 90)), int(data.get("hold_ms", 250)))
    return jsonify({"ok": True, "angle": angle, "mode": servo.backend})


@app.route("/api/status")
def status():
    return jsonify({"angle": servo.angle, "mode": servo.backend, "pin": PIN})


if __name__ == "__main__":
    # 인터넷 중계 — 주제가 있으면 시작한다. 없으면 로컬 HTTP 서버만.
    if RELAY_TOPIC:
        RelayClient(servo, RELAY_URL, RELAY_TOPIC).start()
    else:
        print("[relay] DESKY_RELAY_TOPIC이 없어 로컬 HTTP 서버만 실행합니다 "
              "(PC와 같은 공유기에 있어야 조명이 동작합니다)")

    try:
        # host="0.0.0.0" 이어야 폰에서 접속된다. 기본값(127.0.0.1)이면
        # 파이 자기 자신에서만 열린다 — 가장 흔히 걸리는 지점이다.
        app.run(host="0.0.0.0", port=LOCAL_PORT)
    except OSError as e:
        # 포트가 이미 쓰이는 등의 이유로 로컬 화면을 못 띄우는 경우.
        # 조명 제어는 중계소로 들어오므로 여기서 프로세스를 끝내면 안 된다 —
        # 곁다리 기능 하나 때문에 본 기능이 같이 죽는 셈이 된다.
        print(f"[web] 로컬 화면을 열지 못했습니다({e}). "
              f"다른 포트를 쓰려면 PI_PORT=5001 처럼 지정하세요.")
        if RELAY_TOPIC:
            print("[web] 중계소를 통한 조명 제어는 계속 동작합니다. 종료: Ctrl+C")
            try:
                threading.Event().wait()      # 중계 스레드가 계속 돌도록 유지
            except KeyboardInterrupt:
                pass
    finally:
        servo.close()
