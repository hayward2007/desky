#!/usr/bin/env python3
"""라즈베리파이 서보 웹 컨트롤 — 최소 예제.

파이 한 대에서 웹서버와 서보 제어를 모두 한다. 폰이나 PC 브라우저로
http://<파이IP>:5000 에 접속하면 버튼으로 서보를 돌릴 수 있다.

설치:
    sudo apt install -y python3-flask
    # 파이 4 이하
    sudo apt install -y pigpio python3-pigpio && sudo systemctl enable --now pigpiod
    # 파이 5
    sudo apt install -y python3-lgpio

실행:
    python3 server.py
    # 다른 기기에서 접속하려면 파이 IP를 확인: hostname -I

GPIO 라이브러리가 없어도 실행된다(모의 모드). 하드웨어 없이 웹 화면을 먼저
확인할 수 있으므로, 배선 문제와 웹 문제를 분리해서 디버깅할 수 있다.
"""

import threading

from flask import Flask, jsonify, render_template, request


PIN = 18                # BCM 번호 (12번 물리 핀)
PULSE_MIN_US = 500      # 0도
PULSE_MAX_US = 2500     # 180도
DETACH_AFTER_S = 0.5    # 이동 후 신호를 끊기까지의 시간


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
            for chip in (0, 4):        # 펌웨어에 따라 칩 번호가 다르다
                try:
                    h = lgpio.gpiochip_open(chip)
                    print(f"[gpio] lgpio 사용 (chip {chip})")
                    return "lgpio", h
                except Exception:
                    continue
        except ImportError:
            pass

        print("[gpio] 라이브러리 없음 — 모의 모드로 실행합니다(서보는 안 움직임)")
        return "mock", None

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
    try:
        # host="0.0.0.0" 이어야 폰에서 접속된다. 기본값(127.0.0.1)이면
        # 파이 자기 자신에서만 열린다 — 가장 흔히 걸리는 지점이다.
        app.run(host="0.0.0.0", port=5000)
    finally:
        servo.close()
