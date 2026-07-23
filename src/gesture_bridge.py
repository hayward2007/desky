"""가위바위보 제스처를 실제 동작(폰 명령 + 조명)에 연결하는 다리 객체.

[병합 메모] 병합 전에는 mobile 브랜치 `src/app.py`의 `_make_gesture_recognizer()`
라는 클로저 팩토리였고, `camera`/`light` 전역을 캡처해서 썼다. 병합하면서
전역이 사라졌으므로(모두 생성자 주입) 클래스로 바꿨다.

왜 '다리'가 필요한가:
- 제스처가 확정되는 곳: **서버**의 인식 루프(perception.gesture).
- 실제로 실행돼야 하는 곳: **폰**(카메라·마이크·화면이 거기 있다) 또는
  **라즈베리파이**(조명 스위치).
따라서 콜백은 직접 뭔가를 실행하지 않고, 이미 열려 있는 `/ws/camera` 소켓으로
명령 한 줄을 되돌려 보낸다(`Camera.broadcast`) — transcript를 폰으로 보내는
것과 같은 경로다. 조명만은 폰을 거칠 필요가 없어 서버가 파이를 직접 부른다.

  가위(SCISSORS) → 폰: 문서 스캔 시작
  바위(ROCK)     → 폰: 음성 인식(대화 세션) 시작
  보(PAPER)      → 폰: 카메라 끄기  +  파이: 조명 끄기
"""

from fundamental.const import GestureConst
from fundamental.logger import Logger
from perception.gesture import Gesture, GestureRecognizer, draw_gesture


class GestureBridge:
    """제스처 인식기 + 그 결과를 폰/조명으로 내보내는 배선.

    `light`는 선택이다 — 파이가 없으면 None으로 두면 조명 부분만 빠지고
    나머지 제스처는 그대로 동작한다(프로젝트 전반의 "기능만 빠지고 앱은 계속
    뜬다" 패턴).
    """

    def __init__(self, camera, light=None, enabled=True):
        self.camera = camera
        self.light = light
        self.enabled = enabled
        self.recognizer = self._build_recognizer()

    # ------------------------------------------------------------------
    # 배선
    # ------------------------------------------------------------------
    def _build_recognizer(self) -> GestureRecognizer:
        """제스처 → 동작 표를 붙인 인식기를 만든다(임계값은 GestureConst)."""
        return GestureRecognizer(
            actions={
                Gesture.SCISSORS: self._action("scan"),                    # 가위
                Gesture.ROCK: self._action("voice"),                       # 바위
                Gesture.PAPER: self._action("camera_off", light="off"),    # 보
            },
            hold_frames=GestureConst.HOLD_FRAMES,
            hold_overrides={Gesture.PAPER: GestureConst.HOLD_FRAMES_PAPER},
            cooldown_s=GestureConst.COOLDOWN_S,
        )

    def _action(self, action, light=None):
        """폰에 `action`을 보내고(필요하면) 조명도 바꾸는 콜백을 만든다."""
        def _fire():
            if self.camera.broadcast({"type": "gesture", "action": action}) == 0:
                Logger.log("GESTURE", f"{action}: no mobile client connected")
            if light and self.light is not None:
                # 파이 호출은 네트워크 대기가 있으므로 별도 스레드로 보낸다 —
                # 여기는 카메라 루프 안이라, 기다리면 영상이 그대로 멈춘다.
                self.light.set_async(light)
        return _fire

    # ------------------------------------------------------------------
    # 프레임 처리
    # ------------------------------------------------------------------
    def update(self, hands, frame_shape):
        """프레임마다 호출 — 확정된 제스처를 돌려주고 필요하면 동작을 발동한다.

        `hands`는 `perception.hand_tracker.HandTracker`가 이미 만들어 둔 결과라
        인식을 한 번 더 돌리지 않는다. 꺼져 있으면 아무 일도 하지 않고
        `Gesture.NONE`을 돌려준다.
        """
        if not self.enabled:
            return Gesture.NONE
        return self.recognizer.update(hands, frame_shape)

    def draw(self, frame_bgr, gesture):
        """확정된 제스처를 카메라 미리보기 창에 글자로 표시한다(제자리 수정)."""
        return draw_gesture(frame_bgr, gesture)

    def set_enabled(self, enabled: bool):
        """제스처 기능을 켜고 끈다. 끌 때는 확정 상태도 함께 비운다.

        상태를 비우지 않으면, 껐다 켠 뒤 같은 모양을 다시 들어도 '이미 확정된
        값'이라 발동하지 않는다(엣지 트리거이므로).
        """
        self.enabled = enabled
        if not enabled:
            self.recognizer.reset()
