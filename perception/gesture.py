"""가위/바위/보 손모양 인식 — 손 랜드마크 21개를 세 가지 제스처로 분류한다.

  가위(SCISSORS) → 문서 스캔 시작
  바위(ROCK)     → 음성 인식(대화 세션) 시작
  보(PAPER)      → 카메라 끄기 (+ 조명 끄기)

[병합 메모] 이 모듈은 원래 서버에서 mediapipe가 직접 찾은 손을 분류했다.
병합 후에는 손 인식 자체가 휴대폰으로 옮겨갔지만(perception.hand_tracker 모듈
docstring 참고) 이 모듈은 **그대로 재사용된다** — 분류에 필요한 건 "x, y를 가진
랜드마크 21개"뿐이고, 그 21개는 서버 mediapipe든 휴대폰의 MediaPipe Tasks
Vision이든 같은 손 모델의 같은 인덱스이기 때문이다. 즉 이 모듈은
`perception.hand_tracker.HandTracker.process_landmarks()`가 이미 만들어 둔
`Hand` 목록을 그대로 받아 쓰므로 인식을 한 번 더 돌리지 않는다(추가 비용 0).

판정은 손목(랜드마크 0) 기준 '상대 거리'로 하기 때문에 손이 기울어져 있어도
(= 로봇팔이 돌아가 있어도) 비교적 안정적이다. 화면 정규화 좌표는 x가 가로,
y가 세로로 각각 다른 픽셀 수에 매핑돼 있으므로 x에 종횡비를 곱해 왜곡을 편다.

같은 모양이 연속 N프레임 잡혔을 때만 '확정'하고, 확정값이 바뀌는 순간에만
콜백을 1회 부른다(엣지 트리거). 손을 계속 들고 있어도 반복 실행되지 않는다.

의존성: 없음(표준 라이브러리만). 화면 표시 함수만 opencv-python을 쓴다.
"""

import math
import time
from enum import Enum

from fundamental.const import GestureConst
from fundamental.logger import Logger


class Gesture(str, Enum):
    """인식 결과. NONE은 손 없음, UNKNOWN은 손은 있으나 셋 중 어느 것도 아님."""

    NONE = "none"
    ROCK = "rock"
    PAPER = "paper"
    SCISSORS = "scissors"
    UNKNOWN = "unknown"


# 손 랜드마크 인덱스 — 설명은 fundamental.const.GestureConst 참고.
WRIST = GestureConst.WRIST
THUMB_IP, THUMB_TIP = GestureConst.THUMB_IP, GestureConst.THUMB_TIP
MIDDLE_MCP, PINKY_MCP = GestureConst.MIDDLE_MCP, GestureConst.PINKY_MCP
FINGER_PIP = GestureConst.FINGER_PIP   # 검지, 중지, 약지, 새끼의 두 번째 관절
FINGER_TIP = GestureConst.FINGER_TIP   # 같은 순서의 손끝


def _points(landmarks, aspect=1.0):
    """랜드마크를 종횡비 보정한 (x, y) 리스트로 바꾼다.

    `landmarks`의 원소는 `.x`, `.y`를 가진 무엇이든 된다 — 휴대폰이 보낸 값을
    감싼 `perception.hand_tracker._Landmark`(namedtuple)도, 예전 서버 mediapipe의
    NormalizedLandmark도 같은 속성 이름을 쓴다.
    """
    return [(lm.x * aspect, lm.y) for lm in landmarks]


def _dist(a, b):
    """(x, y) 두 점 사이의 거리."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finger_states(pts, margin=GestureConst.EXTENDED_MARGIN):
    """검지·중지·약지·새끼가 펴졌는지 여부 4개.

    손끝이 두 번째 관절보다 손목에서 더 멀면 편 것으로 본다. 손목 기준 거리라
    손 방향이 어디를 향하든(위/아래/옆) 같은 기준이 통한다. `margin`은 애매한
    반쯤 굽힌 상태에서 값이 파닥이지 않게 하는 여유분.
    """
    wrist = pts[WRIST]
    return [_dist(pts[tip], wrist) > _dist(pts[pip], wrist) * margin
            for pip, tip in zip(FINGER_PIP, FINGER_TIP)]


def thumb_extended(pts, margin=GestureConst.EXTENDED_MARGIN):
    """엄지가 펴졌는지 여부.

    엄지는 다른 손가락과 달리 옆으로 벌어지므로 손목이 아니라 새끼 MCP를
    기준점으로 쓴다 — 펴면 새끼 쪽에서 멀어지고, 접으면 손바닥 위로 붙는다.
    좌우 손 구분이 필요 없다는 것도 이 방식의 장점.
    """
    ref = pts[PINKY_MCP]
    return _dist(pts[THUMB_TIP], ref) > _dist(pts[THUMB_IP], ref) * margin


def classify(landmarks, aspect=1.0) -> Gesture:
    """랜드마크 21개를 가위/바위/보 중 하나로 분류한다."""
    if landmarks is None or len(landmarks) < 21:
        return Gesture.NONE

    pts = _points(landmarks, aspect)
    index, middle, ring, pinky = finger_states(pts)
    open_count = sum((index, middle, ring, pinky))

    if open_count == 0:
        return Gesture.ROCK                      # 주먹
    if index and middle and not ring and not pinky:
        return Gesture.SCISSORS                  # 브이
    if open_count == 4 or (open_count == 3 and thumb_extended(pts)):
        return Gesture.PAPER                     # 손바닥 (약지 하나쯤 덜 펴져도 인정)
    return Gesture.UNKNOWN


def _landmarks_of(hand):
    """`Hand` 객체(또는 랜드마크 목록 자체)에서 21개 랜드마크를 꺼낸다.

    병합으로 손 인식이 휴대폰으로 옮겨가면서 `Hand.landmarks`의 타입이
    mediapipe의 NormalizedLandmarkList에서 `_Landmark`(namedtuple) 리스트로
    바뀌었다. 두 경우 모두 여기서 흡수하므로, 나중에 서버 인식으로 되돌리거나
    다른 소스를 붙여도 이 아래 판정 코드는 손댈 필요가 없다.
    """
    obj = getattr(hand, "landmarks", hand)     # perception.hand_tracker.Hand
    obj = getattr(obj, "landmark", obj)        # (구) mediapipe NormalizedLandmarkList
    try:
        return list(obj)
    except TypeError:
        return None


def classify_hands(hands, frame_shape=None) -> Gesture:
    """여러 손이 잡히면 화면에 가장 크게(=가장 가깝게) 보이는 손 하나만 쓴다."""
    if not hands:
        return Gesture.NONE

    aspect = 1.0
    if frame_shape is not None and frame_shape[0]:
        aspect = frame_shape[1] / frame_shape[0]

    best, best_size = None, -1.0
    for hand in hands:
        lms = _landmarks_of(hand)
        if lms is None or len(lms) < 21:
            continue
        pts = _points(lms, aspect)
        size = _dist(pts[WRIST], pts[MIDDLE_MCP])   # 손바닥 길이 = 화면상 손 크기
        if size > best_size:
            best, best_size = lms, size

    return classify(best, aspect) if best is not None else Gesture.NONE


class GestureRecognizer:
    """제스처를 안정화한 뒤 동작을 딱 한 번 발동시키는 디스패처.

    actions        : {Gesture: 콜백} — 확정된 순간 호출된다
    hold_frames    : 같은 모양이 몇 프레임 연속 잡혀야 확정할지
    hold_overrides : 제스처별 hold_frames 예외. 카메라 끄기(보)처럼 되돌리기
                     번거로운 동작은 더 오래 들고 있게 해서 오작동을 줄인다.
    cooldown_s     : 어떤 동작이든 직전 실행 후 이 시간 안에는 다시 실행 안 함
    idle_reset_s   : 이만큼 프레임이 끊기면(예: 카메라를 껐다 켬) 상태 초기화

    기본값은 fundamental.const.GestureConst에 있다 — 휴대폰이 프레임마다
    랜드마크를 보내는 지금의 프레임레이트에 맞춰 잡은 값이다.
    """

    def __init__(self, actions=None, hold_frames=GestureConst.HOLD_FRAMES,
                 hold_overrides=None, cooldown_s=GestureConst.COOLDOWN_S,
                 idle_reset_s=GestureConst.IDLE_RESET_S):
        self.actions = actions or {}
        self.hold_frames = hold_frames
        self.hold_overrides = hold_overrides or {}
        self.cooldown_s = cooldown_s
        self.idle_reset_s = idle_reset_s

        self.stable = Gesture.NONE
        self._candidate = Gesture.NONE
        self._count = 0
        self._last_fire = 0.0
        self._last_update = 0.0

    def _needed(self, gesture):
        """이 제스처를 확정하는 데 필요한 연속 프레임 수."""
        return self.hold_overrides.get(gesture, self.hold_frames)

    def reset(self):
        """확정 상태를 지운다. 같은 제스처를 다시 하면 또 발동할 수 있게 된다."""
        self.stable = self._candidate = Gesture.NONE
        self._count = 0

    def update(self, hands, frame_shape=None) -> Gesture:
        """프레임마다 호출. 확정된 제스처를 돌려주고, 바뀌었으면 콜백을 부른다."""
        now = time.monotonic()
        if self._last_update and now - self._last_update > self.idle_reset_s:
            # 카메라가 꺼졌다 켜진 경우 — 껐을 때의 '보'가 확정 상태로 남아
            # 있으면 다시 켜고 보를 내밀어도 반응하지 않으므로 초기화한다.
            self.reset()
        self._last_update = now

        gesture = classify_hands(hands, frame_shape)

        if gesture == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = gesture, 1

        if self._count >= self._needed(gesture) and gesture != self.stable:
            self.stable = gesture
            self._fire(gesture)
        return self.stable

    def _fire(self, gesture):
        """확정된 제스처의 콜백을 쿨다운을 지켜 한 번 호출한다."""
        action = self.actions.get(gesture)
        if action is None:
            return
        now = time.monotonic()
        if now - self._last_fire < self.cooldown_s:
            Logger.log("GESTURE", f"{gesture.value} ignored (cooldown)")
            return
        self._last_fire = now
        Logger.log("GESTURE", f"{gesture.value} → action")
        try:
            action()
        except Exception as e:
            # 콜백이 터져도 카메라 루프는 계속 돌아야 한다.
            Logger.log("GESTURE", f"{gesture.value} action failed: {e}")


# ----------------------------------------------------------------------
# 화면 표시
# ----------------------------------------------------------------------

# cv2.putText는 한글을 못 그리므로 로마자 라벨을 쓴다.
LABELS = {
    Gesture.ROCK: "ROCK -> voice",
    Gesture.PAPER: "PAPER -> camera off",
    Gesture.SCISSORS: "SCISSORS -> scan",
    Gesture.UNKNOWN: "hand (?)",
    Gesture.NONE: "",
}


def draw_gesture(frame_bgr, gesture, org=(12, 34)):
    """인식된 제스처를 카메라 미리보기 좌상단에 표시한다(제자리 수정)."""
    import cv2

    text = LABELS.get(gesture, "")
    if not text:
        return frame_bgr
    color = (0, 200, 255) if gesture is Gesture.UNKNOWN else (0, 255, 0)
    cv2.putText(frame_bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(frame_bgr, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
    return frame_bgr
