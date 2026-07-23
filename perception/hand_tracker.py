"""MediaPipe 손 인식 + 인식된 손을 3D 월드 좌표로 올리는 모듈.

핵심 아이디어: 휴대폰 카메라는 end-effector에 달려 있으므로, 손의 "화면상 크기"
로부터 카메라까지의 거리를 역산하고(핀홀 모델), 그 거리를 이용해 손 랜드마크를
end-effector 좌표계 → 월드 좌표계로 변환한다. 그러면 로봇팔이 움직여도 손이
3D 씬 안의 올바른 위치에 그려진다.

의존성: mediapipe, opencv-python, numpy
"""

import math

from logger import Logger


class Hand:
    """인식된 손 하나. `landmarks`는 mediapipe의 21개 랜드마크,
    `depth`는 추정된 카메라~손 거리(m), `world_points`는 월드 좌표 21개."""

    def __init__(self, landmarks, depth, world_points):
        """landmarks: mediapipe 랜드마크 객체, depth: 추정 거리(m),
        world_points: 월드 좌표 (x, y, z) 21개."""
        self.landmarks = landmarks
        self.depth = depth
        self.world_points = world_points


class HandTracker:
    """카메라 프레임에서 손을 찾아 3D 월드 좌표로 변환하는 객체.

    `with HandTracker() as tracker:` 형태로 쓰면 mediapipe 세션이 결정적으로
    정리된다. mediapipe가 설치돼 있지 않으면 `available`이 False가 되고
    `process()`는 항상 빈 리스트를 돌려준다 — 하드웨어/Gemini 미구성 때와
    같은 "기능만 빠지고 앱은 계속 뜬다" 패턴.
    """

    # 이미지 '가로'로 정규화한 핀홀 초점거리. 즉 거리 d에 있는, 광축에 수직으로
    # o만큼 떨어진 점은 화면 중심에서 o / (d / FOCAL_NORM) 만큼 떨어져 보인다.
    # 1.0은 대략 수평 화각 53도로, 일반적인 휴대폰 카메라의 어림값이다.
    # 실제 캘리브레이션을 한 값이 아니므로 아래 거리 추정은 측정이 아니라 추정치다.
    FOCAL_NORM = 1.0

    # 손목(랜드마크 0) ~ 엄지 CMC(랜드마크 1) 실제 길이. 거리 역산의 기준자.
    WRIST_TO_THUMB_CMC_M = 0.035

    # 손바닥 사각형을 그릴 때 잇는 랜드마크 (손목 - 엄지CMC - 검지MCP - 새끼MCP)
    PALM_QUAD = [0, 1, 5, 17]

    def __init__(self, max_num_hands=2, min_detection_confidence=0.5,
                 min_tracking_confidence=0.5):
        """mediapipe Hands 세션을 만든다. mediapipe import에 실패하면
        조용히 비활성 상태(`available == False`)로 남는다."""
        self.hands = None
        self.error = None
        self._mp = None
        self._drawing = None
        try:
            import mediapipe as mp

            self._mp = mp.solutions.hands
            self._drawing = mp.solutions.drawing_utils
            self.hands = self._mp.Hands(
                static_image_mode=False,
                max_num_hands=max_num_hands,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            Logger.log("HAND", "MediaPipe hand tracker ready")
        except Exception as e:
            self.error = str(e)
            Logger.log("HAND", f"Hand tracking disabled: {self.error}")

    @property
    def available(self) -> bool:
        """mediapipe 세션이 정상적으로 만들어졌는지 여부."""
        return self.hands is not None

    @property
    def connections(self):
        """3D 씬에서 랜드마크를 잇는 데 쓰는 mediapipe의 손 골격 연결 목록."""
        return self._mp.HAND_CONNECTIONS if self._mp is not None else None

    def __enter__(self):
        """컨텍스트 매니저 진입 — 그냥 자기 자신을 돌려준다."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """컨텍스트 매니저 종료 — mediapipe 세션을 닫는다."""
        self.close()

    def close(self):
        """mediapipe Hands 세션을 닫는다(내부 그래프/스레드 정리)."""
        if self.hands is not None:
            self.hands.close()
            self.hands = None

    # ------------------------------------------------------------------
    # 인식
    # ------------------------------------------------------------------
    def process(self, frame_rgb, T_ee, frame_shape) -> list:
        """RGB 프레임 한 장에서 손을 찾아 `Hand` 리스트로 반환한다.

        frame_rgb  : cv2.COLOR_BGR2RGB로 변환한 프레임 (mediapipe는 RGB를 받는다)
        T_ee       : 현재 end-effector의 4x4 월드 변환 행렬 (Arm.fk_matrix(q))
        frame_shape: (height, width, channels) — 정규화 좌표를 픽셀로 되돌릴 때 사용
        """
        if not self.available:
            return []

        results = self.hands.process(frame_rgb)
        if not results.multi_hand_landmarks:
            return []

        height, width = frame_shape[0], frame_shape[1]
        hands = []
        for hand_landmarks in results.multi_hand_landmarks:
            depth = self.estimate_depth(hand_landmarks.landmark, width, height)
            world_points = [self.landmark_to_world(lm, T_ee, depth)
                            for lm in hand_landmarks.landmark]
            hand = Hand(hand_landmarks, depth, world_points)
            hands.append(hand)
        return hands

    def estimate_depth(self, landmarks, frame_width, frame_height) -> float:
        """손이 카메라에서 얼마나 떨어져 있는지(m)를 '화면에 보이는 크기'로 추정한다.

        손이 작아 보이면 줄어든 게 아니라 멀어진 것 — 실제 손목~엄지CMC 길이가
        WRIST_TO_THUMB_CMC_M라고 가정하고 핀홀 투영식
        (보이는 크기 = 초점거리 x 실제 크기 / 거리)을 거리에 대해 역산한다.
        """
        wrist, thumb_cmc = landmarks[0], landmarks[1]
        dx = (thumb_cmc.x - wrist.x) * frame_width
        dy = (thumb_cmc.y - wrist.y) * frame_height
        apparent = math.hypot(dx, dy) / frame_width
        apparent = max(apparent, 1e-4)  # 검출이 뭉개져 0에 가까울 때의 0 나눗셈 방지
        return self.FOCAL_NORM * self.WRIST_TO_THUMB_CMC_M / apparent

    def landmark_to_world(self, landmark, T_ee, depth) -> tuple:
        """랜드마크 하나를 월드 좌표 (x, y, z)로 변환한다.

        T_ee의 회전행렬 열벡터를 그대로 end-effector 로컬 축의 월드 방향으로
        쓴다: forward_axis(로컬 +Y = 카메라가 실제로 보는 방향 — 실측으로 확인:
        모든 관절이 서보각 180도일 때 이 벡터가 world +Z를 가리켜야 함),
        up_axis(로컬 +X), side_axis(로컬 +Z). 손 랜드마크가 놓이는 평면은 항상
        origin + forward * forward_axis를 지나고 forward_axis에 수직이므로,
        그 평면의 방향벡터(법선)는 정확히 end-effector의 방향벡터와 같다 —
        팔이 회전하면 손 평면도 그대로 같이 회전한다. 화면 중심(landmark
        (0.5, 0.5))의 점은 항상 이 forward_axis 직선 위에 정확히 놓인다.

        landmark.z(손 안에서의 상대 깊이, 예: 구부린 손가락)는 이 랜드마크만의
        forward 거리를 조정하고, screen_right/screen_down도 공유된 depth가
        아니라 그 forward 거리 기준으로 다시 스케일해 핀홀 투영과 일치시킨다.
        """
        forward_axis = (T_ee[0][1], T_ee[1][1], T_ee[2][1])
        up_axis = (T_ee[0][0], T_ee[1][0], T_ee[2][0])
        side_axis = (T_ee[0][2], T_ee[1][2], T_ee[2][2])
        origin = (T_ee[0][3], T_ee[1][3], T_ee[2][3])

        scale = depth / self.FOCAL_NORM
        forward = depth + landmark.z * scale
        scale_at_forward = forward / self.FOCAL_NORM
        screen_right = (landmark.x - 0.5) * scale_at_forward
        screen_down = (landmark.y - 0.5) * scale_at_forward

        # 화면 <-> 로컬 축 매핑: forward_axis를 축으로 시계방향 90도 회전 보정
        # (카메라 시점, 즉 forward_axis 바깥쪽을 내다보는 기준):
        # screen_right -> -side_axis, screen_down -> +up_axis.
        return tuple(
            origin[i] + forward * forward_axis[i]
            + screen_down * up_axis[i] - screen_right * side_axis[i]
            for i in range(3)
        )

    # ------------------------------------------------------------------
    # 시각화
    # ------------------------------------------------------------------
    def draw_overlay(self, frame_bgr, hands):
        """카메라 프레임 위에 손 골격과 손바닥 사각형을 그린다(제자리 수정).

        mediapipe 기본 랜드마크/연결선에 더해, PALM_QUAD 네 점을 이어 손바닥
        영역을 초록 사각형으로 표시한다 — 손이 카메라를 향하는지 눈으로
        확인하기 쉽게 하기 위한 보조선.
        """
        if not self.available:
            return frame_bgr
        import cv2

        for hand in hands:
            self._drawing.draw_landmarks(
                frame_bgr, hand.landmarks, self._mp.HAND_CONNECTIONS)

            points = [hand.landmarks.landmark[i] for i in self.PALM_QUAD]
            coords = [(int(p.x * frame_bgr.shape[1]), int(p.y * frame_bgr.shape[0]))
                      for p in points]
            for a, b in zip(coords, coords[1:] + coords[:1]):
                cv2.line(frame_bgr, a, b, (0, 255, 0), 2)
        return frame_bgr
