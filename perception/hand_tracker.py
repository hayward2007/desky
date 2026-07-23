"""휴대폰이 인식한 손 랜드마크를 3D 월드 좌표로 올리는 모듈.

손 인식(MediaPipe HandLandmarker) 자체는 더 이상 서버가 하지 않는다 —
`src/templates/mobile.html`이 휴대폰에서 직접 MediaPipe Tasks Vision을 돌려
21개 랜드마크만(이미지 아님) 웹소켓(`/ws/camera`)으로 보내고, `src.api.camera.Camera`가
그걸 받아 저장한다. 얼굴 인식(perception.face_tracker)도 같은 이유로 같은 방식으로
휴대폰이 한다 — 컴퓨터 하나가 얼굴+손 인식을 mediapipe로 전부 처리하면 휴대폰이
실제로 보내는 ~20fps를 못 따라가 로봇 반응이 느려지고 뚝뚝 끊겼고, 압축/전송을
거친 프레임은 화질도 떨어져 인식 자체가 잘 안 되는 문제도 있었다. 서버는 이제
어느 쪽도 모델 추론을 하지 않고, 휴대폰이 보낸 랜드마크의 좌표 변환만 한다.

핵심 아이디어(인식 주체가 바뀌어도 동일): 휴대폰 카메라는 end-effector에 달려
있으므로, 손의 "화면상 크기"로부터 카메라까지의 거리를 역산하고(핀홀 모델),
그 거리를 이용해 손 랜드마크를 end-effector 좌표계 → 월드 좌표계로 변환한다.
그러면 로봇팔이 움직여도 손이 3D 씬 안의 올바른 위치에 그려진다.

`HandTracker`(좌표변환 + 시각화 — 랜드마크는 이미 계산되어 들어온다)와
`HandFollower`(손 위치를 따라 로봇이 이동할 목표 계산)로 나뉜다. 실제 하드웨어
이동 명령(IK 포함)은 이 모듈이 하지 않는다 — perception 패키지는 하드웨어에
의존하지 않으므로, `HandFollower`는 "어디로, 언제" 움직여야 할지만 계산하고
실제 `arm_ctrl.goto_position()` 호출(IK + 서보 명령)은 호출부(`src/app.py`)가
한다.

의존성: opencv-python, numpy, matplotlib(3D 시각화, kinematics.simulate).
mediapipe는 더 이상 이 모듈의 의존성이 아니다(인식이 클라이언트로 옮겨갔으므로).
"""

import math

from kinematics.simulate import draw_points
from fundamental.const import CameraGeometryConst, HandTrackerConst, HandFollowerConst
from perception.camera_geometry import camera_frame as _camera_frame
from perception.camera_geometry import clamp_xy as _clamp_xy
from perception.camera_geometry import to_landmark as _to_landmark

# 표준 MediaPipe 21점 손 골격 연결 — 예전에는 mediapipe 라이브러리가 이미
# 만들어 둔 `mp.solutions.hands.HAND_CONNECTIONS`를 그대로 썼지만, 서버가
# 더 이상 mediapipe에 의존하지 않으므로 같은 위상(손목-엄지-검지-중지-약지-
# 새끼)을 직접 하드코딩했다. 값 자체는 MediaPipe 손 모델 표준 정의와 동일.
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # 엄지
    (0, 5), (5, 6), (6, 7), (7, 8),          # 검지
    (5, 9), (9, 10), (10, 11), (11, 12),     # 중지
    (9, 13), (13, 14), (14, 15), (15, 16),   # 약지
    (13, 17), (17, 18), (18, 19), (19, 20),  # 새끀
    (0, 17),                                  # 손목 - 새끼 (손바닥 밑변)
]


class Hand:
    """인식된 손 하나. `landmarks`는 `camera_geometry.Landmark`(x, y, z) 21개,
    `depth`는 추정된 카메라~손 거리(m), `world_points`는 월드 좌표 21개,
    `center`는 손바닥 중심(PALM_QUAD 네 점의 평균 월드 좌표),
    `screen_offset`은 그 중심이 화면 정중앙 (0.5, 0.5)에서 얼마나 벗어났는지
    (dx, dy), 정규화 이미지 좌표 기준(월드 변환 이전 원본 랜드마크에서 계산)."""

    def __init__(self, landmarks, depth, world_points, center, screen_offset):
        """landmarks: `camera_geometry.Landmark` 21개, depth: 추정 거리(m),
        world_points: 월드 좌표 (x, y, z) 21개, center: 손바닥 중심 (x, y, z),
        screen_offset: 화면 중앙 대비 오프셋 (dx, dy), 정규화 좌표."""
        self.landmarks = landmarks
        self.depth = depth
        self.world_points = world_points
        self.center = center
        self.screen_offset = screen_offset


class HandTracker:
    """휴대폰이 보낸 손 랜드마크를 3D 월드 좌표로 변환하고 시각화하는 객체.

    인식(모델 추론) 자체는 하지 않는다 — `process_landmarks()`가 이미 계산된
    랜드마크(휴대폰의 MediaPipe Tasks Vision HandLandmarker 결과)를 받아 좌표
    변환/깊이 추정만 한다. 랜드마크가 없으면(휴대폰이 아직 안 보냈거나, 화면에
    손이 없어서 빈 리스트를 보낸 경우) 항상 빈 리스트를 돌려준다 — 하드웨어/
    Gemini 미구성 때와 같은 "기능만 빠지고 앱은 계속 뜬다" 패턴.
    """

    # 상수 설명은 fundamental.const.HandTrackerConst 참고.
    FOCAL_NORM = CameraGeometryConst.FOCAL_NORM
    WRIST_TO_THUMB_CMC_M = HandTrackerConst.WRIST_TO_THUMB_CMC_M
    PALM_QUAD = HandTrackerConst.PALM_QUAD

    @property
    def connections(self):
        """3D 씬에서 랜드마크를 잇는 데 쓰는 손 골격 연결 목록."""
        return HAND_CONNECTIONS

    # ------------------------------------------------------------------
    # 인식(휴대폰이 이미 계산한 랜드마크를 좌표 변환만)
    # ------------------------------------------------------------------
    def process_landmarks(self, raw_hands, T_ee, frame_shape) -> list:
        """휴대폰이 보낸 손 랜드마크(원시 좌표)를 `Hand` 리스트로 반환한다.

        raw_hands  : 손마다 21개의 (x, y, z) — 리스트/튜플([x,y,z]) 또는
                     {"x":.., "y":.., "z":..} 딕셔너리 모두 받는다. 빈 리스트면
                     빈 리스트를 그대로 돌려준다(손 없음).
        T_ee       : 현재 end-effector의 4x4 월드 변환 행렬 (Arm.fk_matrix(q))
        frame_shape: (height, width, channels) — 정규화 좌표를 픽셀로 되돌릴 때 사용
        """
        if not raw_hands:
            return []

        height, width = frame_shape[0], frame_shape[1]
        hands = []
        for raw_landmarks in raw_hands:
            landmarks = [_to_landmark(p) for p in raw_landmarks]
            depth = self.estimate_depth(landmarks, width, height)
            world_points = [self.landmark_to_world(lm, T_ee, depth) for lm in landmarks]
            center = tuple(sum(world_points[i][a] for i in self.PALM_QUAD) / len(self.PALM_QUAD)
                           for a in range(3))
            palm_landmarks = [landmarks[i] for i in self.PALM_QUAD]
            screen_offset = (sum(p.x for p in palm_landmarks) / len(palm_landmarks) - 0.5,
                             sum(p.y for p in palm_landmarks) / len(palm_landmarks) - 0.5)
            hands.append(Hand(landmarks, depth, world_points, center, screen_offset))
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

    @staticmethod
    def camera_frame(T_ee):
        """T_ee에서 카메라 기준 로컬 축 3개와 원점을 뽑아낸다.

        perception.camera_geometry.camera_frame으로 옮겨져 FaceTracker와
        공유된다 — 이 메서드는 기존 호출부(`self.camera_frame(...)`,
        `HandTracker.camera_frame(...)`)를 그대로 두기 위한 얇은 래퍼.
        """
        return _camera_frame(T_ee)

    def landmark_to_world(self, landmark, T_ee, depth) -> tuple:
        """랜드마크 하나를 월드 좌표 (x, y, z)로 변환한다.

        손 랜드마크가 놓이는 평면은 항상 origin + forward * forward_axis를
        지나고 forward_axis에 수직이므로, 그 평면의 방향벡터(법선)는 정확히
        end-effector의 방향벡터와 같다 — 팔이 회전하면 손 평면도 그대로 같이
        회전한다. 화면 중심(landmark (0.5, 0.5))의 점은 항상 이 forward_axis
        직선 위에 정확히 놓인다.

        landmark.z(손 안에서의 상대 깊이, 예: 구부린 손가락)는 이 랜드마크만의
        forward 거리를 조정하고, screen_right/screen_down도 공유된 depth가
        아니라 그 forward 거리 기준으로 다시 스케일해 핀홀 투영과 일치시킨다.
        """
        forward_axis, up_axis, side_axis, origin = self.camera_frame(T_ee)

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

        예전엔 mediapipe의 drawing_utils에 맡겼지만, 서버가 더 이상 mediapipe
        객체(NormalizedLandmarkList)를 만들지 않으므로(손 랜드마크가 휴대폰에서
        온 원시 좌표라서) HAND_CONNECTIONS 위상만으로 직접 선/점을 그린다.
        PALM_QUAD 네 점을 이어 손바닥 영역도 초록 사각형으로 덧그린다 — 손이
        카메라를 향하는지 눈으로 확인하기 쉽게 하기 위한 보조선.
        """
        import cv2

        h, w = frame_bgr.shape[0], frame_bgr.shape[1]
        for hand in hands:
            coords = [(int(lm.x * w), int(lm.y * h)) for lm in hand.landmarks]
            for i, j in HAND_CONNECTIONS:
                cv2.line(frame_bgr, coords[i], coords[j], (0, 200, 0), 2)
            for x, y in coords:
                cv2.circle(frame_bgr, (x, y), 3, (0, 0, 255), -1)

            palm_coords = [coords[i] for i in self.PALM_QUAD]
            for a, b in zip(palm_coords, palm_coords[1:] + palm_coords[:1]):
                cv2.line(frame_bgr, a, b, (0, 255, 0), 2)
        return frame_bgr

    def draw_forward_axis_debug(self, ax, T_ee, length):
        """end-effector가 카메라가 본다고 가정하는 방향(camera_frame의
        forward_axis, landmark_to_world와 동일한 정의)을 청록색 화살표로
        그린다 — 그 가정이 실제 카메라 방향과 맞는지 눈으로 확인하는 디버그용."""
        forward_axis, _, _, origin = self.camera_frame(T_ee)
        ax.quiver(origin[0], origin[1], origin[2],
                  forward_axis[0] * length, forward_axis[1] * length, forward_axis[2] * length,
                  color="cyan", linewidth=2.5, arrow_length_ratio=0.25, zorder=11)

    def draw_hands_3d(self, ax, hands):
        """3D 씬에 손 골격(초록)과 각 손의 손바닥 중심(빨간 점)을 그린다.

        손이 둘이면 HandFollower.combined_target()이 실제로 따라가는 지점인
        두 중심의 중점도 좀 더 큰 빨간 점으로 함께 표시한다.
        """
        for hand in hands:
            draw_points(ax, hand.world_points, self.connections)
            ax.scatter([hand.center[0]], [hand.center[1]], [hand.center[2]],
                      color="red", s=60, zorder=12)
        target = HandFollower.combined_target(hands)
        if target is not None and len(hands) > 1:
            ax.scatter([target[0]], [target[1]], [target[2]],
                      color="red", s=90, zorder=13)


class HandFollower:
    """손 위치를 따라 로봇 팔이 이동할 목표 지점을 계산하는 상태 객체.

    한 손이면 그 손의 중심(PALM_QUAD 평균), 두 손이면 두 손 중심의 중점을
    "목표 지점"(combined_target, 월드 좌표)과 "화면 오프셋"(combined_screen_offset,
    정규화 이미지 좌표에서 화면 정중앙까지의 거리)으로 함께 추적한다.

    "얼마나 움직였는지"가 아니라 "화면 중앙에서 얼마나 벗어났는지"를 트리거로
    쓴다 — 손이 화면 가장자리 쪽으로 CENTER_OFFSET_THRESHOLD 이상 벗어났을
    때만 로봇을 재정렬시키고, 중앙 근처(데드존)에 있는 동안은 가만히 둔다.

    트리거가 걸리면:
    - 좌우/상하(카메라 축에 수직인 평면)는 손이 화면 정중앙에 오도록 전부
      보정한다.
    - 앞뒤(카메라가 보는 방향 = 깊이)는 FOLLOW_DISTANCE_M과의 오차 중
      DEPTH_FOLLOW_GAIN 비율만큼만 보정한다 — 손이 카메라 쪽으로 다가오거나
      멀어져도 로봇이 끝까지 따라가지 않고 일정 부분만 따라간 뒤 멈추게 하기
      위함(과도하게 따라오는 느낌을 줄인다).

    실제 이동(IK 계산 + 서보 명령)은 하지 않는다 — `next_ee_target()`이 돌려준
    좌표를 호출부가 `hardware.actuator.ArmController.goto_position()`에
    넘겨야 로봇이 실제로 움직인다(perception 패키지는 하드웨어에 의존하지
    않는다는 프로젝트 관례를 따름).
    """

    # 상수 설명은 fundamental.const.HandFollowerConst 참고.
    FOLLOW_DISTANCE_M = HandFollowerConst.FOLLOW_DISTANCE_M
    CENTER_OFFSET_THRESHOLD = CameraGeometryConst.CENTER_OFFSET_THRESHOLD
    DEPTH_FOLLOW_GAIN = HandFollowerConst.DEPTH_FOLLOW_GAIN

    def __init__(self, follow_distance=FOLLOW_DISTANCE_M,
                 center_offset_threshold=CENTER_OFFSET_THRESHOLD,
                 depth_follow_gain=DEPTH_FOLLOW_GAIN):
        self.follow_distance = follow_distance
        self.center_offset_threshold = center_offset_threshold
        self.depth_follow_gain = depth_follow_gain

    @staticmethod
    def combined_target(hands):
        """손 목록에서 따라갈 목표 지점(월드 좌표) 하나를 뽑는다.

        손이 없으면 None, 하나면 그 손의 중심, 둘 이상이면 처음 두 손 중심의
        중점(현재 HandTracker는 max_num_hands=2 기준이라 실질적으로 전부)."""
        if not hands:
            return None
        if len(hands) == 1:
            return hands[0].center
        a, b = hands[0].center, hands[1].center
        return tuple((a[i] + b[i]) / 2 for i in range(3))

    @staticmethod
    def combined_screen_offset(hands):
        """손 목록에서 화면 정중앙까지의 오프셋(dx, dy, 정규화 좌표) 하나를
        뽑는다. combined_target과 동일한 규칙(하나면 그 손, 둘이면 평균)."""
        if not hands:
            return None
        if len(hands) == 1:
            return hands[0].screen_offset
        a, b = hands[0].screen_offset, hands[1].screen_offset
        return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

    def next_ee_target(self, hands, T_ee):
        """화면 중앙 오프셋이 threshold를 넘었을 때만 새 end-effector 목표
        (x, y, z)를 반환하고, 그렇지 않거나 손이 없으면 None을 반환한다
        (로봇을 그대로 둔다). T_ee: 현재 end-effector의 4x4 월드 변환 행렬."""
        target = self.combined_target(hands)
        offset = self.combined_screen_offset(hands)
        if target is None or offset is None:
            return None
        if math.hypot(offset[0], offset[1]) < self.center_offset_threshold:
            return None

        forward_axis, _, _, origin = HandTracker.camera_frame(T_ee)
        rel = tuple(target[i] - origin[i] for i in range(3))
        forward_dist = sum(rel[i] * forward_axis[i] for i in range(3))

        depth_error = forward_dist - self.follow_distance
        new_forward_dist = forward_dist - self.depth_follow_gain * depth_error

        # target에서 forward_axis 방향으로 new_forward_dist만큼 물러난 지점 —
        # 좌우/상하 성분은 target과 완전히 일치(화면 정중앙으로 재정렬)하고,
        # 깊이 성분만 new_forward_dist로 대체된다. x, y는 FaceFollower와 같은
        # 이유로 clamp_xy(perception.camera_geometry)로 |x|, |y| <=
        # IK_XY_LIMIT_M로 제한한다 — 팔이 베이스 회전축에서 너무 옆으로
        # 벗어나지 않게 하는 안전판.
        result = tuple(target[i] - new_forward_dist * forward_axis[i] for i in range(3))
        return _clamp_xy(result)
