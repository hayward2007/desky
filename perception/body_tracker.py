"""MediaPipe Pose로 사람의 몸(어깨)을 인식하고 로봇이 그 사람을 따라가게 하는
얼굴 인식 폴백 모듈.

얼굴 인식(perception.face_tracker)이 실패했을 때 — 옆모습, 고개를 돌린 경우,
얼굴이 화면 가장자리에 살짝 걸친 경우 등 — 를 위한 폴백이다. 얼굴이 안 보여도
어깨는 보이는 경우가 많다.

얼굴/손과 달리 인식(모델 추론) 자체를 **서버**에서 한다 — 사용자가 명시적으로
요청한 부분("사람 인식은 수신 받은 영상 정보로 백엔드에서 진행"). 다만 얼굴이
인식되는 프레임에서는 이 모듈이 아예 호출되지 않으므로(`src/perception_loop.py`의
`detect_bodies()` 참고), 이 프로젝트에서 서버가 하는 유일한 mediapipe 추론이
상시 비용이 되지는 않는다 — 폰이 실제로 보내는 ~20fps 중 얼굴이 안 보이는
프레임에서만 돈다.

perception.face_tracker/hand_tracker와 같은 구조: `BodyTracker`(인식 + 좌표변환
+ 시각화)와 `BodyFollower`(몸 위치를 따라 로봇이 이동할 목표 계산)로 나뉘고,
카메라 지오메트리(camera_frame)도 perception.camera_geometry를 공유한다.
추종 알고리즘(화면 오프셋 → yaw 회전 + 높이 이동, EMA 평활)은
perception.face_tracker.FaceFollower와 동일한 설계 — 값이 뭘 의미하는지는
fundamental.const.BodyFollowerConst 참고.

실제 하드웨어 이동 명령은 이 모듈이 하지 않는다 — perception 패키지는
하드웨어에 의존하지 않으므로, `BodyFollower`는 "어디로, 언제, 어떻게" 움직여야
할지만 계산하고 실제 `arm_ctrl.goto_joints()` 호출은 호출부(`src/perception_loop.py`)가
한다.

의존성: mediapipe, opencv-python, numpy, matplotlib(3D 시각화, kinematics.simulate)
"""

import math

from kinematics.urdf_loader import load_arm
from fundamental.const import BodyFollowerConst, BodyTrackerConst, CameraGeometryConst
from fundamental.logger import Logger
from perception.camera_geometry import Landmark, camera_frame, clamp_xy


class Body:
    """인식된 사람 몸 하나. `landmarks`는 mediapipe Pose의 33개 랜드마크(원본
    mediapipe 객체, 오버레이 그리기용), `depth`는 추정된 카메라~몸통 거리(m),
    `center`는 몸통 중심(양쪽 어깨 중점)의 월드 좌표, `screen_offset`은 그
    중심이 화면 정중앙 (0.5, 0.5)에서 얼마나 벗어났는지(dx, dy), 정규화 이미지
    좌표."""

    def __init__(self, landmarks, depth, center, screen_offset):
        self.landmarks = landmarks
        self.depth = depth
        self.center = center
        self.screen_offset = screen_offset


class BodyTracker:
    """카메라 프레임에서 사람 몸(어깨)을 찾아 3D 월드 좌표로 변환하는 객체.

    FaceTracker/HandTracker가 휴대폰으로 옮겨가기 전에 쓰던 것과 같은 패턴 —
    mediapipe가 설치돼 있지 않으면 `available`이 False가 되고 `process()`는
    항상 빈 리스트를 돌려준다(하드웨어/Gemini 미구성과 같은 부분 실패 패턴).
    """

    # 상수 설명은 fundamental.const.BodyTrackerConst 참고.
    FOCAL_NORM = CameraGeometryConst.FOCAL_NORM
    SHOULDER_WIDTH_M = BodyTrackerConst.SHOULDER_WIDTH_M
    LEFT_SHOULDER = BodyTrackerConst.LEFT_SHOULDER
    RIGHT_SHOULDER = BodyTrackerConst.RIGHT_SHOULDER
    MIN_LANDMARK_VISIBILITY = BodyTrackerConst.MIN_LANDMARK_VISIBILITY

    def __init__(self, min_detection_confidence=0.8, min_tracking_confidence=0.8):
        """mediapipe Pose 세션을 만든다. mediapipe import에 실패하면 조용히
        비활성 상태(`available == False`)로 남는다.

        model_complexity=0(Lite 모델)을 쓴다 — 이건 얼굴 인식이 실패했을 때만
        도는 폴백이라 최고 정확도보다 속도가 더 중요하다.
        """
        self.pose = None
        self.error = None
        self._mp = None
        self._drawing = None
        try:
            import mediapipe as mp

            self._mp = mp.solutions.pose
            self._drawing = mp.solutions.drawing_utils
            self.pose = self._mp.Pose(
                static_image_mode=False,
                model_complexity=0,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            Logger.log("BODY", "MediaPipe pose tracker ready")
        except Exception as e:
            self.error = str(e)
            Logger.log("BODY", f"Body tracking disabled: {self.error}")

    @property
    def available(self) -> bool:
        """mediapipe 세션이 정상적으로 만들어졌는지 여부."""
        return self.pose is not None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        """mediapipe Pose 세션을 닫는다(내부 그래프/스레드 정리)."""
        if self.pose is not None:
            self.pose.close()
            self.pose = None

    # ------------------------------------------------------------------
    # 인식
    # ------------------------------------------------------------------
    def process(self, frame_rgb, T_ee, frame_shape) -> list:
        """RGB 프레임 한 장에서 사람 몸을 찾아 `Body` 리스트로 반환한다.

        mediapipe Pose는 프레임 하나에 사람 한 명만 찾으므로 리스트 길이는
        항상 0 또는 1이다(FaceTracker의 max_num_faces=1과 같은 이유로 맞춘
        인터페이스 — FollowController가 "리스트의 첫 번째"만 보는 관례가
        그대로 통한다).

        frame_rgb  : cv2.COLOR_BGR2RGB로 변환한 프레임
        T_ee       : 현재 end-effector의 4x4 월드 변환 행렬 (Arm.fk_matrix(q))
        frame_shape: (height, width, channels)
        """
        if not self.available:
            return []

        results = self.pose.process(frame_rgb)
        if results.pose_landmarks is None:
            return []

        landmarks = results.pose_landmarks.landmark
        left, right = landmarks[self.LEFT_SHOULDER], landmarks[self.RIGHT_SHOULDER]
        if (left.visibility < self.MIN_LANDMARK_VISIBILITY
                or right.visibility < self.MIN_LANDMARK_VISIBILITY):
            return []  # 어깨가 잘 안 보이면(가려짐 등) 신뢰하지 않는다

        height, width = frame_shape[0], frame_shape[1]
        depth = self.estimate_depth(left, right, width, height)
        center_lm = Landmark(
            (left.x + right.x) / 2, (left.y + right.y) / 2, (left.z + right.z) / 2)
        center = self.landmark_to_world(center_lm, T_ee, depth)
        screen_offset = (center_lm.x - 0.5, center_lm.y - 0.5)
        return [Body(results.pose_landmarks, depth, center, screen_offset)]

    def estimate_depth(self, left, right, frame_width, frame_height) -> float:
        """FaceTracker.estimate_depth와 같은 핀홀 역산 — 기준자만 양쪽 어깨
        사이 거리(SHOULDER_WIDTH_M)로 바꿨다."""
        dx = (right.x - left.x) * frame_width
        dy = (right.y - left.y) * frame_height
        apparent = math.hypot(dx, dy) / frame_width
        apparent = max(apparent, 1e-4)
        return self.FOCAL_NORM * self.SHOULDER_WIDTH_M / apparent

    def landmark_to_world(self, landmark, T_ee, depth) -> tuple:
        """FaceTracker/HandTracker.landmark_to_world와 동일한 변환(같은
        camera_frame 공유)."""
        forward_axis, up_axis, side_axis, origin = camera_frame(T_ee)

        scale = depth / self.FOCAL_NORM
        forward = depth + landmark.z * scale
        scale_at_forward = forward / self.FOCAL_NORM
        screen_right = (landmark.x - 0.5) * scale_at_forward
        screen_down = (landmark.y - 0.5) * scale_at_forward

        return tuple(
            origin[i] + forward * forward_axis[i]
            + screen_down * up_axis[i] - screen_right * side_axis[i]
            for i in range(3)
        )

    # ------------------------------------------------------------------
    # 시각화
    # ------------------------------------------------------------------
    def draw_overlay(self, frame_bgr, bodies):
        """카메라 프레임 위에 몸 골격(POSE_CONNECTIONS)을 그린다(제자리 수정).

        얼굴/손과 달리 인식이 서버에서 직접 돌기 때문에 mediapipe 원본 객체가
        있다 — drawing_utils를 그대로 쓴다(FaceTracker/HandTracker가 휴대폰
        이관 전에 쓰던 것과 같은 방식).
        """
        if not self.available:
            return frame_bgr
        for body in bodies:
            self._drawing.draw_landmarks(
                frame_bgr, body.landmarks, self._mp.POSE_CONNECTIONS)
        return frame_bgr

    def draw_bodies_3d(self, ax, bodies):
        """3D 씬에 각 몸의 중심(어깨 중점)을 초록 점으로 그린다(얼굴의 파란
        점, 손의 빨간 점과 구분)."""
        for body in bodies:
            ax.scatter([body.center[0]], [body.center[1]], [body.center[2]],
                      color="green", s=70, zorder=12)


class BodyFollower:
    """몸(어깨) 위치를 따라 로봇이 이동할 다음 명령을 계산하는 상태 객체.

    perception.face_tracker.FaceFollower와 완전히 같은 설계다 — 화면 중앙
    정렬을 좌우/상하 축으로 분리해서 처리한다:
    - 좌우(screen_offset[0]): 1번 관절(yaw)을 오프셋에 비례해 돌려서 향하게
      한다.
    - 상하(screen_offset[1]): 위쪽에 있으면(offset 음수) 팔 높이(z)를 올리고,
      아래쪽에 있으면(offset 양수) 내린다.

    IK에 넘기는 목표 위치는 항상 x=y=0(perception.camera_geometry.clamp_xy로
    |x|, |y| <= IK_XY_LIMIT_M 유지)만 쓴다. 데드존 판정과 스텝 계산 모두
    `body.screen_offset`을 그대로 쓰지 않고 `_smooth()`로 지수이동평균(EMA)을
    건 값을 쓴다 — FaceFollower와 같은 이유(대상이 가만히 있어도 랜드마크가
    프레임마다 조금씩 흔들려서 팔이 "왔다갔다" 떨리는 현상 방지).

    실제 이동(IK 계산 + 서보 명령)은 하지 않는다 — `next_command()`가 돌려준
    (kind, payload)를 호출부가 `hardware.actuator.ArmController`의
    `goto_joints`에 넘겨야 로봇이 실제로 움직인다.
    """

    # 상수 설명은 fundamental.const.BodyFollowerConst 참고.
    CENTER_OFFSET_THRESHOLD = CameraGeometryConst.CENTER_OFFSET_THRESHOLD
    YAW_GAIN = BodyFollowerConst.YAW_GAIN
    YAW_STEP_LIMIT = BodyFollowerConst.YAW_STEP_LIMIT
    HEIGHT_GAIN = BodyFollowerConst.HEIGHT_GAIN
    HEIGHT_STEP_LIMIT = BodyFollowerConst.HEIGHT_STEP_LIMIT
    SCREEN_OFFSET_SMOOTHING = BodyFollowerConst.SCREEN_OFFSET_SMOOTHING

    def __init__(self, arm=None, center_offset_threshold=CENTER_OFFSET_THRESHOLD,
                 yaw_gain=YAW_GAIN, yaw_step_limit=YAW_STEP_LIMIT,
                 height_gain=HEIGHT_GAIN, height_step_limit=HEIGHT_STEP_LIMIT,
                 offset_smoothing=SCREEN_OFFSET_SMOOTHING):
        self.arm = arm if arm is not None else load_arm()
        self.center_offset_threshold = center_offset_threshold
        self.yaw_gain = yaw_gain
        self.yaw_step_limit = yaw_step_limit
        self.height_gain = height_gain
        self.height_step_limit = height_step_limit
        self.offset_smoothing = offset_smoothing
        self._yaw_index = self.arm.id_to_index[1]
        # 화면 오프셋의 지수이동평균 상태 — 몸을 놓쳤다 다시 찾으면(None)
        # 엉뚱한 예전 평균이 남아있지 않도록 리셋한다.
        self._smoothed_offset = None

    @staticmethod
    def primary_body(bodies):
        """mediapipe Pose는 프레임당 몸을 하나만 찾으므로 사실상 이거 하나뿐
        — FaceFollower.primary_face와 같은 관례를 맞춘 것."""
        return bodies[0] if bodies else None

    def _smooth(self, raw_offset):
        """screen_offset에 EMA를 걸어 프레임 간 랜드마크 잔떨림을 줄인다
        (FaceFollower._smooth와 동일한 로직)."""
        if self._smoothed_offset is None:
            self._smoothed_offset = raw_offset
        else:
            a = self.offset_smoothing
            self._smoothed_offset = (
                self._smoothed_offset[0] * (1 - a) + raw_offset[0] * a,
                self._smoothed_offset[1] * (1 - a) + raw_offset[1] * a,
            )
        return self._smoothed_offset

    def next_command(self, bodies, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload) 튜플로 반환하거나, 할 일이
        없으면(몸 없음/데드존 안) None을 반환한다.

        kind="joints": payload는 servo_deg 리스트 — goto_joints로 이동.
        current_q: 현재 관절각(rad) 리스트 — yaw/높이 보정의 기준값.
        """
        body = self.primary_body(bodies)
        if body is None:
            self._smoothed_offset = None
            return None
        offset = self._smooth(body.screen_offset)
        if math.hypot(*offset) < self.center_offset_threshold:
            return None

        return "joints", self._track_command(offset, T_ee, current_q)

    def _track_command(self, offset, T_ee, current_q):
        """좌우는 yaw 회전, 상하는 높이(z) 이동으로 화면 중앙에 맞춘다
        (FaceFollower._track_command와 동일한 로직)."""
        offset_x, offset_y = offset

        current_z = T_ee[2][3]
        z_step = self.height_gain * -offset_y
        z_step = max(-self.height_step_limit, min(self.height_step_limit, z_step))
        target = clamp_xy((0.0, 0.0, current_z + z_step))

        q_target, converged = self.arm.ik(target, seed=current_q)
        q = list(q_target) if converged else list(current_q)

        yaw_step = self.yaw_gain * offset_x
        yaw_step = max(-self.yaw_step_limit, min(self.yaw_step_limit, yaw_step))
        q[self._yaw_index] = current_q[self._yaw_index] + yaw_step

        return self.arm.q_to_servo_deg(q)
