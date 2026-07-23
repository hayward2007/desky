"""MediaPipe FaceMesh로 얼굴을 인식하고 로봇이 그 얼굴을 따라가게 하는 모듈.

perception.hand_tracker와 같은 구조: `FaceTracker`(인식 + 좌표변환 + 시각화)와
`FaceFollower`(얼굴 위치를 따라 로봇이 이동할 목표 계산)로 나뉘고, 카메라
지오메트리(camera_frame)도 perception.camera_geometry를 그대로 공유한다.

HandFollower와 다른 점: 화면 중앙 정렬을 좌우/상하 축을 분리해서 처리한다.
- 좌우(화면 가로 오프셋): 팔을 옆으로 이동시키는 대신 1번 관절(yaw, 베이스
  회전)을 오프셋에 비례해 돌려서 얼굴을 향하게 한다.
- 상하(화면 세로 오프셋): 얼굴이 화면 위쪽에 있으면 팔 높이(z)를 올리고,
  아래쪽에 있으면 내려서 중앙에 오게 한다.

IK 타겟은 항상 x=y=0 근방만 쓴다(perception.camera_geometry.clamp_xy로
|x|, |y| <= IK_XY_LIMIT_M 유지) — 좌우 정렬을 x/y 이동이 아니라 yaw 회전으로
흡수하기 때문에, 팔은 베이스 회전축 위에서 회전 + 높이 변화 위주로만
움직인다(IDLE_POSITION이 x=y=0인 것과 같은 이유).

실제 하드웨어 이동 명령은 이 모듈이 하지 않는다 — perception 패키지는
하드웨어에 의존하지 않으므로, `FaceFollower`는 "어디로, 언제, 어떻게(3D 위치
또는 관절각)" 움직여야 할지만 계산하고 실제 `arm_ctrl.goto_position()` /
`arm_ctrl.goto_joints()` 호출은 호출부(`src/app.py`)가 한다.

의존성: mediapipe, opencv-python, numpy, matplotlib(3D 시각화, kinematics.simulate)
"""

import math

from kinematics.simulate import draw_points
from kinematics.urdf_loader import load_arm
from fundamental.const import FaceTrackerConst, FaceFollowerConst
from fundamental.logger import Logger
from perception.camera_geometry import camera_frame, clamp_xy


class Face:
    """인식된 얼굴 하나. `landmarks`는 mediapipe FaceMesh의 랜드마크(468개),
    `depth`는 추정된 카메라~얼굴 거리(m), `center`는 얼굴 중심(코끝 랜드마크)의
    월드 좌표, `screen_offset`은 화면 정중앙 (0.5, 0.5) 대비 오프셋(dx, dy),
    정규화 이미지 좌표."""

    def __init__(self, landmarks, depth, center, screen_offset):
        self.landmarks = landmarks
        self.depth = depth
        self.center = center
        self.screen_offset = screen_offset


class FaceTracker:
    """카메라 프레임에서 얼굴을 찾아 3D 월드 좌표로 변환하는 객체.

    HandTracker와 같은 부분 실패 패턴 — mediapipe가 설치돼 있지 않으면
    `available`이 False가 되고 `process()`는 항상 빈 리스트를 돌려준다.
    """

    # 상수 설명은 fundamental.const.FaceTrackerConst 참고.
    FOCAL_NORM = FaceTrackerConst.FOCAL_NORM
    EYE_OUTER_DISTANCE_M = FaceTrackerConst.EYE_OUTER_DISTANCE_M
    LEFT_EYE_OUTER = FaceTrackerConst.LEFT_EYE_OUTER
    RIGHT_EYE_OUTER = FaceTrackerConst.RIGHT_EYE_OUTER
    CENTER_LANDMARK = FaceTrackerConst.CENTER_LANDMARK

    def __init__(self, max_num_faces=1, min_detection_confidence=0.6,
                 min_tracking_confidence=0.6):
        """mediapipe FaceMesh 세션을 만든다. mediapipe import에 실패하면
        조용히 비활성 상태(`available == False`)로 남는다."""
        self.face_mesh = None
        self.error = None
        self._mp = None
        self._drawing = None
        try:
            import mediapipe as mp

            self._mp = mp.solutions.face_mesh
            self._drawing = mp.solutions.drawing_utils
            self.face_mesh = self._mp.FaceMesh(
                static_image_mode=False,
                max_num_faces=max_num_faces,
                refine_landmarks=False,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            Logger.log("FACE", "MediaPipe face tracker ready")
        except Exception as e:
            self.error = str(e)
            Logger.log("FACE", f"Face tracking disabled: {self.error}")

    @property
    def available(self) -> bool:
        """mediapipe 세션이 정상적으로 만들어졌는지 여부."""
        return self.face_mesh is not None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def close(self):
        """mediapipe FaceMesh 세션을 닫는다(내부 그래프/스레드 정리)."""
        if self.face_mesh is not None:
            self.face_mesh.close()
            self.face_mesh = None

    # ------------------------------------------------------------------
    # 인식
    # ------------------------------------------------------------------
    def process(self, frame_rgb, T_ee, frame_shape) -> list:
        """RGB 프레임 한 장에서 얼굴을 찾아 `Face` 리스트로 반환한다.

        frame_rgb  : cv2.COLOR_BGR2RGB로 변환한 프레임
        T_ee       : 현재 end-effector의 4x4 월드 변환 행렬 (Arm.fk_matrix(q))
        frame_shape: (height, width, channels)
        """
        if not self.available:
            return []

        results = self.face_mesh.process(frame_rgb)
        if not results.multi_face_landmarks:
            return []

        height, width = frame_shape[0], frame_shape[1]
        faces = []
        for face_landmarks in results.multi_face_landmarks:
            depth = self.estimate_depth(face_landmarks.landmark, width, height)
            center_lm = face_landmarks.landmark[self.CENTER_LANDMARK]
            center = self.landmark_to_world(center_lm, T_ee, depth)
            screen_offset = (center_lm.x - 0.5, center_lm.y - 0.5)
            faces.append(Face(face_landmarks, depth, center, screen_offset))
        return faces

    def estimate_depth(self, landmarks, frame_width, frame_height) -> float:
        """HandTracker.estimate_depth와 같은 핀홀 역산 — 기준자만
        양쪽 눈 바깥쪽 끝 사이 거리(EYE_OUTER_DISTANCE_M)로 바꿨다."""
        left, right = landmarks[self.LEFT_EYE_OUTER], landmarks[self.RIGHT_EYE_OUTER]
        dx = (right.x - left.x) * frame_width
        dy = (right.y - left.y) * frame_height
        apparent = math.hypot(dx, dy) / frame_width
        apparent = max(apparent, 1e-4)
        return self.FOCAL_NORM * self.EYE_OUTER_DISTANCE_M / apparent

    def landmark_to_world(self, landmark, T_ee, depth) -> tuple:
        """HandTracker.landmark_to_world와 동일한 변환(같은 camera_frame 공유)."""
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
    def draw_overlay(self, frame_bgr, faces):
        """카메라 프레임 위에 얼굴 윤곽(FACEMESH_CONTOURS)을 그린다(제자리 수정)."""
        if not self.available:
            return frame_bgr
        for face in faces:
            self._drawing.draw_landmarks(
                frame_bgr, face.landmarks, self._mp.FACEMESH_CONTOURS)
        return frame_bgr

    def draw_faces_3d(self, ax, faces):
        """3D 씬에 각 얼굴의 중심을 파란 점으로 그린다(손의 빨간 점과 구분)."""
        for face in faces:
            ax.scatter([face.center[0]], [face.center[1]], [face.center[2]],
                      color="blue", s=70, zorder=12)


class FaceFollower:
    """얼굴 위치를 따라 로봇이 이동할 다음 명령을 계산하는 상태 객체.

    화면 중앙 정렬을 좌우/상하 축으로 분리해서 처리한다:
    - 좌우(screen_offset[0]): 1번 관절(yaw)을 오프셋에 비례해 현재 각도에서
      더 돌려 얼굴을 향하게 한다. 회전 방향(yaw_gain의 부호)은 실측으로 확인된
      값이 아니므로, 실제로 반대로 도는 것 같으면 yaw_gain 부호만 뒤집으면
      된다.
    - 상하(screen_offset[1]): 얼굴이 화면 위쪽에 있으면(offset 음수) 팔
      높이(z)를 올리고, 아래쪽에 있으면(offset 양수) 내린다.

    IK에 넘기는 목표 위치는 항상 x=y=0(perception.camera_geometry.clamp_xy로
    |x|, |y| <= IK_XY_LIMIT_M 유지)만 쓴다 — 좌우 정렬을 x/y 이동이 아니라
    yaw 회전으로 흡수하므로, 팔은 베이스 회전축 위에서 회전 + 높이 변화 위주로만
    움직인다.

    실제 이동(IK 계산 + 서보 명령)은 하지 않는다 — `next_command()`가 돌려준
    (kind, payload)를 호출부가 `hardware.actuator.ArmController`의
    `goto_joints`에 넘겨야 로봇이 실제로 움직인다.
    """

    # 상수 설명은 fundamental.const.FaceFollowerConst 참고.
    CENTER_OFFSET_THRESHOLD = FaceFollowerConst.CENTER_OFFSET_THRESHOLD
    YAW_GAIN = FaceFollowerConst.YAW_GAIN
    YAW_STEP_LIMIT = FaceFollowerConst.YAW_STEP_LIMIT
    HEIGHT_GAIN = FaceFollowerConst.HEIGHT_GAIN
    HEIGHT_STEP_LIMIT = FaceFollowerConst.HEIGHT_STEP_LIMIT

    def __init__(self, arm=None, center_offset_threshold=CENTER_OFFSET_THRESHOLD,
                 yaw_gain=YAW_GAIN, yaw_step_limit=YAW_STEP_LIMIT,
                 height_gain=HEIGHT_GAIN, height_step_limit=HEIGHT_STEP_LIMIT):
        self.arm = arm if arm is not None else load_arm()
        self.center_offset_threshold = center_offset_threshold
        self.yaw_gain = yaw_gain
        self.yaw_step_limit = yaw_step_limit
        self.height_gain = height_gain
        self.height_step_limit = height_step_limit
        self._yaw_index = self.arm.id_to_index[1]

    @staticmethod
    def primary_face(faces):
        """여러 얼굴이 인식돼도 첫 번째(mediapipe가 가장 먼저 돌려준) 얼굴만
        따라간다 — max_num_faces=1이면 사실상 항상 이거 하나뿐."""
        return faces[0] if faces else None

    def next_command(self, faces, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload) 튜플로 반환하거나, 할 일이
        없으면(얼굴 없음/데드존 안) None을 반환한다.

        kind="joints": payload는 servo_deg 리스트 — goto_joints로 이동.
        current_q: 현재 관절각(rad) 리스트 — yaw/높이 보정의 기준값.
        """
        face = self.primary_face(faces)
        if face is None:
            return None
        if math.hypot(*face.screen_offset) < self.center_offset_threshold:
            return None

        return "joints", self._track_command(face, T_ee, current_q)

    def _track_command(self, face, T_ee, current_q):
        """좌우는 yaw 회전, 상하는 높이(z) 이동으로 화면 중앙에 맞춘다."""
        offset_x, offset_y = face.screen_offset

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
