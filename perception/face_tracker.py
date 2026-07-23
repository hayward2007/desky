"""MediaPipe FaceMesh로 얼굴을 인식하고 로봇이 그 얼굴을 따라가게 하는 모듈.

perception.hand_tracker와 같은 구조: `FaceTracker`(인식 + 좌표변환 + 시각화)와
`FaceFollower`(얼굴 위치를 따라 로봇이 이동할 목표 계산)로 나뉘고, 카메라
지오메트리(camera_frame)도 perception.camera_geometry를 그대로 공유한다.

HandFollower와 다른 점: 거리에 따라 두 가지 추종 방식을 쓴다.
- 가까움(NEAR_DISTANCE_M 미만): HandFollower와 같은 방식 — 화면 중앙 오프셋이
  threshold를 넘으면 좌우/상하는 완전히 재정렬하고 깊이는 일부만 보정.
- 멀리(NEAR_DISTANCE_M 이상, 책상에 앉아 있을 때의 일반적인 거리): 팔 전체를
  움직이는 대신 HOME_POSITION으로 IK를 풀어 둔 자세를 유지한 채 1번 관절
  (yaw, 베이스 회전)만 화면 좌우 오프셋에 비례해 돌려서 얼굴을 향하게 한다.

실제 하드웨어 이동 명령은 이 모듈이 하지 않는다 — perception 패키지는
하드웨어에 의존하지 않으므로, `FaceFollower`는 "어디로, 언제, 어떻게(3D 위치
또는 관절각)" 움직여야 할지만 계산하고 실제 `arm_ctrl.goto_position()` /
`arm_ctrl.goto_joints()` 호출은 호출부(`src/app.py`)가 한다.

의존성: mediapipe, opencv-python, numpy, matplotlib(3D 시각화, kinematics.simulate)
"""

import math

from kinematics.simulate import draw_points
from kinematics.urdf_loader import load_arm
from logger import Logger
from perception.camera_geometry import camera_frame


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

    FOCAL_NORM = 1.0

    # 양쪽 눈 바깥쪽 끝(landmark 33=오른쪽, 263=왼쪽) 사이 실제 거리 추정치.
    # 실측 캘리브레이션 값이 아니므로 거리 추정은 참고치 — HandTracker의
    # WRIST_TO_THUMB_CMC_M와 같은 방식(핀홀 역산의 기준자).
    EYE_OUTER_DISTANCE_M = 0.09
    LEFT_EYE_OUTER = 33
    RIGHT_EYE_OUTER = 263
    # 얼굴 중심으로 쓰는 랜드마크 — FaceMesh 표준 index, 코끝 근처.
    CENTER_LANDMARK = 1

    def __init__(self, max_num_faces=1, min_detection_confidence=0.5,
                 min_tracking_confidence=0.5):
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

    거리에 따라 두 가지 추종 방식을 쓴다:
    - NEAR_DISTANCE_M 미만(가까움): HandFollower와 같은 방식 — 화면 중앙
      오프셋이 CENTER_OFFSET_THRESHOLD를 넘으면 좌우/상하는 완전히
      재정렬하고, 깊이는 FOLLOW_DISTANCE_M과의 오차 중 DEPTH_FOLLOW_GAIN
      비율만큼만 보정한 3D 목표 위치를 낸다 → 호출부가 goto_position(IK)로
      이동시킨다.
    - NEAR_DISTANCE_M 이상(보통 사용 거리): 팔 전체를 움직이지 않는다.
      HOME_POSITION에 대해 IK를 풀어 둔 자세(관절 2~5)는 그대로 두고, 1번
      관절(yaw)만 화면 좌우 오프셋(screen_offset[0])에 비례해 현재 각도에서
      더 돌려 얼굴을 향하게 한다 → 호출부가 goto_joints(직접 관절 제어)로
      이동시킨다. 1번 관절의 회전 방향(yaw_gain의 부호)은 실측으로 확인된
      값이 아니므로, 실제로 반대로 도는 것 같으면 yaw_gain 부호만 뒤집으면
      된다.

    실제 이동(IK 계산 + 서보 명령)은 하지 않는다 — `next_command()`가 돌려준
    (kind, payload)를 호출부가 `hardware.actuator.ArmController`의
    `goto_position`/`goto_joints`에 넘겨야 로봇이 실제로 움직인다.
    """

    FOLLOW_DISTANCE_M = 0.3
    NEAR_DISTANCE_M = 0.4
    CENTER_OFFSET_THRESHOLD = 0.15
    DEPTH_FOLLOW_GAIN = 0.3
    HOME_POSITION = (0.0, 0.0, 0.3)
    # 화면 좌우 오프셋(대략 -0.5~0.5)을 1번 관절 각도 보정량(rad)으로 바꾸는
    # 비례 이득 — 실측으로 튜닝 필요.
    YAW_GAIN = 0.8
    # 한 번의 갱신에서 1번 관절이 움직일 수 있는 최대 각도(rad) — 큰 오프셋이
    # 갑자기 튀어도 로봇이 한 번에 확 돌지 않도록 하는 안전판.
    YAW_STEP_LIMIT = math.radians(20)

    def __init__(self, arm=None, follow_distance=FOLLOW_DISTANCE_M,
                 near_distance=NEAR_DISTANCE_M,
                 center_offset_threshold=CENTER_OFFSET_THRESHOLD,
                 depth_follow_gain=DEPTH_FOLLOW_GAIN,
                 home_position=HOME_POSITION, yaw_gain=YAW_GAIN,
                 yaw_step_limit=YAW_STEP_LIMIT):
        self.arm = arm if arm is not None else load_arm()
        self.follow_distance = follow_distance
        self.near_distance = near_distance
        self.center_offset_threshold = center_offset_threshold
        self.depth_follow_gain = depth_follow_gain
        self.home_position = home_position
        self.yaw_gain = yaw_gain
        self.yaw_step_limit = yaw_step_limit
        self._yaw_index = self.arm.id_to_index[1]

    @staticmethod
    def primary_face(faces):
        """여러 얼굴이 인식돼도 첫 번째(mediapipe가 가장 먼저 돌려준) 얼굴만
        따라간다 — max_num_faces=1이면 사실상 항상 이거 하나뿐."""
        return faces[0] if faces else None

    def next_command(self, faces, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload) 튜플로 반환하거나, 할 일이
        없으면(얼굴 없음/데드존 안) None을 반환한다.

        kind="position": payload는 (x, y, z) — goto_position(IK)로 이동.
        kind="joints"  : payload는 servo_deg 리스트 — goto_joints로 이동.
        current_q: 현재 관절각(rad) 리스트 — 원거리일 때 1번 관절의 기준값.
        """
        face = self.primary_face(faces)
        if face is None:
            return None
        if math.hypot(*face.screen_offset) < self.center_offset_threshold:
            return None

        if face.depth < self.near_distance:
            return "position", self._near_target(face, T_ee)
        return "joints", self._far_yaw_command(face, current_q)

    def _near_target(self, face, T_ee):
        """가까울 때: 좌우/상하는 화면 정중앙으로 완전히, 깊이는 일부만 보정."""
        forward_axis, _, _, origin = camera_frame(T_ee)
        rel = tuple(face.center[i] - origin[i] for i in range(3))
        forward_dist = sum(rel[i] * forward_axis[i] for i in range(3))

        depth_error = forward_dist - self.follow_distance
        new_forward_dist = forward_dist - self.depth_follow_gain * depth_error

        return tuple(face.center[i] - new_forward_dist * forward_axis[i] for i in range(3))

    def _far_yaw_command(self, face, current_q):
        """멀 때: HOME_POSITION의 IK 자세를 유지한 채 1번 관절만 조정."""
        q_home, converged = self.arm.ik(self.home_position)
        q = list(q_home) if converged else (list(current_q) if current_q is not None
                                             else [0.0] * len(self.arm.joints))

        current_yaw = current_q[self._yaw_index] if current_q is not None else q[self._yaw_index]
        step = self.yaw_gain * face.screen_offset[0]
        step = max(-self.yaw_step_limit, min(self.yaw_step_limit, step))
        q[self._yaw_index] = current_yaw + step

        return self.arm.q_to_servo_deg(q)
