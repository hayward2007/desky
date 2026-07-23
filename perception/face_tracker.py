"""휴대폰이 인식한 얼굴 랜드마크를 3D 월드 좌표로 올리고, 로봇이 그 얼굴을
따라가게 하는 모듈.

얼굴 인식(MediaPipe FaceLandmarker) 자체는 더 이상 서버가 하지 않는다 —
perception.hand_tracker와 같은 이유, 같은 방식이다: `src/templates/mobile.html`이
휴대폰에서 직접 MediaPipe Tasks Vision을 돌려 필요한 랜드마크 3개(코끝 +
양쪽 눈 바깥쪽 끝)만 웹소켓(`/ws/camera`)으로 보내고, `src.api.camera.Camera`가
그걸 받아 저장한다. 컴퓨터 하나가 얼굴+손 인식을 mediapipe로 전부 처리하면
휴대폰이 실제로 보내는 ~20fps를 못 따라갔던 것도 이유지만, 그것과 별개로
서버가 받는 프레임은 JPEG 압축 + 다운스케일을 거쳐 화질이 떨어져 인식 자체가
잘 안 되는 문제도 있었다 — 휴대폰이 압축 전 원본 영상에서 직접 인식하면 이
문제도 함께 없어진다.

perception.hand_tracker와 같은 구조: `FaceTracker`(좌표변환 + 시각화 —
랜드마크는 이미 계산되어 들어온다)와 `FaceFollower`(얼굴 위치를 따라 로봇이
이동할 목표 계산)로 나뉘고, 카메라 지오메트리(camera_frame, to_landmark)도
perception.camera_geometry를 그대로 공유한다.

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

의존성: opencv-python, numpy, matplotlib(3D 시각화, kinematics.simulate).
mediapipe는 더 이상 이 모듈의 의존성이 아니다(인식이 클라이언트로 옮겨갔으므로).
"""

import math

from kinematics.urdf_loader import load_arm
from fundamental.const import CameraGeometryConst, FaceTrackerConst, FaceFollowerConst
from perception.camera_geometry import camera_frame, clamp_xy, to_landmark

# 얼굴 중심으로 쓰는 이름 — 랜드마크 자체는 3개뿐이지만(코끝, 양쪽 눈 바깥쪽
# 끝), 어느 게 어느 건지 헷갈리지 않도록 이름으로 접근한다(휴대폰이 보내는
# JSON도 같은 이름의 키를 쓴다 — mobile.html 10번 섹션 참고).
_CENTER = "center"
_LEFT_EYE_OUTER = "left_eye_outer"
_RIGHT_EYE_OUTER = "right_eye_outer"


class Face:
    """인식된 얼굴 하나. `landmarks`는 {"center", "left_eye_outer",
    "right_eye_outer"} 세 개의 `camera_geometry.Landmark`, `depth`는 추정된
    카메라~얼굴 거리(m), `center`는 얼굴 중심(코끝)의 월드 좌표,
    `screen_offset`은 화면 정중앙 (0.5, 0.5) 대비 오프셋(dx, dy), 정규화
    이미지 좌표."""

    def __init__(self, landmarks, depth, center, screen_offset):
        self.landmarks = landmarks
        self.depth = depth
        self.center = center
        self.screen_offset = screen_offset


class FaceTracker:
    """휴대폰이 보낸 얼굴 랜드마크를 3D 월드 좌표로 변환하고 시각화하는 객체.

    HandTracker와 같은 전환 — 인식(모델 추론)은 하지 않는다. `process_landmarks()`가
    이미 계산된 랜드마크(휴대폰의 MediaPipe Tasks Vision FaceLandmarker 결과 중
    3개만)를 받아 좌표 변환/깊이 추정만 한다. 랜드마크가 없으면 항상 빈
    리스트를 돌려준다.
    """

    # 상수 설명은 fundamental.const.FaceTrackerConst 참고.
    FOCAL_NORM = CameraGeometryConst.FOCAL_NORM
    EYE_OUTER_DISTANCE_M = FaceTrackerConst.EYE_OUTER_DISTANCE_M

    # ------------------------------------------------------------------
    # 인식(휴대폰이 이미 계산한 랜드마크를 좌표 변환만)
    # ------------------------------------------------------------------
    def process_landmarks(self, raw_faces, T_ee, frame_shape) -> list:
        """휴대폰이 보낸 얼굴 랜드마크(원시 좌표)를 `Face` 리스트로 반환한다.

        raw_faces  : 얼굴마다 {"center": [x,y,z], "left_eye_outer": [x,y,z],
                     "right_eye_outer": [x,y,z]} — 각 값은 리스트/튜플 또는
                     {"x":.., "y":.., "z":..} 딕셔너리 모두 받는다(to_landmark
                     참고). 빈 리스트면 빈 리스트를 그대로 돌려준다(얼굴 없음).
        T_ee       : 현재 end-effector의 4x4 월드 변환 행렬 (Arm.fk_matrix(q))
        frame_shape: (height, width, channels) — 정규화 좌표를 픽셀로 되돌릴 때 사용
        """
        if not raw_faces:
            return []

        height, width = frame_shape[0], frame_shape[1]
        faces = []
        for raw in raw_faces:
            landmarks = {
                _CENTER: to_landmark(raw[_CENTER]),
                _LEFT_EYE_OUTER: to_landmark(raw[_LEFT_EYE_OUTER]),
                _RIGHT_EYE_OUTER: to_landmark(raw[_RIGHT_EYE_OUTER]),
            }
            depth = self.estimate_depth(landmarks, width, height)
            center_lm = landmarks[_CENTER]
            center = self.landmark_to_world(center_lm, T_ee, depth)
            screen_offset = (center_lm.x - 0.5, center_lm.y - 0.5)
            faces.append(Face(landmarks, depth, center, screen_offset))
        return faces

    def estimate_depth(self, landmarks, frame_width, frame_height) -> float:
        """HandTracker.estimate_depth와 같은 핀홀 역산 — 기준자만
        양쪽 눈 바깥쪽 끝 사이 거리(EYE_OUTER_DISTANCE_M)로 바꿨다."""
        left, right = landmarks[_LEFT_EYE_OUTER], landmarks[_RIGHT_EYE_OUTER]
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
        """카메라 프레임 위에 얼굴 중심 점 + 눈 사이 선을 그린다(제자리 수정).

        예전엔 mediapipe drawing_utils로 얼굴 윤곽 전체(468점)를 그렸지만,
        서버가 받는 랜드마크가 이제 3개뿐이라(코끝 + 눈 양쪽) 그 3점만으로
        표시한다 — 얼굴이 인식되고 있는지, 대략 어디를 향하는지 확인하는
        용도로는 충분하다.
        """
        import cv2

        h, w = frame_bgr.shape[0], frame_bgr.shape[1]
        for face in faces:
            c = face.landmarks[_CENTER]
            left = face.landmarks[_LEFT_EYE_OUTER]
            right = face.landmarks[_RIGHT_EYE_OUTER]
            cx, cy = int(c.x * w), int(c.y * h)
            lx, ly = int(left.x * w), int(left.y * h)
            rx, ry = int(right.x * w), int(right.y * h)
            cv2.line(frame_bgr, (lx, ly), (rx, ry), (255, 120, 0), 2)
            cv2.circle(frame_bgr, (cx, cy), 5, (255, 120, 0), -1)
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

    데드존 판정과 스텝 계산 모두 `face.screen_offset`을 그대로 쓰지 않고
    `_smooth()`로 지수이동평균(EMA)을 건 값을 쓴다 — 얼굴이 가만히 있어도
    랜드마크가 프레임마다 조금씩 흔들려서, 원본 값으로는 오프셋이 데드존
    경계를 넘나들며 팔이 계속 "왔다갔다" 떨렸다.
    """

    # 상수 설명은 fundamental.const.FaceFollowerConst 참고.
    CENTER_OFFSET_THRESHOLD = CameraGeometryConst.CENTER_OFFSET_THRESHOLD
    YAW_GAIN = FaceFollowerConst.YAW_GAIN
    YAW_STEP_LIMIT = FaceFollowerConst.YAW_STEP_LIMIT
    HEIGHT_GAIN = FaceFollowerConst.HEIGHT_GAIN
    HEIGHT_STEP_LIMIT = FaceFollowerConst.HEIGHT_STEP_LIMIT
    SCREEN_OFFSET_SMOOTHING = FaceFollowerConst.SCREEN_OFFSET_SMOOTHING

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
        # 화면 오프셋의 지수이동평균 상태 — 얼굴을 놓쳤다 다시 찾으면(None)
        # 엉뚱한 예전 평균이 남아있지 않도록 리셋한다.
        self._smoothed_offset = None

    @staticmethod
    def primary_face(faces):
        """여러 얼굴이 인식돼도 첫 번째(mediapipe가 가장 먼저 돌려준) 얼굴만
        따라간다 — max_num_faces=1이면 사실상 항상 이거 하나뿐."""
        return faces[0] if faces else None

    def _smooth(self, raw_offset):
        """screen_offset에 EMA를 걸어 프레임 간 랜드마크 잔떨림을 줄인다.

        가만히 있어도 FaceLandmarker의 랜드마크가 프레임마다 조금씩 흔들리는데,
        그 원본 값으로 데드존 판정/스텝 계산을 하면 오프셋이 경계를 넘나들며
        팔이 "왔다갔다" 떨리는 현상(limit cycle)이 생긴다 — 평활한 값을 쓰면
        그 잔떨림이 걸러진다.
        """
        if self._smoothed_offset is None:
            self._smoothed_offset = raw_offset
        else:
            a = self.offset_smoothing
            self._smoothed_offset = (
                self._smoothed_offset[0] * (1 - a) + raw_offset[0] * a,
                self._smoothed_offset[1] * (1 - a) + raw_offset[1] * a,
            )
        return self._smoothed_offset

    def next_command(self, faces, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload) 튜플로 반환하거나, 할 일이
        없으면(얼굴 없음/데드존 안) None을 반환한다.

        kind="joints": payload는 servo_deg 리스트 — goto_joints로 이동.
        current_q: 현재 관절각(rad) 리스트 — yaw/높이 보정의 기준값.
        """
        face = self.primary_face(faces)
        if face is None:
            self._smoothed_offset = None
            return None
        offset = self._smooth(face.screen_offset)
        if math.hypot(*offset) < self.center_offset_threshold:
            return None

        return "joints", self._track_command(offset, T_ee, current_q)

    def _track_command(self, offset, T_ee, current_q):
        """좌우는 yaw 회전, 상하는 높이(z) 이동으로 화면 중앙에 맞춘다."""
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
