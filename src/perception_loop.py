"""휴대폰 카메라 프레임 한 장을 받아 로봇의 다음 동작까지 끌고 가는 루프.

[병합 핵심] 두 브랜치가 가장 정면으로 부딪힌 곳이 바로 이 루프다.

- develop 브랜치: 얼굴 인식(서버 mediapipe FaceMesh) + 손 랜드마크(휴대폰이
  보내 준 것) → `FollowController`가 얼굴 > 손 > idle 우선순위로 팔을 움직임.
  명령/렌더링을 각각 별도 간격으로 throttle.
- mobile 브랜치: 손 인식을 **서버 mediapipe Hands**로 직접 돌리고 그 결과로
  가위바위보를 판정 → 폰에 명령 전송. throttle 없음.

두 개를 그대로 합치면 손 인식이 두 번(서버 + 휴대폰) 돌아 서로 다른 손 목록이
생기고, 서버 쪽 mediapipe 비용 때문에 프레임레이트도 다시 무너진다. 그래서
**손 인식 경로를 하나로 통일**했다:

    휴대폰(MediaPipe Tasks Vision) → /ws/camera → Camera.hand_landmarks
        → HandTracker.process_landmarks()  →  ① FollowController (팔 추종)
                                              ② GestureBridge   (가위바위보)

즉 손 목록 하나를 두 소비자가 나눠 쓴다. 가위바위보 판정에 필요한 건 랜드마크
21개의 x/y뿐이고 그건 휴대폰이 보내 준 값으로 충분하므로, 서버에서 mediapipe
Hands를 다시 돌릴 이유가 없다.

[추가 이관] 얼굴 인식도 같은 이유로 같은 방식으로 휴대폰으로 옮겼다 — 서버가
얼굴+손을 둘 다 mediapipe로 처리하면 폰이 실제로 보내는 ~20fps를 못 따라갔고,
그것과 별개로 서버가 받는 프레임은 JPEG 압축 + 다운스케일 때문에 화질이
떨어져 인식 자체가 잘 안 되는 문제도 있었다(흐릿한 프레임 → FaceMesh가 얼굴을
못 잡음). 휴대폰이 압축 전 원본 영상에서 직접 인식하면 프레임레이트와 화질
문제가 한 번에 해결된다:

    휴대폰(MediaPipe Tasks Vision) → /ws/camera → Camera.face_landmarks
        → FaceTracker.process_landmarks() → FollowController (팔 추종)

이제 서버는 얼굴/손 어느 쪽도 모델 추론을 하지 않는다 — 둘 다 좌표 변환만
한다. mediapipe(Python)는 대부분 이 루프의 의존성이 아니게 됐다.

[몸 폴백 추가] 얼굴 인식이 실패하는 경우(옆모습, 고개를 돌린 경우 등)를 위해
사람 몸(어깨) 인식을 폴백으로 추가했다 — 이건 사용자가 명시적으로 서버에서
돌아야 한다고 요청한 유일한 예외다:

    카메라 프레임(수신된 JPEG) → perception.body_tracker.BodyTracker.process()
        (서버 mediapipe Pose, 얼굴이 안 보일 때만) → FollowController (팔 추종)

`FollowController`의 우선순위는 얼굴 > 몸 > 손 > idle이다. 얼굴이 보이는
프레임에서는 몸 인식 자체를 건너뛰므로(아래 `detect_bodies()` 참고), 서버가
하는 이 유일한 mediapipe 추론도 상시 비용은 아니다.

세 가지 일이 **서로 다른 주기**로 돈다 — 이것도 병합에서 정리한 부분이다:
  1. 인식/판단  : 새 프레임이 올 때마다(최대 ~20fps). 상태 머신이 최신이어야 함.
  2. 하드웨어 명령: AppConst.COMMAND_MIN_INTERVAL_S 이상 간격(너무 자주 새 목표를
                   주면 팔이 계속 움직임을 새로 시작해 덜덜거린다).
  3. 미리보기 창 : AppConst.VIS_MIN_INTERVAL_S 이상 간격(matplotlib 3D 렌더링이
                   인식보다 비싸서, 매 프레임 그리면 이 렌더링이 병목이 된다).
"""

import time

import cv2
import numpy as np

from fundamental.const import AppConst, FollowControllerConst
from fundamental.logger import Logger
from perception.body_tracker import BodyTracker
from perception.face_tracker import FaceTracker
from perception.follow_controller import FollowController
from perception.hand_tracker import HandTracker
from src.arm_service import HardwareUnavailable
from src.render import ScenePreview


class PerceptionLoop:
    """카메라 → 인식 → 추종/제스처 → 미리보기까지를 담당하는 루프 객체."""

    # 상수 설명은 fundamental.const.AppConst 참고.
    COMMAND_MIN_INTERVAL_S = AppConst.COMMAND_MIN_INTERVAL_S
    VIS_MIN_INTERVAL_S = AppConst.VIS_MIN_INTERVAL_S
    BODY_MEDIAPIPE_MAX_WIDTH = AppConst.BODY_MEDIAPIPE_MAX_WIDTH
    # FollowController와 같은 값을 공유한다 — 얼굴이 이 거리보다 가까우면
    # FollowController가 몸으로 갈아탈 일이 없으니 몸 인식 자체를 건너뛴다
    # (fundamental.const.FollowControllerConst 참고).
    FACE_BODY_SWITCH_DISTANCE_M = FollowControllerConst.FACE_BODY_SWITCH_DISTANCE_M
    # FollowController._pick_target의 히스테리시스 폭 — 몸 인식은 이 폭만큼
    # 더 일찍(가까운 거리에서부터) 켜 둔다. 그래야 얼굴이 히스테리시스
    # 경계(FACE_BODY_SWITCH_DISTANCE_M + 이 값)를 실제로 넘어 몸으로 갈아탈
    # 시점에 몸 데이터가 이미 준비돼 있다 — 정확히 경계에서만 켜면 그 프레임엔
    # 아직 몸이 안 잡혀 전환이 한 박자 늦거나, 몸 인식 자체가 들쭉날쭉해져
    # 히스테리시스를 둔 의미가 없어진다.
    FACE_BODY_SWITCH_HYSTERESIS_M = FollowControllerConst.FACE_BODY_SWITCH_HYSTERESIS_M

    def __init__(self, arm_service, camera, renderer,
                 gesture_bridge=None, show_preview=True):
        """트래커·추종 컨트롤러·미리보기 창을 준비한다(아직 돌리지는 않는다).

        arm_service   : 팔 상태 + 실제 이동 (src.arm_service.ArmService)
        camera        : 휴대폰이 보낸 프레임/랜드마크 보관소 (src.api.camera.Camera)
        renderer      : 3D 그림 (src.render.ArmRenderer)
        gesture_bridge: 가위바위보 배선. None이면 제스처 기능만 빠진다.
        show_preview  : 로컬 cv2 창을 띄울지. 서버로만 돌릴 땐 False.
        """
        self.arm_service = arm_service
        self.camera = camera
        self.renderer = renderer
        self.gesture_bridge = gesture_bridge
        self.show_preview = show_preview

        # 손/얼굴 인식은 둘 다 휴대폰이 한다 — 이 객체들은 좌표 변환/시각화만
        # 담당한다(모델 추론 없음, mediapipe 의존성 없음).
        self.hand_tracker = HandTracker()
        self.face_tracker = FaceTracker()
        # 몸(어깨) 인식은 얼굴 인식 실패 시 폴백으로 서버가 직접 mediapipe
        # Pose를 돌린다 — mediapipe가 없으면 available이 False가 되어
        # process()가 빈 리스트를 돌려준다(앱은 계속 뜸).
        self.body_tracker = BodyTracker()
        # 얼굴 > 몸 > 손 > idle 우선순위와 두리번거리기 상태 머신.
        self.follow_controller = FollowController(arm_service.arm)

        self.scene = ScenePreview(renderer) if show_preview else None

        self._last_frame_count = 0
        self._last_command_time = 0.0
        self._last_vis_time = 0.0

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------
    def run_forever(self):
        """새 프레임이 올 때까지 돌면서 프레임마다 `process_frame()`을 부른다.

        'q' 또는 Esc(미리보기 창에서)나 Ctrl+C로 끝난다. cv2의 창 관련 호출은
        반드시 메인 스레드에 있어야 한다(macOS에서 다른 스레드의 imshow/waitKey는
        조용히 아무 일도 하지 않는다) — 그래서 Flask 서버 쪽이 백그라운드
        스레드로 가고 이 루프가 메인 스레드에 남는다.
        """
        Logger.log("CAMERA", "Press 'q' in the camera window (or Ctrl+C here) to quit")
        try:
            while True:
                frame_bytes, frame_count = self.camera.snapshot()
                # 진짜 새 프레임일 때만 디코드한다 — 폰은 초당 몇 장만 보내는데
                # 이 루프는 훨씬 빨리 도므로(waitKey(1)은 상한이지 보장이 아니다),
                # 확인하지 않으면 같은 바이트를 계속 다시 디코드하게 된다.
                if frame_bytes is not None and frame_count != self._last_frame_count:
                    self._last_frame_count = frame_count
                    self.process_frame(frame_bytes)

                # waitKey는 HighGUI 이벤트 루프를 돌리는 역할도 한다 — 이게
                # 없으면 창이 아예 갱신되지 않는다.
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def process_frame(self, frame_bytes):
        """프레임 한 장: 디코드 → 인식 → 팔 명령 → 미리보기."""
        # /mobile이 방향(회전/미러링)을 이미 맞춰서 보내므로 여기서 돌릴 필요가 없다.
        frame = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        q = self.arm_service.current_q_or_home()
        T_ee = self.arm_service.ee_matrix(q)

        faces = self.collect_faces(T_ee, frame.shape)
        # 몸(어깨) 인식은 얼굴이 안 보이거나, 보여도 FollowController가 몸으로
        # 갈아탈 만큼 멀 때만 돈다(FACE_BODY_SWITCH_DISTANCE_M) — 서버가 하는
        # 유일한 mediapipe 추론이라, 필요할 때만 돌려 비용을 아낀다. 얼굴이
        # 가까이 있으면 몸 인식 자체를 건너뛴다. 경계에서 히스테리시스 폭만큼
        # 더 일찍(가까운 쪽에서부터) 켜는 이유는 위 FACE_BODY_SWITCH_HYSTERESIS_M
        # 참고 — FollowController가 실제로 몸으로 갈아타는 시점보다 몸 인식이
        # 늦게 켜지면 그 히스테리시스를 둔 의미가 없어진다.
        primary_face = faces[0] if faces else None
        need_body = (
            primary_face is None
            or primary_face.depth >= self.FACE_BODY_SWITCH_DISTANCE_M - self.FACE_BODY_SWITCH_HYSTERESIS_M
        )
        bodies = self.detect_bodies(frame, T_ee) if need_body else []
        hands = self.collect_hands(T_ee, frame.shape)

        now = time.monotonic()
        self.drive_arm(faces, hands, bodies, T_ee, q, now)
        gesture = self.update_gestures(hands, frame.shape)
        self.update_preview(frame, faces, hands, bodies, gesture, q, T_ee, now)

    # ------------------------------------------------------------------
    # 인식
    # ------------------------------------------------------------------
    def collect_faces(self, T_ee, frame_shape):
        """휴대폰이 보낸 얼굴 랜드마크를 월드 좌표 `Face` 목록으로 바꾼다.

        collect_hands()와 같은 이유로 같은 모양이다 — 인식(모델 추론)은 폰이
        끝냈으므로 여기서는 좌표 계산만 한다. 폰이 아직 아무것도 안 보냈으면
        빈 목록.
        """
        raw_faces = self.camera.latest_face_landmarks()
        if not raw_faces:
            return []
        return self.face_tracker.process_landmarks(raw_faces, T_ee, frame_shape)

    def detect_bodies(self, frame, T_ee):
        """프레임에서 사람 몸(어깨)을 찾는다(서버 mediapipe Pose) — 얼굴
        인식이 실패했을 때만 호출부(process_frame)가 부르는 폴백이다.

        mediapipe 비용은 픽셀 수에 비례하므로, 폰이 큰 해상도로 보내면
        BODY_MEDIAPIPE_MAX_WIDTH로 줄인 **복사본**에서만 인식을 돌린다.
        랜드마크는 정규화 좌표(0~1)라 비율만 유지하면 정확도에 영향이 없고,
        깊이 추정에는 원본 크기(`frame.shape`)를 그대로 넘긴다 — 핀홀 계산이
        실제 카메라 화각 기준이라 축소본 크기를 쓰면 거리가 어긋난다.
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        if width > self.BODY_MEDIAPIPE_MAX_WIDTH:
            scale = self.BODY_MEDIAPIPE_MAX_WIDTH / width
            rgb = cv2.resize(rgb, (int(width * scale), int(height * scale)),
                             interpolation=cv2.INTER_AREA)
        return self.body_tracker.process(rgb, T_ee, frame.shape)

    def collect_hands(self, T_ee, frame_shape):
        """휴대폰이 보낸 손 랜드마크를 월드 좌표 `Hand` 목록으로 바꾼다.

        모델 추론이 아니라 좌표 계산뿐이므로(인식은 폰이 끝냈다) 얼굴이 이미
        잡힌 프레임에서도 건너뛰지 않는다 — 팔은 얼굴을 따라가더라도 제스처는
        손으로 받아야 하기 때문. 폰이 아직 아무것도 안 보냈으면 빈 목록.
        """
        raw_hands = self.camera.latest_hand_landmarks()
        if not raw_hands:
            return []
        return self.hand_tracker.process_landmarks(raw_hands, T_ee, frame_shape)

    # ------------------------------------------------------------------
    # 판단 → 동작
    # ------------------------------------------------------------------
    def drive_arm(self, faces, hands, bodies, T_ee, q, now):
        """추종 상태 머신을 갱신하고, 간격이 되면 실제 하드웨어 명령을 낸다.

        판단(`next_command`)은 **매 프레임** 부른다 — 상태 머신 내부의
        추종/복귀/두리번거리기 상태가 최신이어야 하기 때문. 하지만 실제 명령은
        COMMAND_MIN_INTERVAL_S 간격으로만 보낸다: 20fps로 새 목표를 계속 주면
        팔이 매번 움직임을 새로 시작해 덜덜거린다.
        """
        command = self.follow_controller.next_command(faces, hands, bodies, T_ee, q)
        if command is None or not self.arm_service.connected:
            return
        if now - self._last_command_time < self.COMMAND_MIN_INTERVAL_S:
            return
        self._last_command_time = now
        try:
            self.arm_service.execute(command)
        except HardwareUnavailable:
            pass          # 도중에 연결이 끊겼다 — 인식/미리보기는 계속한다
        except Exception as e:
            # 시리얼 오류 하나로 카메라 루프가 죽으면 안 된다.
            Logger.log("LOOP", f"arm command failed: {e}")

    def update_gestures(self, hands, frame_shape):
        """가위바위보를 판정한다(배선이 없으면 아무것도 하지 않는다).

        같은 모양을 계속 들고 있어도 확정되는 순간 한 번만 발동한다(엣지
        트리거) — 중복 방지는 `GestureRecognizer`가 하므로 여기서 또 하지 않는다.
        """
        if self.gesture_bridge is None:
            return None
        return self.gesture_bridge.update(hands, frame_shape)

    # ------------------------------------------------------------------
    # 미리보기 (로봇 동작과 무관한 디버그 창)
    # ------------------------------------------------------------------
    def update_preview(self, frame, faces, hands, bodies, gesture, q, T_ee, now):
        """카메라 창과 3D 씬 창을 갱신한다(간격 제한 있음).

        이 창들은 로봇의 동작에 아무 영향을 주지 않는 디버그 보조물인데,
        matplotlib 3D 렌더링이 인식보다 훨씬 비싸다 — 매 프레임 다시 그리면
        그 렌더링이 병목이 되어 폰이 보내는 프레임을 못 따라간다. 그래서 판단
        루프와 별개로 훨씬 느리게 갱신한다.
        """
        if not self.show_preview:
            return
        if now - self._last_vis_time < self.VIS_MIN_INTERVAL_S:
            return
        self._last_vis_time = now

        self.face_tracker.draw_overlay(frame, faces)
        self.hand_tracker.draw_overlay(frame, hands)
        self.body_tracker.draw_overlay(frame, bodies)
        if gesture is not None and self.gesture_bridge is not None:
            self.gesture_bridge.draw(frame, gesture)
        cv2.imshow(AppConst.WINDOW_CAMERA, frame)

        scene = self.scene.draw(q, T_ee, hands, faces, bodies,
                                self.hand_tracker, self.face_tracker, self.body_tracker)
        cv2.imshow(AppConst.WINDOW_SCENE, scene)

    def close(self):
        """창을 닫고 mediapipe Pose 세션을 정리한다(얼굴/손은 인식을 휴대폰이
        하므로 서버가 mediapipe 세션을 만드는 건 몸 인식(BodyTracker)뿐)."""
        cv2.destroyAllWindows()
        self.body_tracker.close()
