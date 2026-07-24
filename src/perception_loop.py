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

[몸 폴백도 결국 휴대폰으로] 얼굴 인식이 실패하는 경우(옆모습, 고개를 돌린
경우 등)를 위한 사람 몸(어깨) 인식 폴백은 처음엔 "사람 인식은 서버가 직접"
하도록 명시적으로 요청받아 서버 mediapipe Pose로 돌았지만, 얼굴/손과 똑같은
이유(서버가 프레임마다 여러 모델 추론 + matplotlib 렌더링 + 하드웨어 제어를
다 하면 폰의 프레임레이트를 못 따라간다)로 이후 몸 인식도 휴대폰으로
옮겼다 — 지금은 셋 다 같은 경로다:

    휴대폰(MediaPipe Tasks Vision) → /ws/camera → Camera.body_landmarks
        → BodyTracker.process_landmarks() → FollowController (팔 추종)

이제 서버는 얼굴/손/몸 어느 쪽도 모델 추론을 하지 않는다 — 셋 다 좌표 변환만
한다. mediapipe(Python)는 더 이상 이 루프(이 프로젝트 전체)의 의존성이 아니다.

`FollowController`의 우선순위는 얼굴 > 몸 > 손 > idle이다(거리 기반 얼굴<->몸
전환 + 히스테리시스 + 디바운스는 perception/follow_controller.py 참고) —
몸 인식이 이제 서버 추론이 아니므로, 그 우선순위 판정과 무관하게 손처럼 매
프레임 그냥 수신한 좌표를 변환한다(아래 `collect_bodies()`).

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

        # 손/얼굴/몸 인식은 셋 다 휴대폰이 한다 — 이 객체들은 좌표 변환/시각화만
        # 담당한다(모델 추론 없음, mediapipe 의존성 없음).
        self.hand_tracker = HandTracker()
        self.face_tracker = FaceTracker()
        self.body_tracker = BodyTracker()
        # 얼굴 > 몸 > 손 > idle 우선순위와 두리번거리기 상태 머신.
        self.follow_controller = FollowController(arm_service.arm)

        self.scene = ScenePreview(renderer) if show_preview else None

        self._last_frame_count = 0
        self._last_command_time = 0.0
        self._last_vis_time = 0.0

        # 카메라 연결과 무관하게 켜자마자 IDLE_POSITION으로 보낸다 — 이
        # 이동을 next_command()(카메라 프레임이 와야 도는)에만 맡기면, 폰이
        # 아직 한 번도 안 붙었을 때 팔이 이전 세션의 자세 그대로 계속
        # 가만히 있게 된다. FollowController의 STARTUP_IDLE_S(생성 직후
        # 몇 초간 무슨 대상이 보여도 무시하는 것)와는 별개 — 그건 "카메라가
        # 붙은 뒤에도 곧장 추적을 시작하지 않는다"는 뜻이고, 이건 "카메라가
        # 붙기 전에도 최소한 idle 자세로는 보내 둔다"는 뜻이다.
        self._move_to_idle_on_startup()

    def _move_to_idle_on_startup(self):
        """전원을 켜자마자 팔을 IDLE_POSITION으로 이동시킨다(하드웨어가
        있을 때만). 시리얼 오류 하나로 앱 시작이 막히면 안 되므로
        HardwareUnavailable을 포함해 실패는 조용히 무시한다 — 이후
        정상적인 인식/추종 루프가 시작되면 어차피 같은 자리로 복귀를
        다시 시도한다."""
        if not self.arm_service.connected:
            return
        try:
            self.arm_service.execute(("position", FollowControllerConst.IDLE_POSITION))
        except HardwareUnavailable:
            pass
        except Exception as e:
            Logger.log("LOOP", f"startup idle move failed: {e}")

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
        bodies = self.collect_bodies(T_ee, frame.shape)
        hands = self.collect_hands(T_ee, frame.shape)

        now = time.monotonic()
        self.drive_arm(faces, hands, bodies, T_ee, q, now, self.camera.connected())
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

    def collect_bodies(self, T_ee, frame_shape):
        """휴대폰이 보낸 몸(어깨) 랜드마크를 월드 좌표 `Body` 목록으로 바꾼다.

        collect_faces()/collect_hands()와 같은 이유로 같은 모양이다 — 얼굴
        인식이 실패했을 때의 폴백이지만, 그 판단(얼굴<->몸 우선순위)은
        FollowController가 하므로 여기서는 얼굴 여부와 무관하게 매 프레임
        그냥 수신한 좌표를 변환한다. 폰이 아직 아무것도 안 보냈거나 어깨
        신뢰도가 낮아 이번 프레임엔 안 보냈으면 빈 목록.
        """
        raw_bodies = self.camera.latest_body_landmarks()
        if not raw_bodies:
            return []
        return self.body_tracker.process_landmarks(raw_bodies, T_ee, frame_shape)

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
    def drive_arm(self, faces, hands, bodies, T_ee, q, now, camera_connected):
        """추종 상태 머신을 갱신하고, 간격이 되면 실제 하드웨어 명령을 낸다.

        판단(`next_command`)은 **매 프레임** 부른다 — 상태 머신 내부의
        추종/복귀/두리번거리기 상태가 최신이어야 하기 때문. 하지만 실제 명령은
        COMMAND_MIN_INTERVAL_S 간격으로만 보낸다: 20fps로 새 목표를 계속 주면
        팔이 매번 움직임을 새로 시작해 덜덜거린다. camera_connected는
        `FollowController`가 idle 중 두리번거리기로 넘어갈지 판단하는 데
        쓴다(perception.follow_controller 모듈 docstring 참고).
        """
        command = self.follow_controller.next_command(faces, hands, bodies, T_ee, q, camera_connected)
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
        """미리보기 창을 닫는다 — 얼굴/손/몸 어느 트래커도 인식을 휴대폰이
        하므로(모델 추론 없음) 서버 쪽에 정리할 mediapipe 세션이 없다."""
        cv2.destroyAllWindows()
