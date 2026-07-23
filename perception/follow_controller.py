"""얼굴/몸/손 인식 결과로 팔이 다음에 뭘 해야 할지 결정하는 최상위 상태 머신.

perception.face_tracker.FaceFollower, perception.body_tracker.BodyFollower,
perception.hand_tracker.HandFollower를 그대로 조립해서 쓴다 — 우선순위는
얼굴 > 몸 > 손 > idle이 기본이지만, 얼굴은 **거리에 따라 몸에게 자리를
내준다**:

- 얼굴까지 거리가 `FollowControllerConst.FACE_BODY_SWITCH_DISTANCE_M`(기본
  1m)보다 가까우면 FaceFollower로 얼굴을 추적한다.
- 그보다 멀면(또는 얼굴이 아예 안 보이면) 몸(어깨)이 보이는 한 BodyFollower로
  몸을 추적한다 — 멀리서는 얼굴 랜드마크(특히 깊이 추정)가 더 부정확해지고
  화면에서 작아져 노이즈에 약해지는데, 어깨는 상대적으로 크고 안정적으로
  잡히기 때문. 얼굴이 멀리서라도 보이는데 몸이 안 보이면 그래도 얼굴을 쓴다
  (아무것도 안 쓰는 것보다는 낫다). 몸 인식은 얼굴/손과 달리 서버에서 직접
  mediapipe Pose로 돈다(perception/body_tracker.py 참고).
- 이 거리 판정에는 히스테리시스(`FollowControllerConst.FACE_BODY_SWITCH_HYSTERESIS_M`)와
  깊이값 자체의 평활(`FACE_DEPTH_SMOOTHING`)이 같이 걸린다 — 얼굴 깊이
  추정치가 경계(1m) 바로 근처에서 맴돌면(랜드마크 잔떨림, 고개를 살짝
  돌리는 정도로도 흔함) 단순 부등호 비교로는 얼굴<->몸 판정이 프레임마다
  뒤집혀서, 아래 디바운스가 있어도 "몇 초에 한 번씩 계속 갈아타며 위아래로
  훑는" 느린 진동이 남았다. 그래서 지금 얼굴을 보는 중이면 거리가
  `FACE_BODY_SWITCH_DISTANCE_M + FACE_BODY_SWITCH_HYSTERESIS_M`보다 멀어져야
  몸으로, 지금 몸을 보는 중이면 그보다 `- FACE_BODY_SWITCH_HYSTERESIS_M`만큼
  가까워져야 얼굴로 되돌아간다(`_pick_target`) — 경계를 두 개로 벌려 놓은
  전형적인 데드밴드다.
- 얼굴도 몸도 없고 손이 보이면 HandFollower로 손을 추적한다 — 단
  `FollowControllerConst.HAND_FOLLOW_ENABLED`가 False(기본값)면 이 단계 자체를
  건너뛴다: 손은 계속 인식되고(휴대폰이 계속 랜드마크를 보내고) 가위바위보
  제스처(perception/gesture.py, src/perception_loop.py의 GestureBridge)도
  그 랜드마크로 그대로 동작하지만, 팔이 손 위치로 움직이지는 않는다 — "손
  인식/제스처는 켜 두고 싶지만 팔이 손을 쫓아다니는 것만 원치 않는다"는
  요청으로 추가된 스위치.
- 셋 다 안 보이거나(또는 손만 보이는데 HAND_FOLLOW_ENABLED가 꺼져 있으면)
  IDLE_POSITION으로 복귀한다(RETURNING 상태). 복귀가 끝나면
  1번 관절(yaw)을 idle 자세 기준으로 좌우 왕복시키며 "두리번거리는" 동작을
  한다(LOOKING_AROUND 상태).

**추적 대상 전환은 디바운스된다**(`_pick_target`/`TARGET_SWITCH_TIMEOUT_S`) —
얼굴/몸 인식은 프레임마다 100% 안정적으로 성공하지 않아서, 한두 프레임
놓쳤다고 바로 다른 대상으로(또는 idle로) 갈아타면 코끝(얼굴 중심)과 어깨
중점(몸 중심)의 화면상 높이가 달라서 팔이 목표를 바꿀 때마다 높이(z)가
왔다갔다 흔들리는 문제가 있었다. 그래서 지금 추적 중인 대상이 사라지거나
다른 대상이 더 나아 보여도, 그 상태가 `TARGET_SWITCH_TIMEOUT_S`(기본 0.5초)
동안 끊기지 않고 이어져야 실제로 전환한다 — 그 유예 기간 동안은 마지막
대상을 그대로 쓰고(이번 프레임에 그 데이터가 없으면 팔은 가만히 있는다).
반대로 idle 상태에서 뭔가 새로 나타났을 때는 디바운스 없이 바로 추적을
시작한다(반응성을 위해).

FaceFollower/BodyFollower/HandFollower와 같은 규칙을 따른다 — 실제 하드웨어
이동(IK 계산 + 서보 명령)은 이 모듈이 하지 않는다. `next_command()`가 돌려준
(kind, payload)를 호출부(`src/perception_loop.py`)가
`arm_ctrl.goto_position()`/`goto_joints()`에 넘겨야 로봇이 실제로 움직인다.
"""

import math
import time

from .body_tracker import BodyFollower
from .face_tracker import FaceFollower
from .hand_tracker import HandFollower
from fundamental.const import FollowControllerConst

# _pending_target의 "지금 디바운스 타이머가 안 돌고 있음" 상태를 나타내는
# 표식. None은 "대상이 전혀 없음(idle)"이라는 유효한 후보값이라 그 자체로는
# 못 쓴다 — None과 구분하기 위한 별도 객체.
_NO_PENDING = object()


class FollowController:
    """얼굴/몸/손/idle 사이를 오가며 다음 명령을 결정하는 상태 객체."""

    # 상수 설명은 fundamental.const.FollowControllerConst 참고.
    IDLE_POSITION = FollowControllerConst.IDLE_POSITION
    RETURN_TOLERANCE_M = FollowControllerConst.RETURN_TOLERANCE_M
    LOOKAROUND_AMPLITUDE = FollowControllerConst.LOOKAROUND_AMPLITUDE
    LOOKAROUND_PERIOD_S = FollowControllerConst.LOOKAROUND_PERIOD_S
    HAND_FOLLOW_ENABLED = FollowControllerConst.HAND_FOLLOW_ENABLED
    FACE_BODY_SWITCH_DISTANCE_M = FollowControllerConst.FACE_BODY_SWITCH_DISTANCE_M
    FACE_BODY_SWITCH_HYSTERESIS_M = FollowControllerConst.FACE_BODY_SWITCH_HYSTERESIS_M
    FACE_DEPTH_SMOOTHING = FollowControllerConst.FACE_DEPTH_SMOOTHING
    TARGET_SWITCH_TIMEOUT_S = FollowControllerConst.TARGET_SWITCH_TIMEOUT_S

    def __init__(self, arm, face_follower=None, body_follower=None, hand_follower=None,
                 hand_follow_enabled=HAND_FOLLOW_ENABLED,
                 face_body_switch_distance_m=FACE_BODY_SWITCH_DISTANCE_M,
                 face_body_switch_hysteresis_m=FACE_BODY_SWITCH_HYSTERESIS_M,
                 face_depth_smoothing=FACE_DEPTH_SMOOTHING,
                 target_switch_timeout_s=TARGET_SWITCH_TIMEOUT_S):
        self.arm = arm
        self.face_follower = face_follower if face_follower is not None else FaceFollower(arm=arm)
        self.body_follower = body_follower if body_follower is not None else BodyFollower(arm=arm)
        self.hand_follower = hand_follower if hand_follower is not None else HandFollower()
        self.hand_follow_enabled = hand_follow_enabled
        self.face_body_switch_distance_m = face_body_switch_distance_m
        self.face_body_switch_hysteresis_m = face_body_switch_hysteresis_m
        self.face_depth_smoothing = face_depth_smoothing
        self.target_switch_timeout_s = target_switch_timeout_s
        self._yaw_index = arm.id_to_index[1]
        # "tracking": 얼굴/몸/손을 추적 중(또는 방금까지 추적하다 막 놓친 프레임).
        # "returning": idle로 복귀 중. "looking_around": idle에서 두리번거리는 중.
        self._state = "tracking"
        self._lookaround_t0 = None
        self._lookaround_base_q = None
        # 디바운스 상태: 지금 실제로 쓰고 있는 대상("face"/"body"/"hand"/None)과,
        # 그와 다른 대상으로 갈아타려는 시도가 얼마나 계속됐는지.
        self._active_target = None
        self._pending_target = _NO_PENDING
        self._pending_since = None
        # 얼굴<->몸 전환 판정에만 쓰는, 별도로 평활된 얼굴 깊이(m). FaceFollower가
        # 화면 오프셋에 거는 평활과는 독립된 상태 — 얼굴을 놓치면 None으로
        # 리셋한다(_smooth_face_depth 참고).
        self._smoothed_face_depth = None

    def next_command(self, faces, hands, bodies, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload)로 반환하거나, 할 일이 없으면
        None을 반환한다.

        kind="position": payload는 (x, y, z) — goto_position(IK)로 이동.
        kind="joints"  : payload는 servo_deg 리스트 — goto_joints로 이동.
        얼굴/몸/손이 보이는 동안은 각 추적기의 판단(데드존 포함)을 그대로
        넘긴다 — 즉 보여도 화면 중앙 근처면 None(가만히 있기)일 수 있다.
        아무 대상도 없을 때만(디바운스 유예 기간이 다 지나야) idle 상태
        머신으로 넘어간다.
        """
        face = self.face_follower.primary_face(faces)
        body = self.body_follower.primary_body(bodies)
        hand_target = self.hand_follower.combined_target(hands) if self.hand_follow_enabled else None
        face_depth = self._smooth_face_depth(face)

        self._update_active_target(face, body, hand_target, face_depth)

        if self._active_target == "face" and face is not None:
            self._state = "tracking"
            return self.face_follower.next_command(faces, T_ee, current_q)
        if self._active_target == "body" and body is not None:
            self._state = "tracking"
            return self.body_follower.next_command(bodies, T_ee, current_q)
        if self._active_target == "hand" and hand_target is not None:
            self._state = "tracking"
            target = self.hand_follower.next_ee_target(hands, T_ee)
            return ("position", target) if target is not None else None
        if self._active_target is not None:
            # 디바운스 유예 기간 중이라 대상은 유지 중이지만, 이번 프레임엔
            # 그 대상의 데이터가 없다 — 새 목표를 계산할 근거가 없으니 그냥
            # 가만히 있는다(괜히 옛날 좌표로 움직이지 않음).
            return None

        return self._idle_step(T_ee, current_q)

    def _smooth_face_depth(self, face):
        """얼굴<->몸 전환 판정에만 쓰는 얼굴 깊이(m)에 EMA를 걸어 반환한다.

        `Face.depth`는 눈 사이 랜드마크 거리로 매 프레임 새로 역산한 값이라
        고개를 살짝 돌리거나 랜드마크가 잔떨리기만 해도 값이 흔들린다 —
        경계(FACE_BODY_SWITCH_DISTANCE_M) 근처에서 이 잔떨림이 그대로
        `_pick_target`에 들어가면 히스테리시스가 있어도 결국 넘나들게 된다.
        얼굴을 놓치면(face is None) 다음에 다시 잡았을 때 엉뚱한 옛 평균이
        남아있지 않도록 리셋한다.
        """
        if face is None:
            self._smoothed_face_depth = None
            return None
        if self._smoothed_face_depth is None:
            self._smoothed_face_depth = face.depth
        else:
            a = self.face_depth_smoothing
            self._smoothed_face_depth = self._smoothed_face_depth * (1 - a) + face.depth * a
        return self._smoothed_face_depth

    def _pick_target(self, face, body, hand_target, face_depth):
        """이번 프레임 데이터만 놓고 봤을 때 이상적인 추적 대상을 고른다
        (디바운스 반영 전). `face_depth`는 `_smooth_face_depth`가 평활한 값.

        얼굴<->몸 경계에는 히스테리시스를 둔다 — 지금 얼굴을 보는 중이면
        `FACE_BODY_SWITCH_DISTANCE_M + FACE_BODY_SWITCH_HYSTERESIS_M`보다
        멀어져야 몸으로 넘어가고, 지금 몸(또는 그 외)을 보는 중이면
        `FACE_BODY_SWITCH_DISTANCE_M - FACE_BODY_SWITCH_HYSTERESIS_M`보다
        가까워져야 얼굴로 돌아온다. 경계값 하나만 쓰면 깊이 추정치가 그
        근처에서 맴돌 때마다(평활을 걸어도 완전히는 안 없어진다) 판정 자체가
        매번 뒤집혀 아래 디바운스 타임아웃과 무관하게 결국 갈아타게 된다.
        """
        if face is not None:
            near_boundary = (
                self.face_body_switch_distance_m + self.face_body_switch_hysteresis_m
                if self._active_target == "face"
                else self.face_body_switch_distance_m - self.face_body_switch_hysteresis_m
            )
            if face_depth < near_boundary:
                return "face"
        if body is not None:
            return "body"
        if face is not None:
            return "face"  # 멀지만 몸이 안 보이면 그래도 얼굴이 낫다
        if hand_target is not None:
            return "hand"
        return None

    def _update_active_target(self, face, body, hand_target, face_depth):
        """`_active_target`을 갱신한다 — 새 후보가 `target_switch_timeout_s`
        동안 끊기지 않고 이어져야 실제로 전환한다(모듈 docstring 참고).
        idle(활성 대상 없음)에서 뭔가 나타났을 때만 예외적으로 즉시 전환한다.
        """
        ideal = self._pick_target(face, body, hand_target, face_depth)

        if self._active_target is None:
            self._active_target = ideal
            self._pending_target = _NO_PENDING
            self._pending_since = None
            return

        if ideal == self._active_target:
            self._pending_target = _NO_PENDING
            self._pending_since = None
            return

        now = time.monotonic()
        if ideal != self._pending_target:
            self._pending_target = ideal
            self._pending_since = now
        elif now - self._pending_since >= self.target_switch_timeout_s:
            self._active_target = ideal
            self._pending_target = _NO_PENDING
            self._pending_since = None

    def _idle_step(self, T_ee, current_q):
        """얼굴도 손도 안 보일 때: 먼저 IDLE_POSITION으로 복귀시키고, 도착하면
        1번 관절을 좌우로 왕복시키는 룩어라운드 명령을 계속 낸다."""
        if self._state == "tracking":
            self._state = "returning"

        if self._state == "returning":
            current_pos = (T_ee[0][3], T_ee[1][3], T_ee[2][3])
            if math.dist(current_pos, self.IDLE_POSITION) > self.RETURN_TOLERANCE_M:
                return "position", self.IDLE_POSITION

            # 도착 — 룩어라운드로 전환. 기준 자세(idle_q)는 지금 도착한 IK 해를
            # 한 번만 풀어서 캐시해두고, 이후 매 프레임 그 위에 yaw 오프셋만 얹는다.
            self._state = "looking_around"
            self._lookaround_t0 = time.monotonic()
            q_idle, converged = self.arm.ik(self.IDLE_POSITION, seed=current_q)
            self._lookaround_base_q = list(q_idle) if converged else list(current_q)

        elapsed = time.monotonic() - self._lookaround_t0
        offset = self.LOOKAROUND_AMPLITUDE * math.sin(2 * math.pi * elapsed / self.LOOKAROUND_PERIOD_S)
        q = list(self._lookaround_base_q)
        q[self._yaw_index] += offset
        return "joints", self.arm.q_to_servo_deg(q)
