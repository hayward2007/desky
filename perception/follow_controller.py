"""얼굴/손 인식 결과로 팔이 다음에 뭘 해야 할지 결정하는 최상위 상태 머신.

perception.face_tracker.FaceFollower와 perception.hand_tracker.HandFollower를
그대로 조립해서 쓴다 — 우선순위는 얼굴 > 손 > idle:

- 얼굴이 보이면 FaceFollower로 얼굴을 추적한다.
- 얼굴이 없고 손이 보이면 HandFollower로 손을 추적한다.
- 둘 다 안 보이면 IDLE_POSITION으로 복귀한다(RETURNING 상태). 복귀가 끝나면
  1번 관절(yaw)을 idle 자세 기준으로 좌우 왕복시키며 "두리번거리는" 동작을
  한다(LOOKING_AROUND 상태). 추적 대상이 다시 나타나면 어느 상태에 있든 다음
  프레임에 바로 추적으로 돌아간다.

FaceFollower/HandFollower와 같은 규칙을 따른다 — 실제 하드웨어 이동(IK 계산 +
서보 명령)은 이 모듈이 하지 않는다. `next_command()`가 돌려준 (kind, payload)를
호출부(`src/app.py`)가 `arm_ctrl.goto_position()`/`goto_joints()`에 넘겨야
로봇이 실제로 움직인다.
"""

import math
import time

from .face_tracker import FaceFollower
from .hand_tracker import HandFollower
from fundamental.const import FollowControllerConst


class FollowController:
    """얼굴/손/idle 사이를 오가며 다음 명령을 결정하는 상태 객체."""

    # 상수 설명은 fundamental.const.FollowControllerConst 참고.
    IDLE_POSITION = FollowControllerConst.IDLE_POSITION
    RETURN_TOLERANCE_M = FollowControllerConst.RETURN_TOLERANCE_M
    LOOKAROUND_AMPLITUDE = FollowControllerConst.LOOKAROUND_AMPLITUDE
    LOOKAROUND_PERIOD_S = FollowControllerConst.LOOKAROUND_PERIOD_S

    def __init__(self, arm, face_follower=None, hand_follower=None):
        self.arm = arm
        self.face_follower = face_follower if face_follower is not None else FaceFollower(arm=arm)
        self.hand_follower = hand_follower if hand_follower is not None else HandFollower()
        self._yaw_index = arm.id_to_index[1]
        # "tracking": 얼굴/손을 추적 중(또는 방금까지 추적하다 막 놓친 프레임).
        # "returning": idle로 복귀 중. "looking_around": idle에서 두리번거리는 중.
        self._state = "tracking"
        self._lookaround_t0 = None
        self._lookaround_base_q = None

    def next_command(self, faces, hands, T_ee, current_q):
        """다음에 실행할 명령을 (kind, payload)로 반환하거나, 할 일이 없으면
        None을 반환한다.

        kind="position": payload는 (x, y, z) — goto_position(IK)로 이동.
        kind="joints"  : payload는 servo_deg 리스트 — goto_joints로 이동.
        얼굴/손이 보이는 동안은 각 추적기의 판단(데드존 포함)을 그대로 넘긴다 —
        즉 얼굴이 보여도 화면 중앙 근처면 None(가만히 있기)일 수 있다. 얼굴도
        손도 안 보일 때만 idle 상태 머신으로 넘어간다.
        """
        face = self.face_follower.primary_face(faces)
        if face is not None:
            self._state = "tracking"
            return self.face_follower.next_command(faces, T_ee, current_q)

        hand_target = self.hand_follower.combined_target(hands)
        if hand_target is not None:
            self._state = "tracking"
            target = self.hand_follower.next_ee_target(hands, T_ee)
            return ("position", target) if target is not None else None

        return self._idle_step(T_ee, current_q)

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
