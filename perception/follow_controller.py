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

**켠 직후 `STARTUP_IDLE_S`(기본 3초) 동안은 무조건 idle을 유지한다** —
`FollowController`가 생성된 시점(앱/로봇 전원을 막 켠 시점)부터 이 시간
동안은 얼굴/몸/손이 보여도 전부 무시한다(`next_command()` 맨 앞에서 처리,
`_pick_target`/디바운스 로직 자체를 타지 않는다). 카메라가 웜업되기 전이거나
전원 인가 직후 사람이 이미 카메라 앞에 있으면, 그 순간 곧바로 팔이 홱
움직이는 걸 막기 위함이다. 이 시간이 지나면 그 시점에 보이는 대상으로
디바운스 없이 바로 추적을 시작한다(idle에서 뭔가 나타났을 때의 기존
규칙과 동일).

**폰의 인식 파이프라인이 실제로 돌기 시작하기 전에는 idle 중
두리번거리기(LOOKING_AROUND)로 넘어가지 않는다** — `next_command()`가 받는
`camera_connected` 인자(호출부 `src/perception_loop.py`가
`Camera.connected()`로 넘겨준다)가 False이면 `_idle_step()`은
IDLE_POSITION까지만 복귀하고 그 자세를 가만히 유지할 뿐, 좌우로 흔드는
룩어라운드는 시작하지 않는다. 단순히 "웹소켓이 붙어 있는지"만 보면 안
되는 이유가 있다 — `getUserMedia`/웹소켓은 붙어서 JPEG 프레임을 계속
보내고 있어도, `mobile.html`의 Hand/Face/PoseLandmarker는 모델
다운로드+WASM 초기화 때문에 그보다 몇 초 늦게 준비되는 경우가 흔하다. 그
동안은 `collect_faces()`/`collect_bodies()`/`collect_hands()`가 전부 빈
목록을 돌려주므로, 소켓 연결 여부만 봤다면 인식이 단 한 번도 돌기 전에
곧장 두리번거리기가 시작돼 버렸을 것이다. 그래서 `Camera.connected()`는
소켓이 붙어 있을 뿐 아니라 손/얼굴/몸 랜드마크 중 하나라도 최초 메시지를
받은 적이 있어야 True다(`src/api/camera.py` 참고). 두리번거리는 중에
연결이 끊기면(폰 새로고침 등) 그 자리에서 멈추고 다시 대기 상태로
돌아간다.

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

**얼굴을 잃을 때만 유예를 더 길게 준다**(`FACE_LOST_GRACE_S`) — 위
`TARGET_SWITCH_TIMEOUT_S`(0.5초)는 모든 전환에 공통으로 쓰이지만, 얼굴
인식은 고개를 살짝 돌리거나 눈을 깜빡이는 정도로도 한두 프레임 끊기는 일이
흔해서 0.5초로는 몸(어깨)으로 너무 쉽게 넘어가 버렸다. 그래서 지금 얼굴을
추적 중일 때만(`_active_target == "face"`) 몸/기타로 전환하는 판정에
`FACE_LOST_GRACE_S`(기본 1.5초, `TARGET_SWITCH_TIMEOUT_S`보다 김)를 대신
쓴다 — "얼굴이 잠깐 안 보인다고 바로 어깨로 넘기지 말고 다시 찾을 시간을
더 주자"는 요청. 몸->얼굴을 포함한 다른 모든 전환은 여전히
`TARGET_SWITCH_TIMEOUT_S`를 쓴다.

**몸을 추적하며 멈춰 있으면 한 번 살짝 들어 올려 얼굴을 다시 찾아본다**
(`FACE_REACQUIRE_STILL_S`) — 몸(어깨) 추적 중 화면 오프셋이 데드존 안이라
팔이 `FACE_REACQUIRE_STILL_S`(기본 2초) 동안 계속 가만히 있었고, 거리가
`FACE_BODY_SWITCH_DISTANCE_M`보다 가까우면(멀면 애초에 얼굴을 잡을
가능성이 낮으니 시도하지 않는다) 팔을 `FACE_REACQUIRE_HEIGHT_STEP_M`만큼
한 번 들어 올린다. 어깨 중심으로 카메라를 맞추면 각도상 얼굴이 화면 위쪽
경계 밖에 걸쳐 있을 수 있는데, 조금만 올려도 다시 잡히는 경우가 많기
때문. 들어 올린 뒤에는 `FACE_REACQUIRE_SUPPRESS_S` 동안 `BodyFollower`의
보정 명령을 무시하고 그 자세를 유지하며 얼굴이 다시 잡히길 기다린다(안
그러면 BodyFollower가 몸 기준으로 즉시 다시 낮춰버려서 들어 올린 의미가
없어진다). 이 몸 추적 세션 동안은 한 번만 시도한다(`_reacquire_attempted`)
— 계속 실패해도 무한정 위로 계속 들리지 않도록. 대상이 바뀌면(얼굴을
다시 찾거나, idle로 넘어가는 등) 이 상태는 전부 리셋되어 다음에 몸을
추적하게 되면 다시 한 번 시도할 수 있다(`_enter_target`).

FaceFollower/BodyFollower/HandFollower와 같은 규칙을 따른다 — 실제 하드웨어
이동(IK 계산 + 서보 명령)은 이 모듈이 하지 않는다. `next_command()`가 돌려준
(kind, payload)를 호출부(`src/perception_loop.py`)가
`arm_ctrl.goto_position()`/`goto_joints()`에 넘겨야 로봇이 실제로 움직인다.
"""

import math
import time

from .body_tracker import BodyFollower
from .camera_geometry import clamp_xy
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
    FACE_LOST_GRACE_S = FollowControllerConst.FACE_LOST_GRACE_S
    FACE_REACQUIRE_STILL_S = FollowControllerConst.FACE_REACQUIRE_STILL_S
    FACE_REACQUIRE_HEIGHT_STEP_M = FollowControllerConst.FACE_REACQUIRE_HEIGHT_STEP_M
    FACE_REACQUIRE_SUPPRESS_S = FollowControllerConst.FACE_REACQUIRE_SUPPRESS_S
    STARTUP_IDLE_S = FollowControllerConst.STARTUP_IDLE_S

    def __init__(self, arm, face_follower=None, body_follower=None, hand_follower=None,
                 hand_follow_enabled=HAND_FOLLOW_ENABLED,
                 face_body_switch_distance_m=FACE_BODY_SWITCH_DISTANCE_M,
                 face_body_switch_hysteresis_m=FACE_BODY_SWITCH_HYSTERESIS_M,
                 face_depth_smoothing=FACE_DEPTH_SMOOTHING,
                 target_switch_timeout_s=TARGET_SWITCH_TIMEOUT_S,
                 face_lost_grace_s=FACE_LOST_GRACE_S,
                 face_reacquire_still_s=FACE_REACQUIRE_STILL_S,
                 face_reacquire_height_step_m=FACE_REACQUIRE_HEIGHT_STEP_M,
                 face_reacquire_suppress_s=FACE_REACQUIRE_SUPPRESS_S,
                 startup_idle_s=STARTUP_IDLE_S):
        self.arm = arm
        self.face_follower = face_follower if face_follower is not None else FaceFollower(arm=arm)
        self.body_follower = body_follower if body_follower is not None else BodyFollower(arm=arm)
        self.hand_follower = hand_follower if hand_follower is not None else HandFollower()
        self.hand_follow_enabled = hand_follow_enabled
        self.face_body_switch_distance_m = face_body_switch_distance_m
        self.face_body_switch_hysteresis_m = face_body_switch_hysteresis_m
        self.face_depth_smoothing = face_depth_smoothing
        self.target_switch_timeout_s = target_switch_timeout_s
        self.face_lost_grace_s = face_lost_grace_s
        self.face_reacquire_still_s = face_reacquire_still_s
        self.face_reacquire_height_step_m = face_reacquire_height_step_m
        self.face_reacquire_suppress_s = face_reacquire_suppress_s
        self.startup_idle_s = startup_idle_s
        # 이 컨트롤러가 만들어진 시점(앱/로봇 전원을 켠 시점) — next_command()가
        # 이 시점 기준 startup_idle_s가 지날 때까지는 무조건 idle을 유지한다.
        self._start_time = time.monotonic()
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
        # 몸 추적 중 "가만히 있은 지 얼마나 됐는지"(_still_since), 얼굴
        # 재인식을 위해 들어 올린 자세를 언제까지 유지할지(_reacquire_until),
        # 이번 몸 추적 세션에서 이미 한 번 시도했는지(_reacquire_attempted) —
        # 모듈 docstring의 "몸을 추적하며 멈춰 있으면..." 절 참고. 대상이
        # 바뀔 때마다(_enter_target) 리셋된다.
        self._still_since = None
        self._reacquire_until = None
        self._reacquire_attempted = False

    def next_command(self, faces, hands, bodies, T_ee, current_q, camera_connected=True):
        """다음에 실행할 명령을 (kind, payload)로 반환하거나, 할 일이 없으면
        None을 반환한다.

        kind="position": payload는 (x, y, z) — goto_position(IK)로 이동.
        kind="joints"  : payload는 servo_deg 리스트 — goto_joints로 이동.
        얼굴/몸/손이 보이는 동안은 각 추적기의 판단(데드존 포함)을 그대로
        넘긴다 — 즉 보여도 화면 중앙 근처면 None(가만히 있기)일 수 있다.
        아무 대상도 없을 때만(디바운스 유예 기간이 다 지나야) idle 상태
        머신으로 넘어간다. 켠 직후 `startup_idle_s` 동안은 무엇이 보이든
        무조건 idle을 유지한다(모듈 docstring 참고).

        camera_connected: 폰이 지금 `/ws/camera`에 붙어 있는지
        (`src.api.camera.Camera.connected()`). False면 idle이더라도
        두리번거리기로 넘어가지 않고 IDLE_POSITION에서 가만히 대기한다
        (모듈 docstring의 "폰이 실제로 연결돼 있지 않으면..." 절 참고).
        기본값 True는 이 인자를 안 넘기던 기존 호출부/테스트와의 호환용.
        """
        if time.monotonic() - self._start_time < self.startup_idle_s:
            return self._idle_step(T_ee, current_q, camera_connected)

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
            return self._body_track_command(body, bodies, T_ee, current_q)
        if self._active_target == "hand" and hand_target is not None:
            self._state = "tracking"
            target = self.hand_follower.next_ee_target(hands, T_ee)
            return ("position", target) if target is not None else None
        if self._active_target is not None:
            # 디바운스 유예 기간 중이라 대상은 유지 중이지만, 이번 프레임엔
            # 그 대상의 데이터가 없다 — 새 목표를 계산할 근거가 없으니 그냥
            # 가만히 있는다(괜히 옛날 좌표로 움직이지 않음).
            return None

        return self._idle_step(T_ee, current_q, camera_connected)

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

    def _body_track_command(self, body, bodies, T_ee, current_q):
        """몸(어깨) 추적 명령을 계산한다 — 평소엔 `BodyFollower`에 그대로
        맡기지만, 얼굴 재인식을 위해 들어 올린 직후에는 그 자세를 유지하고,
        오래 가만히 있었으면 한 번 들어 올리는 시도를 끼워 넣는다(모듈
        docstring의 "몸을 추적하며 멈춰 있으면..." 절 참고).
        """
        now = time.monotonic()
        if self._reacquire_until is not None:
            if now < self._reacquire_until:
                return None  # 들어 올린 자세 유지 — BodyFollower가 도로 낮추지 못하게
            self._reacquire_until = None

        command = self.body_follower.next_command(bodies, T_ee, current_q)
        if command is not None:
            self._still_since = None  # 실제로 보정 명령이 나갔으니 "가만히 있음"이 아니다
            return command

        # 데드존 안 — 화면상 가만히 있는 중.
        if self._still_since is None:
            self._still_since = now
            return None
        if self._reacquire_attempted or body.depth >= self.face_body_switch_distance_m:
            return None
        if now - self._still_since < self.face_reacquire_still_s:
            return None

        self._reacquire_attempted = True
        self._reacquire_until = now + self.face_reacquire_suppress_s
        current_z = T_ee[2][3]
        target = clamp_xy((0.0, 0.0, current_z + self.face_reacquire_height_step_m))
        q_target, converged = self.arm.ik(target, seed=current_q)
        if not converged:
            return None
        return "joints", self.arm.q_to_servo_deg(q_target)

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
        """`_active_target`을 갱신한다 — 새 후보가 디바운스 시간 동안 끊기지
        않고 이어져야 실제로 전환한다(모듈 docstring 참고). 지금 얼굴을
        추적 중이면 그 디바운스 시간은 `target_switch_timeout_s`가 아니라
        더 긴 `face_lost_grace_s`를 쓴다(얼굴을 잃을 때만 더 참을성 있게).
        idle(활성 대상 없음)에서 뭔가 나타났을 때만 예외적으로 즉시 전환한다.
        """
        ideal = self._pick_target(face, body, hand_target, face_depth)

        if self._active_target is None:
            self._enter_target(ideal)
            return

        if ideal == self._active_target:
            self._pending_target = _NO_PENDING
            self._pending_since = None
            return

        now = time.monotonic()
        if ideal != self._pending_target:
            self._pending_target = ideal
            self._pending_since = now
            return

        timeout = (
            self.face_lost_grace_s if self._active_target == "face"
            else self.target_switch_timeout_s
        )
        if now - self._pending_since >= timeout:
            self._enter_target(ideal)

    def _enter_target(self, target):
        """`_active_target`을 바꾸고 디바운스/재인식 상태를 전부 리셋한다 —
        이전 대상에서 쌓인 타이머(디바운스 대기, 몸 정지 시간, 재인식 시도
        여부)가 새 대상에 잘못 이어지지 않도록 한다."""
        self._active_target = target
        self._pending_target = _NO_PENDING
        self._pending_since = None
        self._still_since = None
        self._reacquire_until = None
        self._reacquire_attempted = False

    def _idle_step(self, T_ee, current_q, camera_connected):
        """얼굴도 손도 안 보일 때: 먼저 IDLE_POSITION으로 복귀시키고, 도착하면
        1번 관절을 좌우로 왕복시키는 룩어라운드 명령을 계속 낸다 — 단
        `camera_connected`가 False면 복귀까지만 하고 그 자리에서 가만히
        대기한다(모듈 docstring의 "폰이 실제로 연결돼 있지 않으면..." 절
        참고). 룩어라운드 중에 연결이 끊기면(폰 새로고침 등) 그 자리에서
        멈추고 다시 대기 상태로 돌아간다."""
        if self._state == "tracking":
            self._state = "returning"

        if self._state == "looking_around" and not camera_connected:
            self._state = "returning"

        if self._state == "returning":
            current_pos = (T_ee[0][3], T_ee[1][3], T_ee[2][3])
            if math.dist(current_pos, self.IDLE_POSITION) > self.RETURN_TOLERANCE_M:
                return "position", self.IDLE_POSITION

            if not camera_connected:
                return None  # 도착 — 연결 전이니 룩어라운드는 시작하지 않고 대기

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
