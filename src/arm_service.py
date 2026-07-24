"""로봇 팔의 '지금 상태'를 한 곳에서 책임지는 서비스 객체.

[병합 메모] 병합 전에는 이 파일의 내용이 전부 `src/app.py`의 모듈 전역 변수와
전역 함수였다(`arm_ctrl`, `arm`, `hardware_error`, `_q_cache`, `_current_q()`,
`_servo_degs_within_limits()` …). 두 브랜치가 모두 그 전역들을 건드리는 바람에
합칠 때 가장 크게 충돌한 부분이라, 아래 한 객체로 묶었다:

- 상태(하드웨어 핸들 · 팔 모델 · 관절각 캐시)가 한 객체 안에 모여, 웹 라우트
  (`src.api.arm.ArmAPI`)와 인식 루프(`src.perception_loop.PerceptionLoop`)가
  같은 인스턴스를 공유한다. 두 쪽이 각자 전역을 읽고 쓰던 구조가 사라진다.
- 하드웨어가 없어도 앱이 뜨는 기존 동작은 그대로다 — 연결에 실패하면
  `connected`가 False가 되고, 움직이는 메서드만 `HardwareUnavailable`을
  던진다(팔 모델을 쓰는 FK/IK/렌더링은 계속 동작).
- 테스트에서 가짜 팔을 넣기 쉬워진다(생성자 인자).

이 객체는 HTTP를 모른다 — Flask 응답(jsonify/상태코드)은 전부
`src.api.arm.ArmAPI`가 만든다. perception 패키지가 하드웨어를 모르는 것과
같은 방향의 분리다.
"""

import time

from fundamental.const import AppConst
from fundamental.logger import Logger
from kinematics.urdf_loader import load_arm


class HardwareUnavailable(RuntimeError):
    """하드웨어가 없는데 실제로 움직이려 했을 때 발생.

    `ArmAPI`가 이걸 잡아 503 "no hardware connected"로 바꾼다(병합 전 각
    라우트마다 `if arm_ctrl is None: return ... 503`을 반복하던 것을 한 곳으로
    모은 것).
    """


class ArmService:
    """팔 모델 + (있으면) 실제 액추에이터 + 관절각 캐시를 감싼 객체."""

    # 상수 설명은 fundamental.const.AppConst 참고.
    Q_CACHE_TTL_S = AppConst.Q_CACHE_TTL_S

    def __init__(self, actuator_ids=range(1, 6), model="AX-18A"):
        """액추에이터 연결을 시도하고, 실패하면 '하드웨어 없음' 상태로 남는다.

        actuator_ids: 연결할 DYNAMIXEL ID들(기본 1~5, 관절 순서와 같다).
        model       : 컨트롤 테이블을 고르는 모델명.
        """
        self.arm_ctrl = None
        self.hardware_error = None
        self._q_cache = {"q": None, "t": 0.0}

        try:
            # import를 try 안에 두는 이유: hardware.controller가 모듈 로드
            # 시점에 dynamixel_sdk를 import하므로, 그 패키지가 아예 없는 것도
            # 앱을 죽이는 대신 "하드웨어 없음"으로 처리해야 한다.
            from hardware.controller import Controller
            from hardware.actuator import Actuator, ArmController

            controller = Controller()
            actuators = [Actuator(id=i, model=model, controller=controller)
                         for i in actuator_ids]
            self.arm_ctrl = ArmController(actuators)
            Logger.log("ARM", "Hardware connected")
        except Exception as e:
            self.hardware_error = str(e)
            Logger.log("ARM", f"No hardware connected: {self.hardware_error}")

        # 하드웨어가 없어도 관절 목록은 필요하다 — 대시보드의 관절 슬라이더,
        # FK/IK 미리보기, 3D 렌더링이 전부 이 모델만으로 동작한다.
        self.arm = self.arm_ctrl.arm if self.arm_ctrl is not None else load_arm()

    # ------------------------------------------------------------------
    # 상태 조회
    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        """실제 액추에이터가 붙어 있는지 여부."""
        return self.arm_ctrl is not None

    @property
    def joints(self):
        """관절 목록(URDF 순서). 라우트/UI가 개수와 id를 물을 때 쓴다."""
        return self.arm.joints

    @property
    def actuators(self):
        """액추에이터 목록. 하드웨어가 없으면 빈 리스트."""
        return self.arm_ctrl.actuators if self.connected else []

    def current_q(self):
        """현재 관절각(rad)을 캐시해서 돌려준다. 하드웨어가 없으면 None.

        `/api/status` 폴링과 인식 루프의 3D 오버레이가 둘 다 현재 자세를
        필요로 하는데, 한 번 읽을 때마다 액추에이터 5개 각각에 DYNAMIXEL 시리얼
        왕복이 발생한다. 캐시가 없으면 두 호출부가 각자 실제 자세가 변하는
        속도보다 훨씬 자주 시리얼 버스를 두드린다.

        실패한 읽기에도 타임스탬프를 갱신하는 게 중요하다 — 예전에는 성공한
        시각만 남겨서, 통신 오류가 이어지는 동안 매 호출이 '캐시 만료'로 보여
        즉시 재시도하는 뜨거운 루프가 됐다(정상일 때와 같은 간격으로 물러나야
        한다).
        """
        if self.arm_ctrl is None:
            return None
        now = time.monotonic()
        if self._q_cache["q"] is not None and now - self._q_cache["t"] < self.Q_CACHE_TTL_S:
            return self._q_cache["q"]

        servo_degs = [a.get_position() for a in self.arm_ctrl.actuators]
        self._q_cache["t"] = now          # 성공/실패와 무관하게 갱신(위 설명 참고)
        if any(d is None for d in servo_degs):
            return self._q_cache["q"]     # 실패 — 있으면 직전 값을 계속 쓴다
        self._q_cache["q"] = self.arm.servo_deg_to_q(servo_degs)
        return self._q_cache["q"]

    def current_q_or_home(self):
        """`current_q()`가 None이면 전부 0(홈)으로 대체한다.

        인식 루프처럼 "자세를 모르면 일단 기준 자세로 계산"해도 되는 곳에서
        `or [0.0] * n` 관용구를 반복하지 않으려고 만든 짧은 도우미.
        """
        return self.current_q() or [0.0] * len(self.arm.joints)

    def ee_matrix(self, q=None):
        """현재(또는 주어진) 자세의 end-effector 4x4 월드 변환 행렬.

        휴대폰 카메라가 end-effector에 달려 있으므로, 이 행렬이 곧 '카메라가
        어디에서 어디를 보고 있는가'다 — 얼굴/손 좌표를 월드로 올릴 때
        perception 쪽이 이 값을 받는다.
        """
        return self.arm.fk_matrix(self.current_q_or_home() if q is None else q)

    def position(self):
        """현재 end-effector 위치 (x, y, z). 자세를 모르면 None."""
        q = self.current_q()
        return list(self.arm.fk(q)) if q is not None else None

    # ------------------------------------------------------------------
    # 관절 한계 검사 (하드웨어 없이도 동작)
    # ------------------------------------------------------------------
    def joint_slider_range(self, joint):
        """UI 슬라이더용 (최소도, 최대도, 홈각도).

        URDF <limit>에서 나온 값이며, 서보의 물리 범위(0~300도)가 아니라
        자기충돌까지 고려한 범위다(kinematics/find_joint_limits.py). 결합
        관절(현재는 joint2)은 상대 관절이 어디에 있든 항상 안전한 보수적
        경계를 쓰고, 실제 명령 직전의 정밀 검사는
        `servo_degs_within_limits()`가 한다.
        """
        lo_deg, hi_deg = sorted((joint.servo_deg(joint.q_min), joint.servo_deg(joint.q_max)))
        return lo_deg, hi_deg, joint.home_deg

    def servo_degs_within_limits(self, servo_degs):
        """관절 순서대로 받은 서보각들이 자기충돌 안전 범위 안인지 검사한다.

        결합 관절(joint2 ↔ joint3)은 '지금 팔이 어디 있는지'가 아니라 **이번에
        함께 명령될 이 벡터 안의 값**으로 결합 한계를 푼다 — 실제로 동시에
        적용될 값끼리 비교해야 맞기 때문.

        모두 정상이면 None, 아니면 처음 벗어난 관절을 짚는 메시지를 돌려준다.
        """
        q = self.arm.servo_deg_to_q(servo_degs)
        for i, joint in enumerate(self.arm.joints):
            q_other = q[self.arm.id_to_index[joint.coupled_with]] \
                if joint.coupled_with is not None else None
            lo, hi = joint.bounds(q_other=q_other)
            if lo - 1e-9 <= q[i] <= hi + 1e-9:
                continue
            lo_deg, hi_deg = sorted((joint.servo_deg(lo), joint.servo_deg(hi)))
            coupling_note = f" (depends on joint{joint.coupled_with}'s current angle)" \
                if joint.coupled_with is not None else ""
            return (f"joint{joint.id} servo={servo_degs[i]:.1f} deg is outside its "
                    f"self-collision-safe range [{lo_deg:.1f}, {hi_deg:.1f}] deg{coupling_note}")
        return None

    def servo_degs_with_one_changed(self, joint_id, degree):
        """관절 하나만 바꾼 '가상의' 서보각 벡터를 만든다.

        관절 하나를 단독으로 움직이더라도, 결합 관절의 안전 범위는 나머지
        관절들이 지금 어디 있느냐에 따라 달라진다. 그래서 현재 자세(모르면 홈)를
        읽어 이 관절 값만 갈아끼운 벡터를 만들어 위 검사에 넘긴다.
        """
        servo_degs = self.arm.q_to_servo_deg(self.current_q_or_home())
        servo_degs[self.arm.id_to_index[joint_id]] = degree
        return servo_degs

    # ------------------------------------------------------------------
    # 실제 이동 (하드웨어 필요)
    # ------------------------------------------------------------------
    def _require_hardware(self):
        """하드웨어가 없으면 `HardwareUnavailable`을 던진다(이동 메서드 공통 관문)."""
        if self.arm_ctrl is None:
            raise HardwareUnavailable(self.hardware_error or "no hardware connected")

    def _remember_commanded_q(self, q):
        """방금 명령한 자세를 관절각 캐시에 즉시 반영한다.

        [진동 원인 수정] 추종 루프(PerceptionLoop.drive_arm)는 COMMAND_MIN_INTERVAL_S
        (0.15초)마다 "현재 yaw/높이 + 이번 오프셋만큼"처럼 **상대적으로** 다음
        목표를 계산하는데, 그 "현재"를 여태 `current_q()`(하드웨어 읽기 캐시,
        Q_CACHE_TTL_S=3초)에서 가져왔다. 즉 최대 3초 동안, 그 사이 이미 실행된
        수십 번의 명령으로 팔이 실제로 얼마나 움직였는지 모른 채 매번 같은(3초
        전) 기준 각도에 새 보정치를 얹어 명령했다 — 그 기준 각도가 실제와
        어긋난 채로 계속 쌓이면서 팔이 점점 크게 흔들리는("폭주") 원인이었다.

        고칠 방법은 하드웨어를 더 자주 읽는 게 아니라(그러면 원래 이 캐시를
        만든 이유인 시리얼 부하 문제가 되돌아온다), 애초에 우리가 방금 무엇을
        명령했는지는 이미 알고 있으므로 그 값을 캐시에 바로 채워 넣는 것이다 —
        다음 `current_q()` 호출이 하드웨어를 다시 읽지 않고도 "방금 명령한
        자세"를 즉시 돌려주게 된다. 서보가 그 자세에 물리적으로 아직 도달하지
        못했더라도, 상대 보정을 쌓아 가는 추종 루프 입장에서는 "우리가
        의도한 목표"를 기준으로 삼는 편이 실제로 더 맞다(그래야 다음 보정이
        그 위에 정확히 얹힌다).
        """
        self._q_cache["q"] = list(q)
        self._q_cache["t"] = time.monotonic()

    def goto_position(self, target):
        """IK로 (x, y, z)까지 이동. (q, converged)를 돌려준다."""
        self._require_hardware()
        Logger.log("ARM", f"goto_position: target={tuple(target)}")
        q, converged = self.arm_ctrl.goto_position(target)
        if converged:
            self._remember_commanded_q(q)
        return q, converged

    def goto_joints(self, servo_degs):
        """모든 관절을 주어진 서보각으로 보내고 결과 위치(FK)를 돌려준다."""
        self._require_hardware()
        Logger.log("ARM", f"goto_joints: degrees={servo_degs}")
        pos = self.arm_ctrl.goto_joints(servo_degs)
        self._remember_commanded_q(self.arm.servo_deg_to_q(servo_degs))
        return pos

    def goto_joint(self, joint_id, degree):
        """관절 하나만 움직인다. 해당 id의 액추에이터가 없으면 KeyError."""
        self._require_hardware()
        actuator = next((a for a in self.arm_ctrl.actuators if a.id == joint_id), None)
        if actuator is None or joint_id not in self.arm.id_to_index:
            raise KeyError(joint_id)
        Logger.log("ARM", f"goto_joint: id={joint_id} degree={degree}")
        actuator.goto(degree)
        servo_degs = self.arm.q_to_servo_deg(self.current_q_or_home())
        servo_degs[self.arm.id_to_index[joint_id]] = degree
        self._remember_commanded_q(self.arm.servo_deg_to_q(servo_degs))

    def execute(self, command):
        """`FollowController.next_command()`가 돌려준 (kind, payload)를 실행한다.

        kind="position" → goto_position(IK),  kind="joints" → goto_joints.
        추종 로직(perception)은 하드웨어를 모르고 "무엇을 할지"만 정하므로,
        그 결정을 실제 명령으로 옮기는 번역이 여기 한 곳에 있다.
        """
        if command is None:
            return
        kind, payload = command
        if kind == "position":
            self.goto_position(payload)
        elif kind == "joints":
            self.goto_joints(payload)
        else:
            Logger.log("ARM", f"unknown command kind: {kind!r}")

    def invalidate_cache(self):
        """관절각 캐시를 강제로 버린다(다음 조회에서 실제 서보를 다시 읽는다).

        이동 메서드들은 **일부러 이 함수를 부르지 않는다.** 캐시 미스 한 번이
        시리얼 왕복 5회이고 추종 루프는 최대 0.15초마다 새 명령을 내리므로,
        움직일 때마다 캐시를 버리면 사실상 캐시가 없는 것과 같아져 이 루프가
        시리얼 대기에 묶인다(Q_CACHE_TTL_S가 몇 초로 넉넉한 것도 같은 이유).
        추종·3D 오버레이는 몇 초 묵은 자세로도 충분하다는 판단이 먼저 있고,
        그 판단을 지키려고 갱신을 TTL에만 맡긴 것이다. 정확한 현재 자세가
        꼭 필요한 특수한 경우를 위해 수동 스위치로만 남겨 둔다.
        """
        self._q_cache["q"] = None
        self._q_cache["t"] = 0.0
