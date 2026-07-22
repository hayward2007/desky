# desky

책상 위에 놓는 5자유도(5-DOF) 로봇팔 프로젝트. End-effector에 **휴대폰을 장착**하는 것이 목표이며,
DYNAMIXEL AX-18A 서보 5개를 시리얼로 제어하고 URDF 기반 순기구학(FK)/역기구학(IK)을 제공한다.

## 하드웨어 / 관절 구성

- 액추에이터: DYNAMIXEL **AX-18A** × 5 (Protocol 1.0), U2D2 등으로 시리얼 연결
- 관절 축 구성 (base → tip):

  | id | 관절 | 축 | 역할 |
  | -- | ---- | -- | ---- |
  | 1 | yaw | Z | 베이스 회전 (방위) |
  | 2 | roll | X | 팔 평면 롤 |
  | 3 | pitch | Y | 어깨 |
  | 4 | pitch | Y | 팔꿈치 |
  | 5 | pitch | Y | 손목 |

- 5-DOF이므로 위치(x, y, z) + 부분적인 자세만 도달 가능하며, 임의의 6-DOF pose는 만족할 수 없다.
- AX-18A 물리 각도 범위: **0~300°**, 내부 유닛값 범위 0~1023.

## 시작하기

### 요구 사항

- Python 3.14 확인됨 (표준 라이브러리 위주라 다른 3.x에서도 대부분 동작할 것)
- 의존성: `dynamixel_sdk`, `python-dotenv` (하드웨어 제어 시에만 필요), `matplotlib`
  ([simulation/simulate.py](simulation/simulate.py) 3D 프리뷰 전용), `flask`
  ([webapp/app.py](webapp/app.py) 제어 대시보드 전용)
- 기구학 패키지(`kinematics/`)는 **표준 라이브러리만** 사용 — numpy 등 불필요

```bash
pip install dynamixel_sdk python-dotenv
```

Homebrew Python처럼 시스템 전역 `pip install`이 막혀 있다면(PEP 668), 프로젝트 venv를
권장한다 (특히 `matplotlib`/`flask`를 쓰는 [simulation/simulate.py](simulation/simulate.py),
[webapp/app.py](webapp/app.py)용):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install matplotlib flask
```

### 환경 변수

실제 서보를 제어하려면 프로젝트 루트에 `.env` 파일이 필요하다.

```env
DEVICE_NAME=tty01        # /dev/ 접두어는 제외하고 작성
BAUDRATE=1000000
PROTOCOL_VERSION=1.0
```

## 파일 구조

하드웨어 제어(`hardware/`), 기구학(`kinematics/`), 3D 시뮬레이션(`simulation/`), 웹 제어
대시보드(`webapp/`)를 별도 패키지로 분리했다. 서로 독립적이며, `logger.py`/`scenario.py`만
공통으로 참조하는 루트 모듈이다.

```
desky/
├── main.py            # 데모 시나리오 진입점 (실제 하드웨어)
├── scenario.py         # main.py/simulation이 공유하는 모션 시퀀스 (단일 소스)
├── logger.py           # 공통 [TAG] message 콘솔 로거
├── hardware/           # 시리얼/서보 제어 (실제 하드웨어 필요)
│   ├── controller.py
│   ├── control_table.py
│   └── util.py
├── kinematics/         # FK/IK (하드웨어 불필요, 표준 라이브러리만 사용)
│   ├── kinematics.py
│   ├── urdf_loader.py
│   └── desky.urdf
├── simulation/         # 3D 프리뷰 (하드웨어 불필요, matplotlib 필요)
│   └── simulate.py
└── webapp/             # Flask 제어 대시보드 (실제 하드웨어 필요)
    ├── app.py
    └── templates/
        └── index.html
```

| 파일 | 역할 |
|------|------|
| [main.py](main.py) | 데모 시나리오 (액추에이터 5개 생성 → `scenario.DEMO_SEQUENCE` 재생). 실제 하드웨어 필요 |
| [scenario.py](scenario.py) | `DEMO_SEQUENCE` — 서보각 웨이포인트 시퀀스. `main.py`/`simulation.simulate`가 동일하게 참조하는 단일 소스 |
| [logger.py](logger.py) | `Logger` — 모든 모듈이 공유하는 `[TAG] message` 콘솔 로거 |
| [hardware/controller.py](hardware/controller.py) | 시리얼 포트/보드레이트 초기화, 저수준 read/write (`set_speed`, `set_goal_position`, `get_present_position`) |
| [hardware/control_table.py](hardware/control_table.py) | AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기)을 클래스로 정의 |
| [hardware/util.py](hardware/util.py) | `Actuator`(모터 1개 추상화) + `ArmController`(FK/IK ↔ Actuator 연동) |
| [kinematics/kinematics.py](kinematics/kinematics.py) | FK/IK. `Arm`/`Joint` + 순수 파이썬 선형대수. 하드웨어 불필요 |
| [kinematics/urdf_loader.py](kinematics/urdf_loader.py) | `desky.urdf`를 파싱해 `Arm` 생성 (`xml.etree`) |
| [kinematics/desky.urdf](kinematics/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |
| [simulation/simulate.py](simulation/simulate.py) | `scenario.DEMO_SEQUENCE`를 3D로 애니메이션 재생 (matplotlib) |
| [webapp/app.py](webapp/app.py) | Flask 앱 — 브라우저에서 위치/관절각 입력 → `ArmController` 호출. 실제 하드웨어 필요 |
| [webapp/templates/index.html](webapp/templates/index.html) | 대시보드 UI (위치 입력, 관절별 입력, 상태 표시) |

> **항상 저장소 루트에서 실행할 것** (`python3 main.py`, `python3 -m kinematics.kinematics`,
> `python3 -m simulation.simulate`, `python3 -m webapp.app` 등). 하위 폴더의 파일을 직접
> 실행하면(`python3 kinematics/kinematics.py`) 루트가 `sys.path`에 없어 `logger`/`scenario`
> 임포트가 깨진다.

### 계층 구조

```
main.py
  └─ hardware.util.ArmController ── hardware.util.Actuator ×5 ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
                  └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
```

`ArmController`가 FK/IK(`kinematics.Arm`)와 실제 `Actuator` 5개를 연결한다. `Actuator.id`
(DYNAMIXEL id)와 URDF `<dynamixel id="">`가 일치하는 조인트끼리 매칭하므로, 생성자에 넘기는
`actuators` 리스트는 순서에 상관없다.

```python
from hardware.controller import Controller
from hardware.util import Actuator, ArmController

controller = Controller()
actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
arm_ctrl = ArmController(actuators)                  # arm=load_arm() 기본값 사용

q, ok = arm_ctrl.goto_position((0.3, 0.05, 0.15))    # IK 계산 → 5개 서보에 goto() 디스패치
pos = arm_ctrl.get_position()                         # 5개 서보 현재각 read → FK로 위치 계산
```

- `goto_position`은 IK가 수렴하지 않으면(`converged=False`) 서보를 움직이지 않는다.
- `get_position`은 액추에이터 중 하나라도 `None`을 반환하면(통신 실패) 전체 결과도 `None`.

## 웹 제어 대시보드 (실제 하드웨어 필요)

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install flask   # 최초 1회
python3 -m webapp.app   # 저장소 루트에서 실행, http://localhost:5000
```

[webapp/app.py](webapp/app.py)는 [main.py](main.py)와 동일하게 시작 시 `Controller()`로
시리얼 포트를 열고 액추에이터 5개 + `ArmController`를 구성한다(즉 `.env` 설정과 실제 하드웨어
필요). 브라우저 대시보드에서:

- 목표 위치 (x, y, z, 미터) 입력 → `ArmController.goto_position()` 호출 (IK → 5개 서보 이동)
- 관절별 서보각(0~300°) 직접 입력 → 해당 `Actuator.goto()` 직접 호출 (IK 우회)
- 1.5초마다 `ArmController.get_position()`으로 현재 위치를 폴링해 표시

라우트: `GET /`(대시보드), `GET /api/status`, `POST /api/goto_position`,
`POST /api/goto_joint`.

> **참고:** `app.run(..., debug=False)`로 고정되어 있다 — Flask 리로더가 모듈을 다시
> 임포트하면 시리얼 포트가 두 번 열리므로 `debug=True`로 바꾸지 말 것.

## 3D 시뮬레이션 (하드웨어 없이 미리보기)

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install matplotlib   # 최초 1회
python3 -m simulation.simulate   # 저장소 루트에서 실행
```

[simulation/simulate.py](simulation/simulate.py)는 `kinematics.urdf_loader.load_arm()`으로
로봇 형상([kinematics/desky.urdf](kinematics/desky.urdf))을, [scenario.py](scenario.py)의
`DEMO_SEQUENCE`로 모션 웨이포인트를 읽어 3D로 애니메이션 재생한다. [main.py](main.py)가
실제 하드웨어에 보내는 것과 **완전히 동일한 시퀀스**를 재생하므로(둘 다 `scenario.py`를
import), `scenario.py`만 수정하면 두 스크립트가 함께 바뀐다.

> **참고:** `simulation/simulate.py`는 서보 웨이포인트를 URDF의 `<limit>`(IK용 소프트
> 리미트)로 클램프하지 않는다 — 실제 `Actuator.goto()`도 물리 범위(0~300°) 안이면 그 리미트를
> 무시하고 그대로 움직이므로, 시뮬레이션도 동일하게 동작해야 "실제 로봇과 정확히 같은" 모션이
> 된다.

## 기구학 사용법

```python
from kinematics.urdf_loader import load_arm
arm = load_arm()                        # kinematics/desky.urdf에서 구성 로드 (권장)
# 또는: from kinematics.kinematics import Arm; arm = Arm()   # 하드코딩 fallback

pos = arm.fk([0, 0, 0, 0, 0])            # 관절각(rad) → end-effector 위치(x, y, z)
q, ok = arm.ik(target_pos)               # 위치 IK (damped least-squares)
q, ok = arm.ik(target_pos, target_rot=R3x3)  # 자세 포함(best-effort)
servo_deg = arm.q_to_servo_deg(q)         # 관절각(rad) → 서보각(0~300°)
```

- IK는 수렴 여부를 `bool`로 반환하며, 도달 불가능한 목표에 대해서도 예외 없이 `False`를 반환한다.
- 관절각 `q`(rad, home 기준) ↔ 서보각(deg) 변환: `servo_deg = home_deg + direction * degrees(q)`.

### 하드웨어 없이 검증하기

저장소 루트에서 실행:

```bash
python3 -m kinematics.kinematics     # FK/IK 라운드트립 self-test
python3 -m kinematics.urdf_loader    # URDF 로드 + FK/IK 라운드트립
```

기대 결과: `converged=True`, 위치 오차 < 1mm.

## 로봇 구성 수정 방법

로봇 치수·서보 캘리브레이션을 바꿀 때는 코드가 아니라 **[kinematics/desky.urdf](kinematics/desky.urdf)
하나만 수정**한다. `urdf_loader`가 이를 읽어 `Arm`을 구성한다.

- 링크 길이 → 각 `<joint>`의 `<origin xyz>` (URDF 관례상 **미터** 단위)
- 관절 한계 → `<limit lower upper>` (라디안)
- 축 → `<axis xyz>`
- DYNAMIXEL id/모델/서보 캘리브레이션 → 프로젝트 확장 태그
  `<dynamixel id="" model="" home_deg="" direction="">` (표준 URDF 툴은 무시하고, `urdf_loader`만 읽음)

### ⚠️ 미교정(placeholder) 값 — 실측 필요

`kinematics/desky.urdf`의 링크 길이(`origin xyz`)와 `home_deg`/`direction`/`<limit>`은 아직
**가짜 기본값**이다. 실제 팔의 치수와 서보 영점을 측정해 채워야 FK/IK가 계산하는 좌표가 실제
물리 좌표와 일치한다. `kinematics/kinematics.py` 상단의 `*_MM` 상수는 URDF 없이 쓸 때의
fallback용(mm 단위)이며, 실사용 시에는 URDF 쪽 미터 단위로 통일하는 것을 권장한다.

## 주의사항 / 알려진 특성

- `hardware/controller.py`의 read/write는 통신·하드웨어 에러가 나도 **예외로 죽지 않고**
  경고를 출력한 뒤 계속 진행한다. `set_*` 계열은 성공 여부를 `bool`로, `get_present_position`은
  실패 시 `None`을 반환하므로 **호출부에서 반드시 `None`을 처리**해야 한다.
- `Controller`는 `with Controller() as c:` 형태의 컨텍스트 매니저를 지원한다 (GC에 맡기지
  않고 포트를 결정적으로 정리).
- 각도 ↔ 유닛 변환은 `/360` 계수를 사용한다 (물리 범위 0~300°는 리미트로 의도적으로 처리됨 —
  변경하지 말 것).
- URDF를 직접 편집할 때 XML 주석 안에 `--`(이중 하이픈)를 넣으면 파싱 에러가 나므로 주의.
- 모든 모듈의 콘솔 출력은 `logger.Logger.log(tag, message)`를 거쳐 `[TAG] message` 형식으로
  통일된다 (`CONTROLLER`, `ACTUATOR`, `MAIN`, `KINEMATICS`, `URDF` 태그). `Logger.enabled`
  플래그 하나로 전체 로그를 끌 수 있다.
