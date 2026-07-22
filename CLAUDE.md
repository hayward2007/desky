# desky

책상 위 5자유도 로봇팔로, end-effector에 **휴대폰을 장착**하는 프로젝트. DYNAMIXEL
AX-18A 서보 5개를 시리얼로 제어하며, URDF 기반 순기구학(FK)/역기구학(IK)을 제공한다.

## 하드웨어 / 관절 구성

- 액추에이터: DYNAMIXEL **AX-18A** ×5 (Protocol 1.0), U2D2 등으로 시리얼 연결
- 관절 축 (base → tip):
  | id | 관절 | 축 | 역할 |
  |----|------|----|------|
  | 1 | yaw   | Z | 베이스 회전 (방위) |
  | 2 | roll  | X | 팔 평면 롤 |
  | 3 | pitch | Y | 어깨 |
  | 4 | pitch | Y | 팔꿈치 |
  | 5 | pitch | Y | 손목 |
- 5-DOF이므로 위치(3) + 부분 자세만 도달 가능. 임의의 6-DOF pose는 불가.
- AX-18A 물리 범위: **0~300°** ↔ 유닛값 0~1023 (`Unit_Number = 1023`).

## 실행 전 설정

`.env` 파일 필요 (README.md 참고):
```env
DEVICE_NAME=tty01        # /dev/ 접두어 제외
BAUDRATE=1000000
PROTOCOL_VERSION=1.0
```
의존성: `dynamixel_sdk`, `python-dotenv` (하드웨어 제어), `matplotlib` (`simulation/` 3D
프리뷰 전용), `flask` (`webapp/` 제어 대시보드 전용). 기구학 모듈 자체는 **표준 라이브러리만**
사용(numpy 불필요). Python 3.14 확인됨.

Homebrew Python은 PEP 668로 시스템 전역 `pip install`을 막으므로, `matplotlib`/`flask` 설치는
프로젝트 venv를 권장:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install matplotlib flask
```

## 파일 구조

프로젝트는 하드웨어 제어(`hardware/`)와 기구학(`kinematics/`)을 별도 패키지로 분리한다.
둘은 서로 독립적이며, `logger.py`만 양쪽에서 공통으로 참조하는 루트 모듈이다.

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

`hardware/`, `kinematics/`, `simulation/`, `webapp/` 모두 패키지(`__init__.py` 포함)이며,
`hardware`/`kinematics` 내부 모듈 간 임포트는 상대 임포트(`.control_table`, `.kinematics`)를
쓴다. `logger.py`/`scenario.py`는 루트에 있으므로 어디서든 `from logger import Logger`,
`from scenario import DEMO_SEQUENCE`로 절대 임포트한다. 항상 **저장소 루트에서** 실행할 것
(`python3 main.py`, `python3 -m kinematics.kinematics`, `python3 -m simulation.simulate`,
`python3 -m webapp.app` 등) — 하위 폴더의 파일을 직접 실행하면(`python3 kinematics/kinematics.py`)
루트가 `sys.path`에 없어 `logger`/`scenario` 임포트가 깨진다.

### 계층
```
main.py
  └─ hardware.util.ArmController ── hardware.util.Actuator ×5 ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
                  └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
```
`hardware.util.ArmController`가 FK/IK(`kinematics.Arm`)와 실제 `Actuator` 5개를 연결한다.
`Actuator.id`(DYNAMIXEL id)와 URDF `<dynamixel id="">`가 일치하는 조인트를 서로 매칭하므로,
생성자에 넘기는 `actuators` 리스트는 순서에 상관없다.

```python
from hardware.controller import Controller
from hardware.util import Actuator, ArmController

controller = Controller()
actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
arm_ctrl = ArmController(actuators)             # arm=load_arm() 기본값 사용

q, ok = arm_ctrl.goto_position((0.3, 0.05, 0.15))   # IK 계산 → 5개 서보에 goto() 디스패치
pos = arm_ctrl.get_position()                        # 5개 서보 현재각 read → FK로 위치 계산
```

- `goto_position`은 IK가 수렴하지 않으면(`converged=False`) 서보를 움직이지 않는다.
- `get_position`은 액추에이터 중 하나라도 `None`을 반환하면(통신 실패) 전체 결과도 `None`.

## 웹 제어 대시보드 (실제 하드웨어 필요)

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install flask   # 최초 1회
python3 -m webapp.app   # 저장소 루트에서 실행, http://localhost:5000
```

`webapp/app.py`는 `main.py`와 동일하게 시작 시 `Controller()`로 시리얼 포트를 열고 액추에이터
5개 + `ArmController`를 구성한다(즉 `.env` 설정과 실제 하드웨어 필요). 브라우저 대시보드에서:

- 목표 위치 (x, y, z, 미터) 입력 → `ArmController.goto_position()` 호출 (IK → 5개 서보 이동)
- 관절별 서보각(0~300°) 직접 입력 → 해당 `Actuator.goto()` 직접 호출 (IK 우회)
- 1.5초마다 `ArmController.get_position()`으로 현재 위치를 폴링해 표시

라우트: `GET /`(대시보드), `GET /api/status`, `POST /api/goto_position`,
`POST /api/goto_joint`. `app.run(..., debug=False)` 고정 — Flask 리로더가 모듈을 다시
임포트하면 시리얼 포트가 두 번 열리므로 `debug=True`로 바꾸지 말 것.

## 3D 시뮬레이션 (하드웨어 없이 미리보기)

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install matplotlib   # 최초 1회
python3 -m simulation.simulate   # 저장소 루트에서 실행
```

`simulation/simulate.py`는 `kinematics.urdf_loader.load_arm()`으로 로봇 형상
(`kinematics/desky.urdf`)을, `scenario.DEMO_SEQUENCE`로 모션 웨이포인트를 읽어 3D로 애니메이션
재생한다. `main.py`가 실제 하드웨어에 보내는 것과 **같은 시퀀스**를 그대로 재생하므로(둘 다
`scenario.py` import), `scenario.py`만 수정하면 두 스크립트가 함께 바뀐다.
`kinematics.Arm.fk_all(q)`가 base → 각 관절 → tool tip 위치 목록을 반환해 막대 인형(stick
figure) 형태로 그린다.

주의: `simulation/simulate.py`는 서보 웨이포인트를 URDF의 `<limit>`(IK용 소프트 리미트)로
클램프하지 않는다 — 실제 `Actuator.goto()`도 물리 범위(0~300°) 안이면 그 리미트를 무시하고
그대로 움직이므로, 시뮬레이션도 동일하게 동작해야 "실제 로봇과 정확히 같은" 모션이 된다.

## 기구학 사용법

```python
from kinematics.urdf_loader import load_arm
arm = load_arm()                        # kinematics/desky.urdf에서 구성 로드 (권장)
# 또는: from kinematics.kinematics import Arm; arm = Arm()   # 하드코딩 fallback

pos = arm.fk([0,0,0,0,0])               # 관절각(rad) → end-effector 위치(x,y,z)
q, ok = arm.ik(target_pos)              # 위치 IK (damped least-squares)
q, ok = arm.ik(target_pos, target_rot=R3x3)   # 자세 포함(best-effort)
servo_deg = arm.q_to_servo_deg(q)       # 관절각(rad) → 서보각(0~300°)
```
- IK는 수렴 여부 `bool`을 반환하며, 도달 불가 목표도 예외 없이 `False` 반환.
- 관절각 `q`(rad, home 기준) ↔ 서보각(deg) 변환: `servo_deg = home_deg + direction * deg(q)`.

## 로봇 구성 수정 방법 (중요)

코드가 아니라 **[kinematics/desky.urdf](kinematics/desky.urdf) 하나만 수정**한다.
`urdf_loader`가 이를 읽어 `Arm`을 만든다.
- 링크 길이 → 각 `<joint>`의 `<origin xyz>` (URDF 관례상 **미터** 단위)
- 관절 한계 → `<limit lower upper>` (라디안)
- 축 → `<axis xyz>`
- DYNAMIXEL id/모델/서보 캘리브레이션 → 프로젝트 확장 태그
  `<dynamixel id="" model="" home_deg="" direction="">` (표준 URDF 툴은 무시, 로더만 읽음)

## 미교정(placeholder) 값 — 실측 필요

`kinematics/desky.urdf`의 링크 길이와 `home_deg`/`direction`/`<limit>`은 **가짜 기본값**이다.
실제 팔 치수·영점을 측정해 채워야 FK/IK 좌표가 물리 좌표와 일치한다.
`kinematics/kinematics.py` 상단의 `*_MM` 상수는 URDF 없이 쓰는 fallback용(mm 단위).
→ 실 사용 시 **미터로 통일** 권장.

## 주의사항 / 알려진 특성

- `hardware/controller.py`의 read/write는 통신·하드웨어 에러 시 **예외로 죽지 않고** 경고
  출력 후 계속 진행. `set_*`는 성공 여부 `bool`, `get_present_position`은 실패 시 `None`
  반환 → **호출부에서 `None` 처리 필요**.
- `Controller`는 `with Controller() as c:` 컨텍스트 매니저 지원 (GC 대신 결정적 포트 정리).
- 각도↔유닛 변환은 `/360` 계수를 쓴다(범위 리미트로 의도적 처리됨 — 유지할 것).
- XML 주석 안에 `--`(이중 하이픈) 금지 (URDF 편집 시 파싱 에러 주의).
- 모든 모듈의 콘솔 출력은 `logger.Logger.log(tag, message)`를 거쳐 `[TAG] message` 형식으로
  통일된다 (`CONTROLLER`, `ACTUATOR`, `MAIN`, `KINEMATICS`, `URDF` 태그). `Logger.enabled`
  플래그 하나로 전체 로그를 끌 수 있다.

## 검증 방법 (하드웨어 없이)

저장소 루트에서 실행:

```bash
python3 -m kinematics.kinematics     # FK/IK 라운드트립 self-test
python3 -m kinematics.urdf_loader    # URDF 로드 + FK/IK 라운드트립
```
기대: IK `converged=True`, 위치 오차 < 1mm.
