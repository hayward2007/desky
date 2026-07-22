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
- 의존성: `dynamixel_sdk`, `python-dotenv` (하드웨어 제어 시에만 필요)
- 기구학 패키지(`kinematics/`)는 **표준 라이브러리만** 사용 — numpy 등 불필요

```bash
pip install dynamixel_sdk python-dotenv
```

### 환경 변수

실제 서보를 제어하려면 프로젝트 루트에 `.env` 파일이 필요하다.

```env
DEVICE_NAME=tty01        # /dev/ 접두어는 제외하고 작성
BAUDRATE=1000000
PROTOCOL_VERSION=1.0
```

## 파일 구조

하드웨어 제어(`hardware/`)와 기구학(`kinematics/`)을 별도 패키지로 분리했다. 둘은 서로
독립적이며, `logger.py`만 양쪽에서 공통으로 참조하는 루트 모듈이다.

```
desky/
├── main.py            # 데모 시나리오 진입점
├── logger.py           # 공통 [TAG] message 콘솔 로거
├── hardware/           # 시리얼/서보 제어 (실제 하드웨어 필요)
│   ├── controller.py
│   ├── control_table.py
│   └── util.py
└── kinematics/         # FK/IK (하드웨어 불필요, 표준 라이브러리만 사용)
    ├── kinematics.py
    ├── urdf_loader.py
    └── desky.urdf
```

| 파일 | 역할 |
|------|------|
| [main.py](main.py) | 데모 시나리오 (액추에이터 5개 생성 → 자세 명령). 실제 하드웨어 필요 |
| [logger.py](logger.py) | `Logger` — 모든 모듈이 공유하는 `[TAG] message` 콘솔 로거 |
| [hardware/controller.py](hardware/controller.py) | 시리얼 포트/보드레이트 초기화, 저수준 read/write (`set_speed`, `set_goal_position`, `get_present_position`) |
| [hardware/control_table.py](hardware/control_table.py) | AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기)을 클래스로 정의 |
| [hardware/util.py](hardware/util.py) | `Actuator` 클래스 — 모터 1개 추상화 (`goto`, `get_position`) |
| [kinematics/kinematics.py](kinematics/kinematics.py) | FK/IK. `Arm`/`Joint` + 순수 파이썬 선형대수. 하드웨어 불필요 |
| [kinematics/urdf_loader.py](kinematics/urdf_loader.py) | `desky.urdf`를 파싱해 `Arm` 생성 (`xml.etree`) |
| [kinematics/desky.urdf](kinematics/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |

> **항상 저장소 루트에서 실행할 것** (`python3 main.py`, `python3 -m kinematics.kinematics`
> 등). 하위 폴더의 파일을 직접 실행하면(`python3 kinematics/kinematics.py`) 루트가
> `sys.path`에 없어 `logger` 임포트가 깨진다.

### 계층 구조

```
main.py
  └─ hardware.util.Actuator ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()   # 하드웨어와 독립적으로 동작
```

> **참고:** 현재 `main.py`(하드웨어 제어)와 기구학(`kinematics`/`urdf_loader`)은 아직 연결되지
> 않았다. IK로 구한 서보각을 `Actuator.goto()`로 보내는 연동은 미구현 상태.

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
