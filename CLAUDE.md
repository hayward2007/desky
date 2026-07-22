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
의존성: `dynamixel_sdk`, `python-dotenv`. 기구학 모듈은 **표준 라이브러리만** 사용
(numpy 불필요). Python 3.14 확인됨.

## 파일 구조

프로젝트는 하드웨어 제어(`hardware/`)와 기구학(`kinematics/`)을 별도 패키지로 분리한다.
둘은 서로 독립적이며, `logger.py`만 양쪽에서 공통으로 참조하는 루트 모듈이다.

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

`hardware/`, `kinematics/` 모두 패키지(`__init__.py` 포함)이며, 내부 모듈 간 임포트는
상대 임포트(`.control_table`, `.kinematics`)를 쓴다. `logger.py`는 루트에 있으므로
어디서든 `from logger import Logger`로 절대 임포트한다. 항상 **저장소 루트에서** 실행할 것
(`python3 main.py`, `python3 -m kinematics.kinematics` 등) — 하위 폴더의 파일을 직접
실행하면(`python3 kinematics/kinematics.py`) 루트가 `sys.path`에 없어 `logger` 임포트가
깨진다.

### 계층
```
main.py
  └─ hardware.util.Actuator ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()   # 하드웨어와 독립
```
현재 `main.py`(하드웨어 제어)와 기구학(`kinematics`/`urdf_loader`)은 **아직 연결되지
않음**. IK로 구한 서보각을 `Actuator.goto()`로 보내는 연동은 미구현.

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
