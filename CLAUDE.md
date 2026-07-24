# desky

책상 위 5자유도 로봇팔로, end-effector에 **휴대폰을 장착**하는 프로젝트. DYNAMIXEL
AX-18A 서보 5개를 시리얼로 제어하며, URDF 기반 순기구학(FK)/역기구학(IK)을 제공한다.
휴대폰 카메라를 통해 얼굴/손/몸(어깨) 인식과 문서 스캔/음성 대화(Gemini)도 지원한다 —
셋 다 휴대폰 자체(브라우저 MediaPipe Tasks Vision)가 전담하고, 서버는 좌표 변환/추종
계산만 한다. 몸(어깨) 인식은 얼굴 인식이 실패했을 때(옆모습 등)의 폴백이다(왜 그런지는
아래 "얼굴/몸/손 인식" 절 참고) — 서버는 셋 중 어느 것도 모델 추론을 하지 않으므로
mediapipe(Python)는 이 프로젝트의 의존성이 아니다. 손 인식/가위바위보 제스처는 켜져
있지만, **팔이 손 위치로 움직이는 것(추종)만** 기본적으로 꺼져 있다
(`fundamental.const.FollowControllerConst.HAND_FOLLOW_ENABLED`).

**이 저장소는 두 브랜치를 합친 통합본이다** — 팔 추적 계열(`desky-develop`: 얼굴/손 추적,
추종 컨트롤러, 두리번거리기)과 웹 기능 계열(`desky-new_mobile_1017ver`: 일정, 조명,
가위바위보 제스처, 라즈베리파이 서버). 무엇이 어디서 왔고 어떤 충돌을 어떻게 풀었는지는
[docs/MERGE.md](docs/MERGE.md), 함수 단위 설명은 [docs/FUNCTIONS.md](docs/FUNCTIONS.md).
**코드를 고치기 전에 이 두 문서를 먼저 읽을 것** — 특히 손 인식 경로(폰이 인식 → 서버가
추종/제스처 양쪽에 재사용)를 모르면 서버에서 mediapipe hands를 다시 돌리는 실수를 하기 쉽다.

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
GEMINI_API_KEY=...       # 채팅/요약/STT/문서 읽기용 (없으면 해당 기능만 비활성)
```
의존성은 [requirements.txt](requirements.txt)에 정리돼 있다: `dynamixel_sdk`,
`python-dotenv`(하드웨어 제어), `matplotlib`(`kinematics/simulate.py` 3D 프리뷰),
`flask`/`flask-sock`/`pyopenssl`(`src/app.py` 제어 대시보드 + 모바일 카메라 스트리밍),
`google-genai`(`/api/ask`, 문서 읽기), `opencv-python`/`numpy`(문서 검출, 카메라 미리보기
창). 얼굴/손/몸(어깨) 인식 자체는 전부 휴대폰 브라우저가 MediaPipe Tasks Vision(JS)으로
하므로 서버 쪽엔 `mediapipe`(Python) 의존성이 없다. 기구학 모듈 자체는 **표준
라이브러리만** 사용(numpy 불필요).

Homebrew Python은 PEP 668로 시스템 전역 `pip install`을 막으므로, 이 선택적 의존성 설치는
프로젝트 venv를 권장:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 파일 구조

프로젝트는 하드웨어 제어(`hardware/`), 기구학(`kinematics/`), AI 인지 기능(`perception/`),
웹 앱(`src/`)을 별도 패키지로 분리한다. 모두 서로 독립적이며, `fundamental/`(로거 + 상수)만
전체가 공통으로 참조하는 루트 패키지다.

```text
desky/
├── main.py             # 진입점 — src.app.run() 실행. 하드웨어 유무 무관
├── fundamental/         # 전체 공통 루트 패키지
│   ├── logger.py           # 공통 [TAG] message 콘솔 로거
│   └── const.py            # 프로젝트 전역 상수(원래 각 모듈에 흩어져 있던 것을 한곳에 모음)
├── requirements.txt    # 전체 의존성 (기능별 주석 포함)
├── hardware/           # 시리얼/서보 제어 (실제 하드웨어 필요)
│   ├── controller.py       # Controller — 시리얼 포트/보드레이트, 저수준 read/write
│   └── actuator.py         # Actuator(모터 1개) + ArmController(FK/IK ↔ Actuator 연동)
├── kinematics/          # FK/IK + 3D 프리뷰 (하드웨어 불필요)
│   ├── kinematics.py       # Arm/Joint — 순수 파이썬 선형대수 (표준 라이브러리만)
│   ├── urdf_loader.py      # configure/desky.urdf를 파싱해 Arm 생성 (xml.etree)
│   ├── simulate.py         # 인터랙티브 FK 슬라이더 + IK 입력 (matplotlib 필요)
│   ├── mujoco_sim.py        # MuJoCo 기반 메시 시뮬레이션 (선택, mujoco 필요)
│   ├── find_joint_limits.py # MuJoCo로 자기충돌 없는 관절 한계를 스윕해서 찾는 스크립트
│   ├── configure/
│   │   ├── desky.urdf         # **로봇 구성의 단일 소스 오브 트루스** (URDF, XML)
│   │   ├── joints.json        # find_joint_limits.py가 계산한 관절 한계
│   │   └── manifest.json      # Fusion 360 내보내기 메시 ↔ URDF 링크 매핑
│   └── meshes/*.stl        # Fusion 360에서 내보낸 실제 부품 메시
├── perception/          # AI 인지 기능 (하드웨어 불필요, 카메라 프레임/랜드마크 입력)
│   ├── camera_geometry.py  # FaceTracker/HandTracker/BodyTracker가 공유하는 핀홀 카메라 지오메트리 + clamp_xy
│   ├── face_tracker.py     # FaceTracker(좌표 변환만, 인식은 휴대폰이 함) + FaceFollower
│   ├── body_tracker.py     # BodyTracker(좌표 변환만, 인식은 휴대폰이 함 — 얼굴 폴백) + BodyFollower
│   ├── hand_tracker.py     # HandTracker(좌표 변환만, 인식은 휴대폰이 함) + HandFollower
│   ├── follow_controller.py # FollowController — 얼굴>몸>손>idle 우선순위 + 두리번 상태 머신
│   ├── gesture.py          # 가위/바위/보 판정 + 엣지 트리거 디스패처 (병합: 웹 계열)
│   └── document_scanner.py # DocumentScanner — 문서 사각형 검출/원근 보정 (OCR 없음)
├── src/                 # Flask 대시보드 + 모바일 카메라/음성/문서/일정/조명 (하드웨어 없어도 실행됨)
│   ├── app.py              # DeskyApp — 조립만(서비스 생성 + 라우트 등록 + 실행)
│   ├── arm_service.py      # ArmService — 팔 상태·하드웨어·관절각 캐시·안전 검사
│   ├── render.py           # ArmRenderer(자세→PNG) + ScenePreview(로컬 3D 창)
│   ├── perception_loop.py  # PerceptionLoop — 카메라→인식→추종→제스처→미리보기
│   ├── gesture_bridge.py   # GestureBridge — 제스처 → 폰 명령 + 조명
│   ├── api/
│   │   ├── arm.py          # ArmAPI — 팔 제어 라우트 7개
│   │   ├── gemini.py       # Gemini — 채팅/요약/STT/문서 읽기
│   │   ├── camera.py       # Camera — /ws/camera 양방향(프레임·음성·랜드마크 ↔ 명령)
│   │   ├── scan.py         # ScanAPI — 문서 스캔 3라우트 (DocumentScanner + Gemini)
│   │   ├── calendar.py     # Calendar — 일정 저장소 (병합: 웹 계열)
│   │   └── light.py        # Light — 조명 프록시 (병합: 웹 계열)
│   ├── static/theme.css # 공통 디자인 시스템(색 토큰·글꼴·칩/버튼) — 모든 페이지가 링크
│   └── templates/
│       ├── index.html
│       ├── mobile.html  # /mobile — 좌: 듣는 면 / 우: 카메라
│       └── calendar.html # /calendar — 한 달 달력 전용 화면
├── raspberry-server/    # 파이에서 따로 실행하는 서보(조명) 서버 — 이 앱과 HTTP로만 통신
└── docs/
    ├── FUNCTIONS.md     # 모듈·함수 단위 설명서
    └── MERGE.md         # 병합 내역과 충돌 해결 기록
```

| 파일 | 역할 |
|------|------|
| [main.py](main.py) | 진입점 — `src.app.run()` 호출(`python main.py` → `https://localhost:8000`). 하드웨어 없어도 실행됨 |
| [fundamental/logger.py](fundamental/logger.py) | `Logger` — 모든 모듈이 공유하는 `[TAG] message` 콘솔 로거 |
| [fundamental/const.py](fundamental/const.py) | 프로젝트 전역 상수. 클래스 하나가 원래 어느 모듈/클래스가 쓰던 상수 묶음인지에 대응(예: `FaceFollowerConst`, `HardwareActuatorConst`) — AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기, `AX_18A`/`ActuatorControlTable`)도 여기 있다. 각 모듈은 여기서 import해서 자기 클래스 속성으로 다시 건다 |
| [hardware/controller.py](hardware/controller.py) | 시리얼 포트/보드레이트 초기화, 저수준 read/write (`set_speed`, `set_goal_position`, `get_present_position`) |
| [hardware/actuator.py](hardware/actuator.py) | `Actuator`(모터 1개 추상화) + `ArmController`(FK/IK ↔ Actuator 연동) |
| [kinematics/kinematics.py](kinematics/kinematics.py) | FK/IK. `Arm`/`Joint` + 순수 파이썬 선형대수. 하드웨어 불필요 |
| [kinematics/urdf_loader.py](kinematics/urdf_loader.py) | `configure/desky.urdf`를 파싱해 `Arm` 생성 (`xml.etree`) |
| [kinematics/simulate.py](kinematics/simulate.py) | 인터랙티브 FK 슬라이더 + IK 입력 3D 미리보기 (matplotlib). `src/app.py`의 `/api/render`도 여기 `draw_pose`/`draw_points`를 그대로 재사용 |
| [kinematics/mujoco_sim.py](kinematics/mujoco_sim.py) | MuJoCo로 실제 메시 기반 시뮬레이션 (선택 기능) |
| [kinematics/find_joint_limits.py](kinematics/find_joint_limits.py) | MuJoCo로 자기충돌이 시작되는 지점까지 관절 한계를 스윕해 `configure/joints.json`을 만드는 스크립트 |
| [kinematics/configure/desky.urdf](kinematics/configure/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |
| [perception/camera_geometry.py](perception/camera_geometry.py) | `camera_frame()`(핀홀 카메라 로컬 축) + `clamp_xy()` + `to_landmark()` — FaceTracker/HandTracker/BodyTracker, FaceFollower/HandFollower/BodyFollower가 공유하는 지오메트리 |
| [perception/face_tracker.py](perception/face_tracker.py) | `FaceTracker` — **인식은 안 함**, 휴대폰이 보낸 얼굴 랜드마크(코끝 + 양쪽 눈 바깥쪽 끝)를 3D 월드 좌표로 변환/시각화만 + `FaceFollower`(yaw 회전 + 높이 이동으로 화면 중앙 추종) |
| [perception/body_tracker.py](perception/body_tracker.py) | `BodyTracker` — **인식은 안 함**(얼굴 인식 실패 시 폴백), 휴대폰이 보낸 어깨 랜드마크(양쪽 어깨)를 3D 월드 좌표로 변환/시각화만 + `BodyFollower`(FaceFollower와 동일한 yaw+높이 추종) |
| [perception/hand_tracker.py](perception/hand_tracker.py) | `HandTracker` — **인식은 하지 않는다**, 휴대폰이 보낸 손 랜드마크(좌표만)를 end-effector의 FK 변환행렬을 이용해 3D 월드 좌표로 올리고 시각화만 한다 + `HandFollower` |
| [perception/follow_controller.py](perception/follow_controller.py) | `FollowController` — 얼굴>몸>손>idle 우선순위로 다음 명령을 정하고, 아무것도 안 보이면 idle 복귀 후 1번 관절을 두리번거리는 상태 머신 |
| [perception/document_scanner.py](perception/document_scanner.py) | `DocumentScanner` — 카메라 프레임에서 종이 문서 사각형을 검출하고 원근 보정(warp)해서 잘라낸다. OCR은 하지 않음(그 결과를 `Gemini.parse_document()`에 넘김) |
| [perception/gesture.py](perception/gesture.py) | 손 랜드마크 → 가위/바위/보 분류 + `GestureRecognizer`(N프레임 유지 확정, 엣지 트리거, 쿨다운). `_landmarks_of()`가 `Hand`/mediapipe 객체/리스트를 모두 받아내므로 인식 주체가 바뀌어도 그대로 동작 |
| [src/app.py](src/app.py) | `DeskyApp` — **조립만** 한다(서비스 생성 + 라우트 등록 + 실행). 페이지 `/`, `/mobile`. 병합 전 500줄 전역 스크립트였던 것을 아래 객체들로 쪼갰다 |
| [src/arm_service.py](src/arm_service.py) | `ArmService` — 팔 모델·하드웨어 연결·관절각 캐시·자기충돌 검사·이동 명령. 병합 전 `app.py`의 전역 `arm_ctrl`/`arm`/`_q_cache`/`_current_q()` 자리 |
| [src/render.py](src/render.py) | `ArmRenderer`(URDF 1회 파싱 → 자세를 PNG로) + `ScenePreview`(Figure 재사용하는 로컬 3D 창) |
| [src/perception_loop.py](src/perception_loop.py) | `PerceptionLoop` — 프레임→얼굴/손→추종 명령→제스처→미리보기. **세 주기가 다르다**(판단=매 프레임, 명령=`COMMAND_MIN_INTERVAL_S`, 창 갱신=`VIS_MIN_INTERVAL_S`) |
| [src/gesture_bridge.py](src/gesture_bridge.py) | `GestureBridge` — 확정된 제스처를 폰 명령(소켓)과 조명 제어로 배선. 조명은 별도 스레드(영상 멈춤 방지) |
| [src/api/arm.py](src/api/arm.py) | `ArmAPI` — `/api/status`, `/fk`, `/ik`, `/render`, `/goto_position`, `/goto_joints`, `/goto_joint`. HTTP만 알고 상태는 `ArmService`가 소유 |
| [src/api/gemini.py](src/api/gemini.py) | `Gemini` — `/api/ask`(채팅/요약), 음성 클립 받아쓰기(STT), 문서 이미지 글자 읽기(`parse_document`) |
| [src/api/camera.py](src/api/camera.py) | `Camera` — `/ws/camera` **양방향**. 폰→서버: JPEG 프레임·WebM 음성(매직 바이트로 구분)·손 랜드마크 JSON(텍스트). 서버→폰: transcript·제스처 명령(`broadcast()`) |
| [src/api/calendar.py](src/api/calendar.py) | `Calendar` — `/api/calendar/*`. **순수 저장소**(상대 날짜 해석은 폰이 자기 시계로 함). `calendar_events.json`에 임시파일→rename으로 원자적 저장 |
| [src/api/light.py](src/api/light.py) | `Light` — `/api/light`. 폰이 파이(HTTP)를 직접 못 부르므로(혼합 콘텐츠 차단) 서버가 대신 호출. 파이가 없어도 앱은 뜨고 요청만 502 |
| [raspberry-server/server.py](raspberry-server/server.py) | 파이에서 **따로** 실행하는 서보 서버(`/api/press`). pigpio→lgpio→모의 모드 자동 선택 |
| [src/api/scan.py](src/api/scan.py) | `ScanAPI` — `/api/scan/preview.jpg`, `/api/scan/detect`, `/api/scan/parse` 3라우트. `DocumentScanner` + `Gemini`를 연결 |
| [src/templates/index.html](src/templates/index.html) | 대시보드 UI (위치 입력, 관절별 입력, 상태, 카메라 프리뷰) |
| [src/templates/mobile.html](src/templates/mobile.html) | 팔에 장착된 휴대폰에서 여는 페이지 — 카메라 스트리밍, Gemini 채팅/요약, 음성 대화(연속 STT+TTS), 음성 명령, 문서 스캔 UI |

`hardware/`, `kinematics/`, `perception/`, `src/`, `fundamental/` 모두 패키지(`__init__.py`
포함)이며, 내부 모듈 간 임포트는 상대 임포트(`.controller`, `.kinematics`)를 쓴다.
`fundamental/`은 루트에 있으므로 어디서든 `from fundamental.logger import Logger`,
`from fundamental.const import ...`로 절대 임포트한다. 항상 **저장소 루트에서** 실행할 것
(`python main.py`, `python -m kinematics.kinematics`, `python -m kinematics.simulate`,
`python -m src.app` 등) — 하위 폴더의 파일을 직접 실행하면(`python kinematics/kinematics.py`)
루트가 `sys.path`에 없어 `fundamental` 임포트가 깨진다.

### 계층
```
main.py
  └─ src.app.DeskyApp                          # 조립만: 서비스 생성 + 라우트 등록 + 실행
       ├─ src.arm_service.ArmService ── hardware.actuator.ArmController ── Actuator ×5 ── Controller ── fundamental.const.AX_18A + dynamixel_sdk
       │                                    └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
       ├─ src.render.ArmRenderer             # URDF 1회 파싱 → PNG / 3D 창
       ├─ src.api.arm.ArmAPI                 # 팔 제어 라우트 7개
       ├─ src.api.gemini.Gemini ── (google-genai)
       ├─ src.api.camera.Camera ── Gemini    # /ws/camera 양방향
       ├─ src.api.scan.ScanAPI ── perception.document_scanner.DocumentScanner + Gemini
       ├─ src.api.calendar.Calendar          # /api/calendar/*        (웹 계열)
       ├─ src.api.light.Light ──HTTP──▶ raspberry-server              (웹 계열)
       ├─ src.gesture_bridge.GestureBridge ── perception.gesture + Camera + Light
       └─ src.perception_loop.PerceptionLoop      # 메인 스레드(cv2 창 제약), 라우트 아님
            ├─ perception.face_tracker.{FaceTracker,FaceFollower}   ← 폰이 보낸 랜드마크 좌표 변환만
            ├─ perception.body_tracker.{BodyTracker,BodyFollower}   ← 폰이 보낸 랜드마크 좌표 변환만 (얼굴 폴백)
            ├─ perception.hand_tracker.{HandTracker,HandFollower}   ← 폰이 보낸 랜드마크 좌표 변환만
            ├─ perception.follow_controller.FollowController        (얼굴 > 몸 > 손 > idle)
            └─ src.render.ScenePreview
```

**병합 후 두 계열의 유일한 접점은 `Camera`(소켓)와 `ArmService`(팔 상태)** 둘뿐이며, 둘 다
락으로 감싸져 웹 요청 스레드와 카메라 루프가 안전하게 공유한다. 새 기능을 붙일 때도 이
경계를 유지할 것 — 기능 객체를 만들고 `register(app)`으로 자기 라우트를 등록하면
`app.py`는 건드릴 필요가 없다.

**폰이 보낸 손 랜드마크 하나를 두 소비자가 나눠 쓴다**(추종 + 제스처). 서버에서
mediapipe(Python) Hands나 FaceMesh, Pose를 다시 돌리지 말 것 — 얼굴/손/몸 인식 셋 다
이제 휴대폰이 전담하고(MediaPipe Tasks Vision, JS), 서버에서 같은 걸 다시 돌리면 결과가
두 갈래로 갈리고 프레임레이트도 다시 무너진다(docs/MERGE.md §2.1). mediapipe(Python)는
이 프로젝트에 더 이상 설치할 필요가 없다.
`main.py`는 `src.app`을 임포트해 실행하는 얇은 launcher다 — 실제 하드웨어/Gemini 초기화는
`src/app.py` 쪽에서 일어난다(하드웨어나 Gemini가 없으면 실패를 잡아 "미연결/미구성" 상태로
대체 — 아래 절 참고).
`hardware.actuator.ArmController`가 FK/IK(`kinematics.Arm`)와 실제 `Actuator` 5개를 연결한다.
`Actuator.id`(DYNAMIXEL id)와 URDF `<dynamixel id="">`가 일치하는 조인트를 서로 매칭하므로,
생성자에 넘기는 `actuators` 리스트는 순서에 상관없다.

```python
from hardware.controller import Controller
from hardware.actuator import Actuator, ArmController

controller = Controller()
actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
arm_ctrl = ArmController(actuators)             # arm=load_arm() 기본값 사용

q, ok = arm_ctrl.goto_position((0.3, 0.05, 0.15))   # IK 계산 → 5개 서보에 goto() 디스패치
pos = arm_ctrl.get_position()                        # 5개 서보 현재각 read → FK로 위치 계산
```

- `goto_position`은 IK가 수렴하지 않으면(`converged=False`) 서보를 움직이지 않는다.
- `get_position`은 액추에이터 중 하나라도 `None`을 반환하면(통신 실패) 전체 결과도 `None`.

## 웹 제어 대시보드 (하드웨어 없어도 실행됨)

```bash
python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt   # 최초 1회
python main.py   # 저장소 루트에서 실행, https://localhost:8000
# 또는 동일하게: python -m src.app
```

`src/app.py`는 시작 시 `Controller()`로 시리얼 포트를 열고 액추에이터 5개 + `ArmController`를
구성하려고 시도한다 — **성공하면** 실제 서보를 제어할 수 있고, **실패하면**(장치 없음,
`.env` 없음, `dynamixel_sdk`/`python-dotenv` 미설치 등) 예외를 잡아 "no hardware connected"
상태로 대시보드가 그대로 뜬다(아래 참고). 하드웨어가 연결되어 있으면 브라우저 대시보드에서:

- 관절별 서보각 슬라이더로 FK 미리보기(`/api/fk`, `/api/render`) 조작 → "Move actuators"로 실제 이동(`/api/goto_joints`)
- 목표 위치 (x, y, z) 입력 → IK 계산(`/api/ik`)으로 슬라이더가 해로 스냅
- 1.5초마다 `/api/status`로 현재 위치를 폴링해 표시
- 팔에 장착된 휴대폰이 `/mobile`에서 스트리밍 중이면 카메라 프리뷰도 함께 표시(아래 참고)

라우트: `GET /`(대시보드), `GET /mobile`, `GET /api/status`, `POST /api/fk`, `POST /api/ik`,
`POST /api/render`, `POST /api/goto_position`, `POST /api/goto_joints`, `POST /api/goto_joint`,
`WS /ws/camera`, `GET /api/camera/latest.jpg`, `GET /api/camera/status`, `POST /api/ask`,
`GET /api/scan/preview.jpg`, `GET /api/scan/detect`, `POST /api/scan/parse`.

`app.run(..., port=8000, debug=False, threaded=True, ssl_context="adhoc")` 고정:

- 포트 5000이 아니라 8000인 이유: macOS AirPlay Receiver가 5000번을 기본으로 점유.
- `debug=False`인 이유: Flask 리로더가 모듈을 다시 임포트하면 시리얼 포트가 두 번 열림 —
  `debug=True`로 바꾸지 말 것.
- `threaded=True`인 이유: `/ws/camera` 웹소켓이 세션 내내 연결을 유지하므로, 개발 서버가
  요청마다 스레드를 띄우지 않으면 다른 HTTP 요청이 막힌다.
- `ssl_context="adhoc"`인 이유: 아래 카메라 스트리밍 절 참고.

### 모바일 카메라 스트리밍 (`/mobile`, `/ws/camera`)

팔 끝(end-effector)에 장착된 휴대폰에서 `https://<이 서버의 LAN IP>:8000/mobile`을 열면:

1. "Start camera + streaming" 버튼을 누르면 `getUserMedia({video: {facingMode: {exact: "user"}},
   audio: false})`로 전면 카메라 권한만 요청한다(마운트 구조상 전면 카메라만 써야 하므로
   `exact`로 강제 — 전면 카메라가 없으면 조용히 후면으로 넘어가는 대신 `OverconstrainedError`로
   바로 실패한다. 음성 대화는 브라우저 내장 `SpeechRecognition`이 별도로 마이크 권한을 요청 —
   아래 음성 대화 절 참고).
2. 승인되면 `<video>`로 로컬 프리뷰를 띄우고, `FRAME_INTERVAL_MS`(66ms ≈ 20fps)마다
   `<canvas>`에 프레임을 그려 `canvas.toBlob('image/jpeg')`로 JPEG를 만들어
   `wss://<host>/ws/camera`로 바이너리 그대로 전송한다. 프레임마다 완결된 JPEG라서 서버는
   컨테이너/코덱을 신경 쓸 필요가 없다. 같은 캔버스에 대고 손 인식(HandLandmarker)도 매번
   같이 돌린다 — 아래 "손 인식" 절 참고.

**방향 보정 (마운트 vs 손에 든 상태):** 이 휴대폰은 손으로 드는 게 아니라 팔의
end-effector에 고정 장착돼 있어서, 브라우저가 "지금 든 방향 기준으로 똑바로" 자동
보정하는 영상이 실제로 앉아서 보는 방향과 어긋날 수 있다. `mobile.html`은 매 프레임마다
Screen Orientation API(`screen.orientation.angle`, 구형 브라우저는 `window.orientation`
폴백)로 지금 회전 각도를 읽어 그만큼 반대로 돌린 뒤 — `<video>` 미리보기(CSS
`transform: rotate()`)와 웹소켓으로 보내는 프레임(캔버스에 그릴 때 `ctx.rotate()`) 양쪽에
동일하게 적용한다. `angle=0`(세로, 화면 안 돌아간 상태)일 때 90도를 돌리는 게 기준점 —
과거엔 서버(`src/app.py`)에서 이 90도를 하드코딩해 mediapipe 처리 직전에 돌렸지만, 마운트
각도가 바뀌면 다시 깨지는 문제가 있어 폰이 보고하는 실제 화면 방향 기준으로 클라이언트에서
보정하도록 바꿨다. 웹소켓으로 오는 프레임은 이제 항상 보정된(정방향) 상태이므로
`src/app.py`/`perception/hand_tracker.py`는 더 이상 프레임을 따로 회전하지 않는다. 마운트
기준 자체를 바꾸려면(예: 항상 90도가 아니라 180도가 기준이 되도록) `mobile.html`의
`correctionAngle()` 안 `90` 상수만 조정하면 된다.

서버(`src/api/camera.py`의 `Camera`)는 `flask_sock.Sock`으로 `/ws/camera`를 raw WebSocket으로
처리한다:

- 들어오는 바이너리 메시지 중 JPEG는 최신 프레임으로 저장(락으로 보호), WebM/Opus 음성
  클립은 Gemini로 받아쓰기(레거시 경로 — 현재 `mobile.html`은 STT에 브라우저 내장
  `SpeechRecognition`을 쓰므로 이 경로는 보내는 클라이언트가 있을 때만 동작).
- 들어오는 **텍스트** 메시지는 JSON으로 파싱해 `type: "hand_landmarks"`만 처리한다 —
  `mobile.html`이 보낸 손 랜드마크(좌표만, 이미지 아님)를 저장한다. 그 외 타입/파싱 실패는
  조용히 무시(연결을 끊지 않음). `Camera.latest_hand_landmarks()`로 읽는다 — 아래 "손 인식"
  절 참고.
- `GET /api/camera/latest.jpg` — 최신 프레임을 `image/jpeg`로 반환(아직 없으면 404).
- `GET /api/camera/status` — `{streaming, clients, frame_count, age_seconds}`.
- 대시보드(`/`)는 `/api/camera/status`를 0.5초마다 폴링하고, 스트리밍 중이면
  `<img src="/api/camera/latest.jpg?t=...">`를 갱신해 실시간처럼 보여준다(진짜 스트리밍이
  아니라 폴링 기반 프리뷰 — 데모 목적으로는 충분).

**HTTPS가 필요한 이유:** 브라우저는 secure context(HTTPS 또는 `localhost`)에서만
`getUserMedia`/`SpeechRecognition`을 허용한다. 휴대폰은 LAN IP로 접속하므로 `localhost`가
아니고, HTTPS가 필수다. `ssl_context="adhoc"`(pyOpenSSL)로 매 실행 시 자체 서명 인증서를
즉석에서 만들어 쓰므로 브라우저가 "안전하지 않음" 경고를 한 번 띄운다 — 데스크톱 대시보드,
모바일 페이지 둘 다에서 "고급 → 계속 진행"으로 넘어가야 한다.

### 얼굴/몸/손 인식 (`perception/face_tracker.py`, `perception/body_tracker.py`, `perception/hand_tracker.py`, `src/perception_loop.py`)

**얼굴/손/몸(어깨) 인식 셋 다 휴대폰이 한다** — 원래는 서버(`PerceptionLoop`)가
mediapipe로 처리했지만(손 → 얼굴 → 몸 순으로 옮겨갔다), 두 가지 문제가 있었다:

1. 컴퓨터 하나가 여러 모델 추론 + matplotlib 3D 미리보기 렌더링 + 하드웨어 제어를 매
   프레임 다 하려니 휴대폰이 실제로 보내는 프레임레이트(`FRAME_INTERVAL_MS` ≈ 20fps)를
   못 따라가서 로봇 반응이 느려지고 뚝뚝 끊겼다.
2. 서버가 받는 프레임은 JPEG 압축 + 다운스케일을 거쳐 화질이 떨어져, 인식 자체가 잘 안
   되는 경우도 있었다(특히 얼굴) — 휴대폰이 압축 전 원본 영상에서 직접 인식하면 이 문제도
   함께 없어진다.

그래서 손 → 얼굴에 이어 몸(어깨)도 같은 방식으로 옮겼다: `mobile.html`이 휴대폰
브라우저에서 직접 MediaPipe Tasks Vision(`HandLandmarker` + `FaceLandmarker` +
`PoseLandmarker`, 같은 CDN 패키지, ES 모듈로 로드)을 돌려 매 캡처 프레임마다 랜드마크
(좌표만, 이미지 아님)를 `/ws/camera`로 JSON 텍스트 메시지로 보낸다 — 손은
`{"type": "hand_landmarks", "landmarks": [...]}`(21점 전부), 얼굴은
`{"type": "face_landmarks", "faces": [{"center", "left_eye_outer", "right_eye_outer"}]}`
(468/478점 중 FaceTracker가 실제로 쓰는 3점만 이름 붙여서 — 대역폭 절약. 인덱스
1/33/263은 `fundamental.const.FaceTrackerConst`와 `mobile.html`의 JS 상수가 반드시
일치해야 한다), 몸은 `{"type": "body_landmarks", "bodies": [{"left_shoulder", "right_shoulder"}]}`
(33점 중 BodyTracker가 실제로 쓰는 양쪽 어깨 2점만 — 인덱스 11/12는
`fundamental.const.BodyTrackerConst`와 일치해야 한다). 어느 것도 안 보이면 빈 배열로
보낸다 — 서버가 "지금은 없다"를 곧바로 알아야 사라진 위치를 계속 붙잡고 있지 않는다.
어깨는 신뢰도(visibility)가 낮으면(옆모습이라 한쪽이 가려짐 등) 그 판정도 폰이 미리
걸러서(`mobile.html`의 `POSE_MIN_SHOULDER_VISIBILITY`) 아예 결과에서 뺀 채로 보낸다.

`src/api/camera.py`의 `Camera`가 최신 값을 각각 저장하고(`latest_hand_landmarks()`/
`latest_face_landmarks()`/`latest_body_landmarks()`), `perception/hand_tracker.py`/
`perception/face_tracker.py`/`perception/body_tracker.py`의 `HandTracker`/`FaceTracker`/
`BodyTracker`는 **인식은 하지 않고** 그 좌표를 3D 월드 좌표로 변환/시각화만 한다
(`process_landmarks()`) — 그래서 서버 쪽 세 트래커 모두 mediapipe(Python)에 의존하지
않는다(이 프로젝트에 더 이상 그 패키지가 필요 없다). `HandLandmarker`/`FaceLandmarker`/
`PoseLandmarker` 로드/초기화는 서로 독립적으로 실패할 수 있다(오프라인, CDN 접근 불가,
구형 브라우저 등) — 하나가 실패해도 `mobile.html`이 그 인식만 조용히 비활성 상태로 두고
계속 동작한다(하드웨어/Gemini 미구성과 같은 부분 실패 패턴). **손 인식(`mobile.html`의
`HAND_TRACKING_ENABLED`)은 켜져 있다** — 가위바위보 제스처가 이 랜드마크로 동작하기
때문. 대신 **팔이 손 위치로 움직이는 것(추종)만** `FollowController.hand_follow_enabled`
(기본값 `FollowControllerConst.HAND_FOLLOW_ENABLED = False`)로 꺼 둔다 — 손 인식/제스처는
그대로 두고 "팔이 손을 쫓아다니는 것만" 원치 않는다는 요청으로 나뉜 두 스위치다:
`HAND_TRACKING_ENABLED`(휴대폰, 랜드마크 송신 자체)를 끄면 손 추종과 제스처가 함께
꺼지고, `HAND_FOLLOW_ENABLED`(서버, `FollowController`)만 끄면 제스처는 유지한 채
팔 추종만 꺼진다.

세 트래커 다 같은 핀홀 투영 아이디어를 쓴다: 휴대폰 카메라가 end-effector에 달려 있다는
전제 아래, 기준자(손은 손목~엄지CMC 3.5cm, 얼굴은 양쪽 눈 바깥쪽 끝 9cm, 몸은 양쪽 어깨
사이 0.4m)가 화면에 보이는 크기로부터 핀홀 투영을 역산해 카메라까지 거리를
추정하고(`estimate_depth`), 그 거리와 end-effector의 FK 변환행렬(`Arm.fk_matrix(q)`)로
랜드마크를 월드 좌표에 올린다(`landmark_to_world`). **캘리브레이션한 값이 아니므로
거리는 측정이 아니라 추정치다.** `perception/camera_geometry.py`의 `to_landmark()`가
휴대폰이 보낸 `[x,y,z]`/`{"x","y","z"}`를 세 트래커가 공유하는 `Landmark` 타입으로
정규화한다.

**몸(어깨) 인식은 얼굴 인식 실패 시 폴백이다** — 옆모습이거나 고개를 돌려서 얼굴이 안
잡힐 때도 어깨는 보이는 경우가 많다는 점을 이용한다. 처음엔 "사람 인식은 백엔드에서"
명시적으로 요청받아 서버가 mediapipe Pose로 직접 돌렸지만, 이후 손/얼굴과 같은 이유로
휴대폰으로 옮겼다 — 지금은 `collect_bodies()`가 `collect_faces()`/`collect_hands()`와
완전히 같은 모양이다(그림 인식 없이 매 프레임 수신한 좌표만 변환).

**얼굴 > 몸 우선순위는 "보이느냐"가 아니라 거리 기준이다** — 얼굴까지 거리가
`FollowControllerConst.FACE_BODY_SWITCH_DISTANCE_M`(기본 1m)보다 가까우면 얼굴을,
그보다 멀면(얼굴이 안 보여도 마찬가지) 몸이 보이는 한 몸을 추적한다. 멀리서는 얼굴
랜드마크(특히 눈 사이 거리로 하는 깊이 추정)가 화면에서 작아져 노이즈에 약해지는데,
어깨는 상대적으로 크고 안정적으로 잡히기 때문이다(`FollowController._pick_target()`).

**거리 경계에는 히스테리시스(데드밴드)를 둔다** — 단순 부등호 하나만 쓰면(거리 <
1m ? 얼굴 : 몸) 얼굴 깊이 추정치가 마침 그 경계 근처에서 맴돌 때(랜드마크 잔떨림,
고개를 살짝 돌리는 정도로도 흔함) 판정이 프레임마다 뒤집혀서, 아래 디바운스가 있어도
몇 초에 한 번씩 계속 얼굴<->몸을 갈아타며 위아래로 훑는 듯한 진동이 남았다(디바운스는
"몇 프레임 순간적으로 놓침"은 막아도, 판정 자체가 경계에서 계속 다시 뒤집히는 건
못 막는다). 그래서 `FollowControllerConst.FACE_BODY_SWITCH_HYSTERESIS_M`(기본
0.15m)만큼 경계를 두 개로 벌렸다(`FollowController._pick_target()`) — 지금 얼굴을
보는 중이면 거리가 `FACE_BODY_SWITCH_DISTANCE_M + HYSTERESIS`보다 멀어져야 몸으로,
지금 몸을 보는 중이면 그보다 `- HYSTERESIS`만큼 가까워져야 얼굴로 되돌아간다. 이
판정에 쓰는 얼굴 깊이 자체도 `FollowControllerConst.FACE_DEPTH_SMOOTHING`으로 별도
평활한다(`FollowController._smooth_face_depth()` — `FaceFollower`가 화면 오프셋에
거는 평활과는 독립된 상태).

**대상 전환은 디바운스된다** — 얼굴/몸 인식이 프레임마다 100% 성공하지는 않아서, 이
디바운스가 없으면 한두 프레임 놓칠 때마다 다른 대상으로(또는 idle로) 갈아타게 되는데,
코끝(얼굴 중심)과 어깨 중점(몸 중심)은 화면상 높이가 달라서 그때마다 팔 높이(z)가
왔다갔다 흔들리는 문제가 있었다. `FollowController._pick_target()`으로 "이번 프레임
기준 이상적인 대상"을 정하고(위 히스테리시스 반영), 그게 지금 추적 중인 대상과
다르면 바로 갈아타지 않고 `FollowControllerConst.TARGET_SWITCH_TIMEOUT_S`(기본
0.5초) 동안 계속 그 상태가 유지돼야 실제로 전환한다(`_update_active_target()`). 그
유예 기간 동안은 이전 대상을 그대로 쓰고, 이번 프레임에 그 데이터가 없으면(예: 살짝
놓침) 팔은 명령 없이 가만히 있는다 — 엉뚱한 좌표로 움직이지 않게. idle(추적 대상이
아예 없던 상태)에서 뭔가 새로 나타났을 때는 반응성을 위해 디바운스 없이 바로 추적을
시작한다. `FollowController`의 우선순위는 **얼굴 > 몸 > 손 > idle**(히스테리시스 +
디바운스 반영)이다 — `BodyFollower`는 화면 오프셋 → yaw 회전 + 높이 이동(EMA 평활
포함)까지 `FaceFollower`와 완전히 같은 알고리즘을 쓴다(값은
`fundamental.const.BodyFollowerConst`로 독립적으로 튜닝 가능).

`PerceptionLoop.process_frame()`의 결정 루프(인식 → `FollowController.next_command()` →
`ArmService.execute()`)는 프레임이 들어올 때마다(최대 ~20fps) 돌지만, 로컬 미리보기 창
(cv2 + matplotlib 3D 씬, `PerceptionLoop.update_preview()`)은 그보다 훨씬 비싸서
`AppConst.VIS_MIN_INTERVAL_S`로 독립적으로 더 느리게(기본 0.1초 간격) 갱신한다 — 로봇
동작 자체에는 영향 없는 디버그용 창이라 괜찮다. 실제 하드웨어 명령도
`AppConst.COMMAND_MIN_INTERVAL_S`(기본 0.15초)보다 자주는 나가지 않는다 — 목표가 프레임마다
조금씩 바뀔 때 그때그때 전부 재명령하면 로봇이 계속 새로 움직이기 시작해서 산만해 보이기
때문이다(`HardwareActuatorConst.DEFAULT_SPEED_PERCENT`를 낮춘 것과 같은 목적 — "주의사항" 절
참고). `ArmService`는 명령을 실행하는 즉시(하드웨어를 다시 읽지 않고) 그 값을 관절각
캐시에 반영한다 — 안 그러면 `Q_CACHE_TTL_S`(3초) 동안 추종 루프가 "이미 여러 번 명령해서
실제로는 많이 움직인 상태"를 모른 채 매번 같은(오래된) 기준 각도에 보정치를 얹어 팔이
점점 크게 흔들리는 문제가 있었다("주의사항" 절 참고).

### Gemini 채팅/요약/STT/문서 요약 (`src/api/gemini.py`, `POST /api/ask`)

`/mobile` 페이지에 "Ask Gemini" 섹션이 있다(대시보드 `/`에는 없음) — 텍스트를 입력하고
Chat/Summarize 모드를 골라 `POST /api/ask`로 보낸다. `src/app.py`는 하드웨어와 똑같은 패턴으로
시작 시 Gemini 클라이언트 구성을 시도하고, 실패해도 앱은 계속 뜬다:

- `GEMINI_API_KEY` 환경 변수(`.env`에 넣으면 `python-dotenv`가 있을 때 자동 로드됨)와
  `google-genai` 패키지가 있어야 `Gemini.configured`가 `True`가 된다.
- 실패하면(키 없음, 패키지 미설치 등) `Gemini.client = None`, `Gemini.error`에 원인 저장 —
  `/mobile`은 "⚠ Gemini not configured — <원인>" 배너를 띄우고 입력/버튼을 비활성화한다.
- 답은 **TTS로 그대로 읽히는 것을 전제**로 짧고(`CHAT_INSTRUCTION`은 100자 이내), 목록·기호·
  마크다운·이모지 없이 말하듯 쓰도록 system instruction에 명시돼 있다.
- 사고(thought) 수준을 억지로 낮추면 모델이 사고 과정을 답변 본문에 흘려 쓴다(페르소나·포맷
  체크리스트가 새어 나옴) — 그래서 사고는 기본값에 맡기고 `max_output_tokens`만 넉넉히(8192)
  준 뒤 `Gemini.answer_text()`가 응답의 `thought` 파트를 걸러내고 실제 답변만 돌려준다. 이
  구조를 바꾸지 말 것.
- `POST /api/ask` — 바디 `{"text": "...", "mode": "chat"|"summary"}`.
  - 미구성 시 `503 {"error": "Gemini not configured: ..."}`.
  - `text` 누락 시 `400 {"error": "text is required"}`.
  - Gemini 호출 자체가 실패하면 `502 {"error": "..."}`.
  - 성공 시 `200 {"answer": "..."}`.
- `Gemini.transcribe(audio_bytes)` — WebM/Opus 음성 클립 하나를 한국어 텍스트로 받아쓴다
  (`Camera`의 레거시 음성 클립 경로가 호출).
- `Gemini.parse_document(image_bytes, mode)` — 문서 이미지에서 글자를 읽는다. `mode="text"`는
  원본 줄바꿈까지 살린 전체 텍스트, `mode="summary"`는 TTS로 읽어주기 좋은 3문장 이내 요약
  (`ScanAPI.parse`가 호출).

### 문서 스캔 (`perception/document_scanner.py`, `src/api/scan.py`, `/mobile`)

`/mobile`의 "문서 스캔" 섹션 — 종이 문서를 카메라로 찾아 잘라낸 뒤 Gemini에게 읽힌다:

1. **실시간 미리보기** (`GET /api/scan/preview.jpg`, 0.5초 폴링) — 최신 카메라 프레임에서
   `DocumentScanner.detect()`로 문서 사각형을 찾아 초록 사각형 + 번호를 그려 보여준다.
2. **"스캔하기"** (`GET /api/scan/detect`) — 그 순간의 원본 프레임과 검출된 문서 좌표를
   `ScanAPI`가 스냅샷으로 고정(freeze)해 저장하고, base64 이미지 + 좌표를 돌려준다. 화면은
   이 스냅샷 위에 사각형을 직접 그려 탭으로 선택할 수 있게 한다.
3. **파싱** (`POST /api/scan/parse`, 바디 `{"id": 문서번호|없음, "mode": "text"|"summary"}`) —
   저장된 스냅샷에서 선택한 문서 영역만 `DocumentScanner.four_point_transform()`으로 원근
   보정해 자른 뒤 `Gemini.parse_document()`로 읽는다. `id`가 없으면 프레임 전체를 그대로
   읽는다("전체 파싱").

검출 파이프라인(`DocumentScanner.detect`): Canny 엣지 → 모폴로지로 끊긴 테두리 연결 → 윤곽선
검출 → 면적/사각형성 필터 → 중복 제거 → 큰 순서로 번호 매기기. OCR 자체는 하지 않고, 잘라낸
이미지를 Gemini에게 넘겨서 처리한다.

### 음성 대화 · 음성 명령 (`/mobile`)

마이크 버튼("🎙 대화 시작")을 누르면 브라우저 내장 `SpeechRecognition`(Web Speech API)으로
연속 음성 인식을 시작한다 — 서버로 오디오를 보내지 않고 브라우저가 직접 텍스트로 변환한다.
5초간 새 텍스트가 없으면(`SILENCE_MS`) 그때까지 들은 문장을 확정하고:

- **알려진 명령과 일치하면**(`handleVoiceCommand()`) Gemini에 묻는 대신 해당 기능을 즉시
  실행한다 — 예: "스캔해줘"(스캔 버튼 클릭), "카메라 시작"(스트리밍 시작), "요약 모드로
  바꿔"(모드 전환), "N번 읽어줘"(문서 선택 화면에서 해당 문서를 Gemini로 읽어줌), "대화
  종료"(세션 종료) 등.
- **명령이 아니면** `POST /api/ask`로 Gemini에게 묻고, 답을 브라우저 `speechSynthesis`(TTS,
  `lang="ko-KR"`)로 읽어준 뒤 자동으로 다시 듣기 상태로 돌아간다(`speakThenResume`).

이 흐름은 전부 `mobile.html`의 클라이언트 자바스크립트에서 처리되며, 서버는 매 질문마다
`/api/ask` 한 번만 받는다 — 대화 세션 상태(`sessionActive`/`processing`/`speaking`)는 서버가
아니라 브라우저 쪽에 있다.

### 일정 / 조명 / 제스처 (병합된 웹 기능)

**일정** (`src/api/calendar.py`, `calendar.html`, `mobile.html` 7.5절) — 달력은 모달이
아니라 전용 페이지 `/calendar`다(폰 화면에서 달력과 영상을 같이 두면 둘 다 작아진다).
넘어가는 동안 `/mobile`이 내려가 카메라·웹소켓이 끊기고, 돌아오면 자동 재시작된다.
서버는 **순수 저장소**다.
"오늘/내일/다음 주 화요일" 같은 상대 날짜를 `YYYY-MM-DD`로 바꾸는 일은 전부 폰이
자기 로컬 시계로 한다(서버와 폰의 시간대가 달라도 "오늘"이 어긋나지 않게). 서버에
날짜 해석 로직을 추가하지 말 것. `calendar_events.json`에 임시파일→rename으로 저장.

**조명** (`src/api/light.py`, `raspberry-server/`) — 경로가 한 단계 더 있는 이유가 있다:

```text
폰 ──HTTPS──▶ 이 서버 (/api/light) ──HTTP──▶ 파이 (/api/press) ──PWM──▶ 서보
```

폰이 파이를 직접 부르면 이 페이지가 HTTPS라 브라우저가 혼합 콘텐츠로 **조용히** 차단한다
(Chrome은 사용자가 허용할 방법조차 없다). 페이지를 HTTP로 내리는 것도 답이 아니다 —
getUserMedia가 보안 컨텍스트를 요구한다. 서버 간 통신은 브라우저를 거치지 않으므로
혼합 콘텐츠도 CORS도 적용되지 않는다.

**제스처** (`perception/gesture.py`, `src/gesture_bridge.py`) — 인식·판정·실행 주체가 셋 다 다르다:

```text
폰(MediaPipe Tasks Vision로 손 인식)
   └─▶ 서버 perception.gesture (N프레임 유지 + 쿨다운으로 확정)
          └─▶ 폰 (카메라·마이크·화면이 거기 있으므로 실행은 폰)
```

가위=스캔, 바위=대화 시작, 보=카메라+조명 끄기. 폰이 올려 준 랜드마크 **하나**를
팔 추종과 제스처가 함께 쓴다. `mobile.html`의 `HAND_TRACKING_ENABLED`를 false로 두면
손 추종과 제스처가 **함께** 꺼진다(얼굴 추적·idle 두리번거림·나머지 웹 기능은 유지) — 팔
추종만 따로 끄고 싶으면(제스처는 유지) 위 "얼굴/몸/손 인식" 절의 `HAND_FOLLOW_ENABLED`
참고.

### 서버 로컬 카메라 미리보기 창 (`src.perception_loop.PerceptionLoop`)

`python main.py`와 `python -m src.app` 둘 다 `src/app.py`의 `run()` 함수 하나를 호출하고,
그건 `create_app(**kwargs).run()`으로 이어진다(`DeskyApp.run()`) — 중복 구현 방지.
`DeskyApp.run()`이 하는 일:

1. `DeskyApp.start_server()`가 Flask 서버(`self.flask.run(...)`)를 **백그라운드 스레드**로
   띄운다 — `flask.run()`은 영원히 블록되는 호출이라, 메인 스레드에서 그대로 부르면 그 뒤
   코드가 절대 실행되지 않는다.
2. 메인 스레드에서 `PerceptionLoop.run_forever()`가 블로킹된다. 매 새 프레임마다
   `process_frame()`이: `FaceTracker.process_landmarks()`/`HandTracker.process_landmarks()`/
   `BodyTracker.process_landmarks()`로 얼굴/손/몸을(셋 다 인식은 휴대폰이 끝냈고, 여기선
   좌표 변환만) 처리해 골격을 오버레이한 뒤 `cv2.imshow(AppConst.WINDOW_CAMERA, ...)`로
   서버가 실행 중인 컴퓨터 화면에 띄운다. 같은 프레임의 현재 관절각(FK)과 인식된
   얼굴/손/몸의 월드 좌표로 "3D scene (robot + hand/face/body)" 창(`AppConst.WINDOW_SCENE`)도
   함께 갱신한다(단, 이 미리보기 갱신 자체는 `AppConst.VIS_MIN_INTERVAL_S`로 결정 루프보다
   느리게 스로틀된다 — 위 "얼굴/몸/손 인식" 절 참고). **cv2 창 관련 호출은 반드시 메인
   스레드에서 실행해야 한다** — macOS에서는 OpenCV HighGUI가 메인 스레드가 아니면
   `imshow`/`waitKey`를 조용히 무시한다.
3. `cv2.waitKey(1)`로 HighGUI 이벤트 루프를 돌리면서(이게 없으면 창이 아예 갱신되지 않는다)
   `'q'`/Esc 입력 시 종료한다. `frame_count`가 실제로 바뀐 경우에만 디코드/표시하도록
   체크한다 — `waitKey(1)`은 "최대 1ms"일 뿐 보장이 아니라서, 이 체크가 없으면 휴대폰이 보내는
   실제 프레임(최대 ~20fps)보다 훨씬 빠르게 같은 프레임을 반복 디코드/표시하게 된다.

### 하드웨어 미연결 시 동작

`hardware.controller`/`hardware.actuator` 임포트 자체(즉 `dynamixel_sdk` 누락 포함)부터
`Controller()`/`ArmController()` 생성까지 전부 `src/app.py`의 `try/except Exception` 안에
있다. 실패하면 `arm_ctrl = None`이 되고:

- 대시보드 페이지는 그대로 렌더링되지만 "⚠ No hardware connected — <원인>" 배너가 뜨고 모든
  입력/버튼이 `disabled` 처리된다 (조인트 목록만 `kinematics.urdf_loader.load_arm()`로 별도
  로드해서 보여준다).
- `GET /api/status` → `{"connected": false, "error": "..."}` (200, 에러 아님).
- `POST /api/goto_position`, `POST /api/goto_joints`, `POST /api/goto_joint` →
  `503 {"error": "no hardware connected"}`.
- `/api/fk`, `/api/ik`, `/api/render`는 하드웨어 없이도 정상 동작(계산 전용).

## 3D 시뮬레이션 (하드웨어 없이 미리보기)

```bash
python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt   # 최초 1회
python -m kinematics.simulate   # 저장소 루트에서 실행
```

`kinematics/simulate.py`는 `kinematics.urdf_loader.load_arm()`으로 로봇 형상
(`kinematics/configure/desky.urdf`)을 읽어 인터랙티브 3D 창을 띄운다 — 관절마다 서보각
슬라이더가 있어 움직이면 FK로 자세가 즉시 갱신되고, x/y/z를 입력해 IK를 풀면 슬라이더가 해로
스냅된다. `src/app.py`의 `/api/render`도 여기 `draw_pose`/`workspace_bounds`를 그대로
재사용해 웹 대시보드의 3D 미리보기가 이 창과 동일하게 그려진다.

주의: `kinematics/simulate.py`는 서보각을 URDF의 `<limit>`(IK용 소프트 리미트)로 클램프하지
않는다 — 실제 `Actuator.goto()`도 물리 범위(0~300°) 안이면 그 리미트를 무시하고 그대로
움직이므로, 시뮬레이션도 동일하게 동작해야 "실제 로봇과 정확히 같은" 모션이 된다.

메시 기반(정확한 형상) 시뮬레이션이 필요하면 `python -m kinematics.mujoco_sim`(MuJoCo 필요,
선택 의존성) — `kinematics/simulate.py`는 STL을 바운딩 박스로 근사하므로 매 프레임 다시
그리기엔 충분히 빠르지만 실제 메시 형상은 아니다.

## 기구학 사용법

```python
from kinematics.urdf_loader import load_arm
arm = load_arm()                        # kinematics/configure/desky.urdf에서 구성 로드 (권장)
# 또는: from kinematics.kinematics import Arm; arm = Arm()   # 하드코딩 fallback

pos = arm.fk([0,0,0,0,0])               # 관절각(rad) → end-effector 위치(x,y,z)
q, ok = arm.ik(target_pos)              # 위치 IK (damped least-squares)
q, ok = arm.ik(target_pos, target_rot=R3x3)   # 자세 포함(best-effort)
servo_deg = arm.q_to_servo_deg(q)       # 관절각(rad) → 서보각(0~300°)
T_ee = arm.fk_matrix(q)                 # 관절각(rad) → end-effector 4x4 월드 변환행렬 (perception.hand_tracker가 사용)
```
- IK는 수렴 여부 `bool`을 반환하며, 도달 불가 목표도 예외 없이 `False` 반환.
- 관절각 `q`(rad, home 기준) ↔ 서보각(deg) 변환: `servo_deg = home_deg + direction * deg(q)`.

## 로봇 구성 수정 방법 (중요)

코드가 아니라 **[kinematics/configure/desky.urdf](kinematics/configure/desky.urdf) 하나만
수정**한다. `urdf_loader`가 이를 읽어 `Arm`을 만든다.
- 링크 길이 → 각 `<joint>`의 `<origin xyz>` (URDF 관례상 **미터** 단위)
- 관절 한계 → `<limit lower upper>` (라디안) — `kinematics/find_joint_limits.py`가 MuJoCo로
  자기충돌 없는 범위를 스윕해서 계산한 값이 `configure/joints.json`에 있다.
- 축 → `<axis xyz>`
- DYNAMIXEL id/모델/서보 캘리브레이션 → 프로젝트 확장 태그
  `<dynamixel id="" model="" home_deg="" direction="">` (표준 URDF 툴은 무시, 로더만 읽음)

## 상수 관리 ([fundamental/const.py](fundamental/const.py))

프로젝트 전역의 튜닝 상수(추종 이득/데드존/타임아웃 같은 숫자, DYNAMIXEL 컨트롤 테이블,
Gemini 시스템 프롬프트 등)는 전부 `fundamental/const.py` 한 곳에 모여 있다. 클래스 하나가
원래 그 상수를 쓰던 모듈/클래스 하나에 대응한다(`FaceFollowerConst`, `HandTrackerConst`,
`HardwareActuatorConst`, `GeminiConst` 등). 각 모듈은 해당 클래스를 import해서 자기
클래스 속성(또는 모듈 전역 변수)으로 다시 건다 — 예:

```python
from fundamental.const import FaceFollowerConst

class FaceFollower:
    YAW_GAIN = FaceFollowerConst.YAW_GAIN
```

그래서 원래 모듈 안의 참조(`self.YAW_GAIN`, 생성자 기본값 등)는 그대로 동작하고,
`DocumentScanner(**overrides)`처럼 인스턴스 생성 시 값을 덮어쓰는 기존 메커니즘도 안 깨진다.
**새 튜닝 상수를 추가할 때는 이 패턴을 따를 것** — 원래 모듈에 바로 리터럴을 박지 말고
`fundamental/const.py`에 설명과 함께 추가한 뒤 그 모듈에서 import한다.

병합으로 추가된 클래스: `GestureConst`(랜드마크 인덱스·확정 프레임 수·쿨다운),
`CalendarConst`(저장 파일명·날짜 형식), `LightConst`(파이 주소·스위치 각도·타임아웃).
웹 계열 브랜치는 이 값들을 각 모듈에 하드코딩하고 있었는데, 이식하면서 위 패턴에 맞췄다.
`AppConst`에는 서버 바인딩(`SERVER_HOST`/`SERVER_PORT`/`SSL_CONTEXT`)과 미리보기 창
제목(`WINDOW_CAMERA`/`WINDOW_SCENE`)이 추가됐다 — 창 제목은 cv2에서 곧 창 식별자라
그리는 곳과 닫는 곳이 같은 문자열을 써야 한다.

옮기지 않은 예외(값이 아니라 구성/경로 로직이라 그대로 둔 것): `kinematics/kinematics.py`의
`DEFAULT_JOINTS`/`TOOL_OFFSET`(그 파일의 `Joint` 클래스를 직접 생성하는 코드),
`kinematics/urdf_loader.py`의 `_DEFAULT_URDF_PATH`(`__file__` 기준으로 계산되는 경로).

## 화면 색 ([src/static/theme.css](src/static/theme.css))

색·글꼴·칩/큰버튼 같은 공통 요소는 전부 `theme.css`의 CSS 변수에 있고, 각 템플릿에는
그 화면만의 배치 CSS만 둔다. **템플릿 안에 색 리터럴(`#4B45F0` 같은 것)을 직접 쓰지 말 것** —
토큰(`var(--signal)`)을 쓴다. 그래야 색 하나를 바꿔도 두 화면이 같이 따라온다.

무채색 골격 + 강조색 둘이라는 규칙이 이 팔레트의 전부다:
`--signal`(인디고)은 **지금 살아 있는 것**에만(영상 면, 로봇의 말, 오늘/고른 날짜),
`--alert`(벽돌)은 **되돌리기 어려운 동작과 오류**에만(카메라 끄기, 실패 메시지).
나머지는 `--paper`/`--panel`/`--ink`/`--graphite`/`--dim` 회색조다. 새 색을 추가하기 전에
기존 둘로 표현할 수 없는지 먼저 확인할 것. 잉크(어두운) 면 위의 칩은 그 면에 `on-ink`
클래스를 붙이면 색이 자동으로 뒤집힌다.

**같은 값을 뜻하는 상수가 클래스마다 따로 선언돼 있으면 `CameraGeometryConst`처럼 공유
클래스로 합친다** — 예: `FaceTrackerConst.FOCAL_NORM`과 `HandTrackerConst.FOCAL_NORM`은
둘 다 "휴대폰 전면 카메라"라는 같은 물리 카메라를 가정한 값이라 우연히 같은 게 아니라
원래 하나여야 하는 값이었다(`FaceFollowerConst`/`HandFollowerConst`의
`CENTER_OFFSET_THRESHOLD`도 마찬가지). 이런 경우를 발견하면 값을 공유 클래스로 옮기고
각 원래 클래스의 주석에 "여기 없음 — OOO로 통합" 이유를 남긴다.

## 주의사항 / 알려진 특성

- `hardware/controller.py`의 read/write는 통신·하드웨어 에러 시 **예외로 죽지 않고** 경고
  출력 후 계속 진행. `set_*`는 성공 여부 `bool`, `get_present_position`은 실패 시 `None`
  반환 → **호출부에서 `None` 처리 필요**.
- `Controller`는 `with Controller() as c:` 컨텍스트 매니저 지원 (GC 대신 결정적 포트 정리).
- 각도↔유닛 변환은 `/360` 계수를 쓴다(범위 리미트로 의도적 처리됨 — 유지할 것).
- XML 주석 안에 `--`(이중 하이픈) 금지 (URDF 편집 시 파싱 에러 주의).
- 모든 모듈의 콘솔 출력은 `fundamental.logger.Logger.log(tag, message)`를 거쳐 `[TAG] message` 형식으로
  통일된다 (`CONTROLLER`, `ACTUATOR`, `WEBAPP`, `GEMINI`, `STT`, `SCAN`, `DOCSCAN`, `HAND`,
  `CAMERA`, `KINEMATICS`, `URDF` 태그). `Logger.enabled` 플래그 하나로 전체 로그를 끌 수 있다.
- `Gemini`/`DocumentScanner`는 모두 하드웨어와 같은 "부분 실패" 패턴을 따른다: 생성자에서
  의존성 실패를 흡수하고 `configured` 플래그 + `error` 문자열만 남긴다. 새 AI 기능을
  추가할 때도 이 패턴을 따를 것 — 앱 전체가 죽으면 안 된다. (`FaceTracker`/`HandTracker`는
  더 이상 이 패턴이 아니다 — 인식 자체를 안 하므로 `available` 개념이 없고, 실패 지점은
  `mobile.html`의 Hand/FaceLandmarker 로드 쪽으로 옮겨갔다.)
- `HardwareActuatorConst.DEFAULT_SPEED_PERCENT`와 `AppConst.COMMAND_MIN_INTERVAL_S`
  (0.15초)는 같이 튜닝하는 값이다 — 인식/렌더링이 밀렸다가 몰아서 명령이 들어올 때, 속도가
  높거나 명령을 너무 자주 내리면 로봇이 매번 확 움직여서 산만하고 갑작스러워 보인다. 이
  둘을 더 낮추면 더 완만해지지만 반응은 더 느려진다 — 트레이드오프. (얼굴 추종 속도 자체를
  조절하려면 이 둘보다 `FaceFollowerConst.YAW_GAIN`/`HEIGHT_GAIN`(+ 그 STEP_LIMIT들)과
  `CameraGeometryConst.CENTER_OFFSET_THRESHOLD`가 먼저다 — fundamental/const.py 참고.)
- **관절각 캐시는 "마지막으로 읽은 값"이 아니라 "마지막으로 명령한 값"으로 갱신된다**
  (`ArmService._remember_commanded_q()`, `goto_position`/`goto_joints`/`goto_joint` 안에서
  호출). 추종 루프는 `COMMAND_MIN_INTERVAL_S`(0.15초)마다 "현재 각도 + 이번 보정치"처럼
  상대적으로 다음 목표를 계산하는데, 그 "현재"를 하드웨어 읽기 캐시(`Q_CACHE_TTL_S`=3초)에서만
  가져오면 최대 3초 동안 이미 실행된 수십 번의 명령을 반영하지 못한 채 매번 같은(오래된)
  기준 각도에 새 보정치를 얹게 되어 팔이 점점 크게 흔들리는("폭주") 원인이 된다. 그래서
  이동 메서드들은 하드웨어를 다시 읽는 대신, 방금 자신이 무엇을 명령했는지를 캐시에 바로
  채워 넣는다. **팔을 움직이는 새 코드를 추가할 때도 `ArmService`의 이동 메서드를 거칠 것**
  — 이 메서드들을 우회해서 액추에이터에 직접 명령하면 이 캐시 동기화가 깨진다.
- `FaceFollower`는 `face.screen_offset`을 바로 안 쓰고 `_smooth()`로 지수이동평균(EMA,
  `FaceFollowerConst.SCREEN_OFFSET_SMOOTHING`)을 건 값으로 데드존 판정/스텝 계산을 한다 —
  가만히 있어도 FaceLandmarker 랜드마크가 프레임마다 조금씩 흔들려서, 원본 값을 그대로 쓰면
  오프셋이 데드존 경계를 넘나들며 팔이 "왔다갔다" 떨리는 현상(limit cycle)이 있었다.
- `HEIGHT_STEP_LIMIT`(높이 한 스텝의 최대 이동, m)은 `YAW_STEP_LIMIT`(각도, rad)과 단순
  비교하면 안 된다 — 높이 변화는 IK로 관절 3~5(어깨/팔꿈치/손목) **세 개**를 동시에 움직이는데,
  이전 값(0.03m)에서는 한 번의 최대 스텝이 팔꿈치를 최대 ~18도까지 움직였다(yaw의 한계인
  10도보다 큰 데다 관절 세 개가 한꺼번에 움직여서 체감 흔들림이 더 컸다) — "앉아서 좌우
  이동은 괜찮은데 일어날 때(높이 변화)는 많이 흔들린다"는 증상이 이것이었다. 관절 델타가
  yaw 쪽과 비슷한 크기가 되도록 0.015m로 낮췄다 — 이 상수를 다시 조절할 땐 값 자체가 아니라
  `arm.ik((0,0,z+step), seed=q)`로 나오는 실제 관절각 델타를 재보고 맞출 것.

## 검증 방법 (하드웨어 없이)

저장소 루트에서 실행:

```bash
python -m kinematics.kinematics     # FK/IK 라운드트립 self-test
python -m kinematics.urdf_loader    # URDF 로드 + FK/IK 라운드트립
python main.py                      # 대시보드 — 하드웨어/Gemini 없어도 뜬다
```
기대: IK `converged=True`, 위치 오차 < 1mm.

앱 전체가 조립되는지 빠르게 보려면:

```python
import src.app as app_mod
print(app_mod.arm_ctrl is not None, app_mod.gemini.configured)
for r in app_mod.app.url_map.iter_rules():
    print(r)
```
