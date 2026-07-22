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
프리뷰 전용), `flask`/`flask-sock`/`pyopenssl` (`webapp/` 제어 대시보드 + 모바일 카메라
스트리밍 전용), `google-genai` (`/api/ask` Gemini 채팅/요약 전용). 기구학 모듈 자체는
**표준 라이브러리만** 사용(numpy 불필요). Python 3.14 확인됨.

Homebrew Python은 PEP 668로 시스템 전역 `pip install`을 막으므로, 이 선택적 의존성 설치는
프로젝트 venv를 권장:

```bash
python -m venv .venv
source .venv/bin/activate
pip install matplotlib flask flask-sock pyopenssl google-genai
```

## 파일 구조

프로젝트는 하드웨어 제어(`hardware/`)와 기구학(`kinematics/`)을 별도 패키지로 분리한다.
둘은 서로 독립적이며, `logger.py`만 양쪽에서 공통으로 참조하는 루트 모듈이다.

```
desky/
├── main.py            # 진입점 — webapp.app.run() 실행. 하드웨어 유무 무관
├── scenario.py         # simulation이 재생하는 모션 웨이포인트 시퀀스
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
└── webapp/             # Flask 제어 대시보드 + 모바일 카메라 스트리밍 (하드웨어 없어도 실행됨)
    ├── app.py
    └── templates/
        ├── index.html
        └── mobile.html
```

| 파일 | 역할 |
|------|------|
| [main.py](main.py) | 진입점 — `webapp.app.run()` 호출(`python main.py` → `https://localhost:8000`). 하드웨어 없어도 실행됨 |
| [scenario.py](scenario.py) | `DEMO_SEQUENCE` — 서보각 웨이포인트 시퀀스. `simulation.simulate`가 3D 프리뷰로 재생하는 소스 |
| [logger.py](logger.py) | `Logger` — 모든 모듈이 공유하는 `[TAG] message` 콘솔 로거 |
| [hardware/controller.py](hardware/controller.py) | 시리얼 포트/보드레이트 초기화, 저수준 read/write (`set_speed`, `set_goal_position`, `get_present_position`) |
| [hardware/control_table.py](hardware/control_table.py) | AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기)을 클래스로 정의 |
| [hardware/util.py](hardware/util.py) | `Actuator`(모터 1개 추상화) + `ArmController`(FK/IK ↔ Actuator 연동) |
| [kinematics/kinematics.py](kinematics/kinematics.py) | FK/IK. `Arm`/`Joint` + 순수 파이썬 선형대수. 하드웨어 불필요 |
| [kinematics/urdf_loader.py](kinematics/urdf_loader.py) | `desky.urdf`를 파싱해 `Arm` 생성 (`xml.etree`) |
| [kinematics/desky.urdf](kinematics/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |
| [simulation/simulate.py](simulation/simulate.py) | `scenario.DEMO_SEQUENCE`를 3D로 애니메이션 재생 (matplotlib) |
| [webapp/app.py](webapp/app.py) | Flask 앱 + `run()` — 위치/관절각 제어 대시보드(`/`), 모바일 카메라 웹소켓(`/ws/camera`, `/mobile`), Gemini 채팅/요약(`/api/ask`), 서버 로컬 cv2 미리보기 창 |
| [webapp/templates/index.html](webapp/templates/index.html) | 대시보드 UI (위치 입력, 관절별 입력, 상태, 카메라 프리뷰) |
| [webapp/templates/mobile.html](webapp/templates/mobile.html) | 팔에 장착된 휴대폰에서 여는 페이지 — 카메라/마이크 권한 요청 후 프레임 전송 + Gemini 채팅/요약 UI |

`hardware/`, `kinematics/`, `simulation/`, `webapp/` 모두 패키지(`__init__.py` 포함)이며,
`hardware`/`kinematics` 내부 모듈 간 임포트는 상대 임포트(`.control_table`, `.kinematics`)를
쓴다. `logger.py`/`scenario.py`는 루트에 있으므로 어디서든 `from logger import Logger`,
`from scenario import DEMO_SEQUENCE`로 절대 임포트한다. 항상 **저장소 루트에서** 실행할 것
(`python main.py`, `python -m kinematics.kinematics`, `python -m simulation.simulate`,
`python -m webapp.app` 등) — 하위 폴더의 파일을 직접 실행하면(`python kinematics/kinematics.py`)
루트가 `sys.path`에 없어 `logger`/`scenario` 임포트가 깨진다.

### 계층
```
main.py
  └─ webapp.app (Flask) ── hardware.util.ArmController ── hardware.util.Actuator ×5 ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
                                            └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
```
`main.py`는 `webapp.app`을 임포트해 실행하는 얇은 launcher다 — 실제 하드웨어 초기화(`Controller()`,
`ArmController`)는 `webapp/app.py` 쪽에서 일어난다(하드웨어가 없으면 실패를 잡아 "no hardware
connected" 상태로 대체 — 아래 웹 대시보드 절 참고).
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

## 웹 제어 대시보드 (하드웨어 없어도 실행됨)

```bash
python -m venv .venv && source .venv/bin/activate && pip install flask flask-sock pyopenssl   # 최초 1회
python main.py   # 저장소 루트에서 실행, https://localhost:8000
# 또는 동일하게: python -m webapp.app
```

`main.py`는 이제 데모 시나리오를 돌리지 않고 `webapp.app`의 Flask 서버를 실행하는 진입점이다.
`webapp/app.py`는 시작 시 `Controller()`로 시리얼 포트를 열고 액추에이터 5개 + `ArmController`를
구성하려고 시도한다 — **성공하면** 실제 서보를 제어할 수 있고, **실패하면**(장치 없음,
`.env` 없음, `dynamixel_sdk`/`python-dotenv` 미설치 등) 예외를 잡아 "no hardware connected"
상태로 대시보드가 그대로 뜬다(아래 참고). 하드웨어가 연결되어 있으면 브라우저 대시보드에서:

- 목표 위치 (x, y, z, 미터) 입력 → `ArmController.goto_position()` 호출 (IK → 5개 서보 이동)
- 관절별 서보각(0~300°) 직접 입력 → 해당 `Actuator.goto()` 직접 호출 (IK 우회)
- 1.5초마다 `ArmController.get_position()`으로 현재 위치를 폴링해 표시
- 팔에 장착된 휴대폰이 `/mobile`에서 스트리밍 중이면 카메라 프리뷰도 함께 표시(아래 참고)

라우트: `GET /`(대시보드), `GET /api/status`, `POST /api/goto_position`,
`POST /api/goto_joint`. `app.run(..., port=8000, debug=False, threaded=True, ssl_context="adhoc")`
고정:

- 포트 5000이 아니라 8000인 이유: macOS AirPlay Receiver가 5000번을 기본으로 점유.
- `debug=False`인 이유: Flask 리로더가 모듈을 다시 임포트하면 시리얼 포트가 두 번 열림 —
  `debug=True`로 바꾸지 말 것.
- `threaded=True`인 이유: `/ws/camera` 웹소켓이 세션 내내 연결을 유지하므로, 개발 서버가
  요청마다 스레드를 띄우지 않으면 다른 HTTP 요청이 막힌다.
- `ssl_context="adhoc"`인 이유: 아래 카메라 스트리밍 절 참고.

### 모바일 카메라 스트리밍 (`/mobile`, `/ws/camera`)

팔 끝(end-effector)에 장착된 휴대폰에서 `https://<이 서버의 LAN IP>:8000/mobile`을 열면:

1. "Start camera + streaming" 버튼을 누르면 `getUserMedia({video, audio: true})`로 카메라+
   마이크 권한을 함께 요청한다(마이크 권한도 같이 뜨게 하기 위해 audio도 요청하지만, **이
   데모에서는 오디오는 전송하지 않고 비디오 프레임만 전송**한다 — "camera data stream"이
   요청 범위였고, 마이크 스트리밍은 별도 작업).
2. 승인되면 `<video>`로 로컬 프리뷰를 띄우고, 200ms(≈5fps)마다 `<canvas>`에 프레임을 그려
   `canvas.toBlob('image/jpeg')`로 JPEG를 만들어 `wss://<host>/ws/camera`로 바이너리 그대로
   전송한다. 프레임마다 완결된 JPEG라서 서버는 컨테이너/코덱을 신경 쓸 필요가 없다.

서버(`webapp/app.py`)는 `flask_sock.Sock`으로 `/ws/camera`를 raw WebSocket으로 처리한다:

- 들어오는 바이너리 메시지를 그대로 `camera_state["frame"]`에 저장(락으로 보호,
  `camera_lock`), `frame_count`/`frame_time`/연결된 클라이언트 수도 함께 추적.
- `GET /api/camera/latest.jpg` — 최신 프레임을 `image/jpeg`로 반환(아직 없으면 404).
- `GET /api/camera/status` — `{streaming, clients, frame_count, age_seconds}`.
- 대시보드(`/`)는 `/api/camera/status`를 0.5초마다 폴링하고, 스트리밍 중이면
  `<img src="/api/camera/latest.jpg?t=...">`를 갱신해 실시간처럼 보여준다(진짜 스트리밍이
  아니라 폴링 기반 프리뷰 — 데모 목적으로는 충분).

**HTTPS가 필요한 이유:** 브라우저는 secure context(HTTPS 또는 `localhost`)에서만
`getUserMedia`를 허용한다. 휴대폰은 LAN IP로 접속하므로 `localhost`가 아니고, HTTPS가
필수다. `ssl_context="adhoc"`(pyOpenSSL)로 매 실행 시 자체 서명 인증서를 즉석에서 만들어
쓰므로 브라우저가 "안전하지 않음" 경고를 한 번 띄운다 — 데스크톱 대시보드, 모바일 페이지
둘 다에서 "고급 → 계속 진행"으로 넘어가야 한다.

### Gemini 채팅/요약 (`/mobile`, `POST /api/ask`)

`/mobile` 페이지에 "Ask Gemini" 섹션이 있다(대시보드 `/`에는 없음) — 텍스트를 입력하고
Chat/Summarize 모드를 골라 `POST /api/ask`로 보낸다. `webapp/app.py`는 하드웨어와 똑같은
패턴으로 시작 시 Gemini 클라이언트 구성을 시도하고, 실패해도 앱은 계속 뜬다:

- `GEMINI_API_KEY` 환경 변수(`.env`에 넣으면 `python-dotenv`가 있을 때 자동 로드됨)와
  `google-genai` 패키지가 있어야 `gemini_client`가 구성된다.
- 실패하면(키 없음, 패키지 미설치 등) `gemini_client = None`, `gemini_error`에 원인 저장 —
  `/mobile`은 "⚠ Gemini not configured — <원인>" 배너를 띄우고 입력/버튼을 비활성화한다.
- `POST /api/ask` — 바디 `{"text": "...", "mode": "chat"|"summary"}`. `mode`에 따라 다른
  system instruction(`GEMINI_CHAT_INSTRUCTION`/`GEMINI_SUMMARY_INSTRUCTION`)을 사용해
  `gemini_client.models.generate_content(model=GEMINI_MODEL, ...)`를 호출한다.
  - 미구성 시 `503 {"error": "Gemini not configured: ..."}`.
  - `text` 누락 시 `400 {"error": "text is required"}`.
  - Gemini 호출 자체가 실패하면 `502 {"error": "..."}`.
  - 성공 시 `200 {"answer": "..."}`.

설치:

```bash
pip install google-genai   # 이미 설치된 python-dotenv도 필요 (.env 로드용)
```

```env
# .env에 추가
GEMINI_API_KEY=...
```

### 서버 로컬 카메라 미리보기 창 (`webapp.app.run`)

`python main.py`와 `python -m webapp.app` 둘 다 `webapp/app.py`의 `run()` 함수 하나를
호출한다(중복 구현 방지). `run()`이 하는 일:

1. Flask 서버(`app.run(...)`)를 **백그라운드 스레드**로 띄운다 — `app.run()`은 영원히 블록되는
   호출이라, 메인 스레드에서 그대로 부르면 그 뒤 코드가 절대 실행되지 않는다.
2. 메인 스레드에서 `/ws/camera`로 들어온 최신 프레임을 `cv2.imshow("Mobile camera", ...)`로
   서버가 실행 중인 컴퓨터 화면에 띄운다. **cv2 창 관련 호출은 반드시 메인 스레드에서 실행해야
   한다** — macOS에서는 OpenCV HighGUI가 메인 스레드가 아니면 `imshow`/`waitKey`를 조용히
   무시한다.
3. `cv2.waitKey(1)`로 HighGUI 이벤트 루프를 돌리면서(이게 없으면 창이 아예 갱신되지 않는다)
   `'q'`/Esc 입력 시 종료한다. `frame_count`가 실제로 바뀐 경우에만 디코드/표시하도록
   체크한다 — `waitKey(1)`은 "최대 1ms"일 뿐 보장이 아니라서, 이 체크가 없으면 휴대폰이 보내는
   실제 프레임(약 5fps)보다 훨씬 빠르게 같은 프레임을 반복 디코드/표시하게 된다.

### 하드웨어 미연결 시 동작

`hardware.controller`/`hardware.util` 임포트 자체(즉 `dynamixel_sdk` 누락 포함)부터
`Controller()`/`ArmController()` 생성까지 전부 `webapp/app.py`의 `try/except Exception` 안에
있다. 실패하면 `arm_ctrl = None`이 되고:

- 대시보드 페이지는 그대로 렌더링되지만 "⚠ No hardware connected — <원인>" 배너가 뜨고 모든
  입력/버튼이 `disabled` 처리된다 (조인트 목록만 `kinematics.urdf_loader.load_arm()`로 별도
  로드해서 보여준다).
- `GET /api/status` → `{"connected": false, "error": "..."}` (200, 에러 아님).
- `POST /api/goto_position`, `POST /api/goto_joint` → `503 {"error": "no hardware connected"}`.

## 3D 시뮬레이션 (하드웨어 없이 미리보기)

```bash
python -m venv .venv && source .venv/bin/activate && pip install matplotlib   # 최초 1회
python -m simulation.simulate   # 저장소 루트에서 실행
```

`simulation/simulate.py`는 `kinematics.urdf_loader.load_arm()`으로 로봇 형상
(`kinematics/desky.urdf`)을, `scenario.DEMO_SEQUENCE`로 모션 웨이포인트를 읽어 3D로 애니메이션
재생한다. `scenario.py`는 원래 `main.py`의 하드코딩된 데모 시퀀스였지만, 지금 `main.py`는 웹
대시보드(수동 위치/관절 제어)를 실행하는 쪽으로 바뀌어서 `DEMO_SEQUENCE`를 실제 하드웨어에 보내는
코드는 현재 없다 — `simulation/simulate.py`가 이 시퀀스의 유일한 소비자다.
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
python -m kinematics.kinematics     # FK/IK 라운드트립 self-test
python -m kinematics.urdf_loader    # URDF 로드 + FK/IK 라운드트립
```
기대: IK `converged=True`, 위치 오차 < 1mm.
