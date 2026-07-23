# desky

책상 위 5자유도 로봇팔로, end-effector에 **휴대폰을 장착**하는 프로젝트. DYNAMIXEL
AX-18A 서보 5개를 시리얼로 제어하며, URDF 기반 순기구학(FK)/역기구학(IK)을 제공한다.
휴대폰 카메라를 통해 손 인식(mediapipe)과 문서 스캔/음성 대화(Gemini)도 지원한다.

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
창), `mediapipe`(`perception/hand_tracker.py` 손 인식). 기구학 모듈 자체는 **표준
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
웹 앱(`src/`)을 별도 패키지로 분리한다. 모두 서로 독립적이며, `logger.py`만 전체가 공통으로
참조하는 루트 모듈이다.

```text
desky/
├── main.py             # 진입점 — src.app.run() 실행. 하드웨어 유무 무관
├── logger.py           # 공통 [TAG] message 콘솔 로거
├── requirements.txt    # 전체 의존성 (기능별 주석 포함)
├── hardware/           # 시리얼/서보 제어 (실제 하드웨어 필요)
│   ├── controller.py       # Controller — 시리얼 포트/보드레이트, 저수준 read/write
│   ├── control_table.py    # AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기)
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
├── perception/          # AI 인지 기능 (하드웨어 불필요, 카메라 프레임만 입력)
│   ├── hand_tracker.py     # HandTracker — mediapipe 손 인식 + 3D 월드 좌표 변환
│   └── document_scanner.py # DocumentScanner — 문서 사각형 검출/원근 보정 (OCR 없음)
└── src/                 # Flask 제어 대시보드 + 모바일 카메라/음성/문서 스캔 (하드웨어 없어도 실행됨)
    ├── app.py
    ├── api/
    │   ├── gemini.py       # Gemini — 채팅/요약/STT/문서 읽기
    │   ├── camera.py       # Camera — /ws/camera 프레임+음성 클립 수신
    │   └── scan.py         # ScanAPI — 문서 스캔 3라우트 (DocumentScanner + Gemini)
    └── templates/
        ├── index.html
        └── mobile.html
```

| 파일 | 역할 |
|------|------|
| [main.py](main.py) | 진입점 — `src.app.run()` 호출(`python main.py` → `https://localhost:8000`). 하드웨어 없어도 실행됨 |
| [logger.py](logger.py) | `Logger` — 모든 모듈이 공유하는 `[TAG] message` 콘솔 로거 |
| [hardware/controller.py](hardware/controller.py) | 시리얼 포트/보드레이트 초기화, 저수준 read/write (`set_speed`, `set_goal_position`, `get_present_position`) |
| [hardware/control_table.py](hardware/control_table.py) | AX-18A 컨트롤 테이블(레지스터 주소·바이트 크기)을 클래스로 정의 |
| [hardware/actuator.py](hardware/actuator.py) | `Actuator`(모터 1개 추상화) + `ArmController`(FK/IK ↔ Actuator 연동) |
| [kinematics/kinematics.py](kinematics/kinematics.py) | FK/IK. `Arm`/`Joint` + 순수 파이썬 선형대수. 하드웨어 불필요 |
| [kinematics/urdf_loader.py](kinematics/urdf_loader.py) | `configure/desky.urdf`를 파싱해 `Arm` 생성 (`xml.etree`) |
| [kinematics/simulate.py](kinematics/simulate.py) | 인터랙티브 FK 슬라이더 + IK 입력 3D 미리보기 (matplotlib). `src/app.py`의 `/api/render`도 여기 `draw_pose`/`draw_points`를 그대로 재사용 |
| [kinematics/mujoco_sim.py](kinematics/mujoco_sim.py) | MuJoCo로 실제 메시 기반 시뮬레이션 (선택 기능) |
| [kinematics/find_joint_limits.py](kinematics/find_joint_limits.py) | MuJoCo로 자기충돌이 시작되는 지점까지 관절 한계를 스윕해 `configure/joints.json`을 만드는 스크립트 |
| [kinematics/configure/desky.urdf](kinematics/configure/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |
| [perception/hand_tracker.py](perception/hand_tracker.py) | `HandTracker` — mediapipe로 손을 인식하고, end-effector의 FK 변환행렬을 이용해 손 랜드마크를 3D 월드 좌표로 올린다. mediapipe 미설치 시 `available=False`로 조용히 비활성 |
| [perception/document_scanner.py](perception/document_scanner.py) | `DocumentScanner` — 카메라 프레임에서 종이 문서 사각형을 검출하고 원근 보정(warp)해서 잘라낸다. OCR은 하지 않음(그 결과를 `Gemini.parse_document()`에 넘김) |
| [src/app.py](src/app.py) | Flask 앱 + `run()` — 위치/관절각 제어 대시보드(`/`), 모바일 카메라 웹소켓(`/ws/camera`, `/mobile`), Gemini 채팅/요약/문서스캔 API, 서버 로컬 cv2 미리보기 창(손 인식 오버레이 포함) |
| [src/api/gemini.py](src/api/gemini.py) | `Gemini` — `/api/ask`(채팅/요약), 음성 클립 받아쓰기(STT), 문서 이미지 글자 읽기(`parse_document`) |
| [src/api/camera.py](src/api/camera.py) | `Camera` — `/ws/camera`로 들어오는 JPEG 프레임 + WebM 음성 클립을 매직 바이트로 구분해 처리 |
| [src/api/scan.py](src/api/scan.py) | `ScanAPI` — `/api/scan/preview.jpg`, `/api/scan/detect`, `/api/scan/parse` 3라우트. `DocumentScanner` + `Gemini`를 연결 |
| [src/templates/index.html](src/templates/index.html) | 대시보드 UI (위치 입력, 관절별 입력, 상태, 카메라 프리뷰) |
| [src/templates/mobile.html](src/templates/mobile.html) | 팔에 장착된 휴대폰에서 여는 페이지 — 카메라 스트리밍, Gemini 채팅/요약, 음성 대화(연속 STT+TTS), 음성 명령, 문서 스캔 UI |

`hardware/`, `kinematics/`, `perception/`, `src/` 모두 패키지(`__init__.py` 포함)이며,
내부 모듈 간 임포트는 상대 임포트(`.control_table`, `.kinematics`)를 쓴다. `logger.py`는
루트에 있으므로 어디서든 `from logger import Logger`로 절대 임포트한다. 항상 **저장소
루트에서** 실행할 것(`python main.py`, `python -m kinematics.kinematics`,
`python -m kinematics.simulate`, `python -m src.app` 등) — 하위 폴더의 파일을 직접
실행하면(`python kinematics/kinematics.py`) 루트가 `sys.path`에 없어 `logger` 임포트가
깨진다.

### 계층
```
main.py
  └─ src.app (Flask) ── hardware.actuator.ArmController ── hardware.actuator.Actuator ×5 ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
       │                                       └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
       ├─ src.api.gemini.Gemini ── (google-genai)
       ├─ src.api.camera.Camera ── src.api.gemini.Gemini (음성 클립 STT)
       ├─ src.api.scan.ScanAPI ── perception.document_scanner.DocumentScanner + src.api.gemini.Gemini
       └─ perception.hand_tracker.HandTracker (run()의 로컬 cv2 미리보기 루프 전용, 라우트 아님)
```
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

1. "Start camera + streaming" 버튼을 누르면 `getUserMedia({video: {facingMode: "environment"},
   audio: false})`로 후면 카메라 권한만 요청한다(음성 대화는 브라우저 내장
   `SpeechRecognition`이 별도로 마이크 권한을 요청 — 아래 음성 대화 절 참고).
2. 승인되면 `<video>`로 로컬 프리뷰를 띄우고, 200ms(≈5fps)마다 `<canvas>`에 프레임을 그려
   `canvas.toBlob('image/jpeg')`로 JPEG를 만들어 `wss://<host>/ws/camera`로 바이너리 그대로
   전송한다. 프레임마다 완결된 JPEG라서 서버는 컨테이너/코덱을 신경 쓸 필요가 없다.

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

### 손 인식 (`perception/hand_tracker.py`, `src.app.run()`)

`src.app.run()`의 로컬 cv2 미리보기 루프(아래 절)가 매 프레임마다 `HandTracker.process()`를
호출해 mediapipe로 손을 인식한다. 휴대폰 카메라가 end-effector에 달려 있다는 전제 아래,
손목~엄지CMC의 실제 길이를 3.5cm로 가정하고 화면에 보이는 크기로부터 핀홀 투영을 역산해
카메라까지 거리를 추정한다(`estimate_depth`). 그 거리와 end-effector의 FK 변환행렬
(`Arm.fk_matrix(q)`)로 손 랜드마크를 월드 좌표에 올린다(`landmark_to_world`). **캘리브레이션한
값이 아니므로 거리는 측정이 아니라 추정치다.** mediapipe가 설치돼 있지 않으면
`HandTracker.available`이 `False`가 되어 카메라 창은 뜨되 손 오버레이만 빠진다(하드웨어/Gemini
미구성과 같은 부분 실패 패턴).

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

### 서버 로컬 카메라 미리보기 창 (`src.app.run`)

`python main.py`와 `python -m src.app` 둘 다 `src/app.py`의 `run()` 함수 하나를 호출한다
(중복 구현 방지). `run()`이 하는 일:

1. Flask 서버(`app.run(...)`)를 **백그라운드 스레드**로 띄운다 — `app.run()`은 영원히 블록되는
   호출이라, 메인 스레드에서 그대로 부르면 그 뒤 코드가 절대 실행되지 않는다.
2. 메인 스레드에서 `/ws/camera`로 들어온 최신 프레임을 디코드하고, `HandTracker.process()`로
   손을 인식해 골격을 오버레이한 뒤 `cv2.imshow("Mobile camera", ...)`로 서버가 실행 중인
   컴퓨터 화면에 띄운다. 같은 프레임의 현재 관절각(FK)과 인식된 손의 월드 좌표로 "3D scene
   (robot + hand)" 창도 함께 갱신한다. **cv2 창 관련 호출은 반드시 메인 스레드에서 실행해야
   한다** — macOS에서는 OpenCV HighGUI가 메인 스레드가 아니면 `imshow`/`waitKey`를 조용히
   무시한다.
3. `cv2.waitKey(1)`로 HighGUI 이벤트 루프를 돌리면서(이게 없으면 창이 아예 갱신되지 않는다)
   `'q'`/Esc 입력 시 종료한다. `frame_count`가 실제로 바뀐 경우에만 디코드/표시하도록
   체크한다 — `waitKey(1)`은 "최대 1ms"일 뿐 보장이 아니라서, 이 체크가 없으면 휴대폰이 보내는
   실제 프레임(약 5fps)보다 훨씬 빠르게 같은 프레임을 반복 디코드/표시하게 된다.

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

## 주의사항 / 알려진 특성

- `hardware/controller.py`의 read/write는 통신·하드웨어 에러 시 **예외로 죽지 않고** 경고
  출력 후 계속 진행. `set_*`는 성공 여부 `bool`, `get_present_position`은 실패 시 `None`
  반환 → **호출부에서 `None` 처리 필요**.
- `Controller`는 `with Controller() as c:` 컨텍스트 매니저 지원 (GC 대신 결정적 포트 정리).
- 각도↔유닛 변환은 `/360` 계수를 쓴다(범위 리미트로 의도적 처리됨 — 유지할 것).
- XML 주석 안에 `--`(이중 하이픈) 금지 (URDF 편집 시 파싱 에러 주의).
- 모든 모듈의 콘솔 출력은 `logger.Logger.log(tag, message)`를 거쳐 `[TAG] message` 형식으로
  통일된다 (`CONTROLLER`, `ACTUATOR`, `WEBAPP`, `GEMINI`, `STT`, `SCAN`, `DOCSCAN`, `HAND`,
  `CAMERA`, `KINEMATICS`, `URDF` 태그). `Logger.enabled` 플래그 하나로 전체 로그를 끌 수 있다.
- `HandTracker`/`Gemini`/`DocumentScanner`는 모두 하드웨어와 같은 "부분 실패" 패턴을 따른다:
  생성자에서 의존성 실패를 흡수하고 `available`/`configured` 플래그 + `error` 문자열만
  남긴다. 새 AI 기능을 추가할 때도 이 패턴을 따를 것 — 앱 전체가 죽으면 안 된다.

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
