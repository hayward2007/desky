# desky

책상 위에 놓는 5자유도(5-DOF) 로봇팔 프로젝트. End-effector에 **휴대폰을 장착**하는 것이 목표이며,
DYNAMIXEL AX-18A 서보 5개를 시리얼로 제어하고 URDF 기반 순기구학(FK)/역기구학(IK)을 제공한다.
휴대폰 카메라를 통해 **손 인식**(mediapipe), **문서 스캔/요약**, **음성 대화**(Gemini)도 지원한다.

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

- Python 3.x (기구학 모듈은 표준 라이브러리 위주라 버전에 크게 구애받지 않음)
- 의존성은 [requirements.txt](requirements.txt)에 정리돼 있다:
  - `dynamixel_sdk`, `python-dotenv` — 하드웨어 제어 시에만 필요
  - `matplotlib` — [kinematics/simulate.py](kinematics/simulate.py) 3D 프리뷰 전용
  - `flask`/`flask-sock`/`pyopenssl` — [src/app.py](src/app.py) 제어 대시보드 + 모바일 카메라 스트리밍
  - `opencv-python`/`numpy` — 문서 검출, 카메라 미리보기 창
  - `google-genai` — Gemini 채팅/요약/STT/문서 읽기 (`/api/ask`, `/api/scan/parse`)
  - `mediapipe` — [perception/hand_tracker.py](perception/hand_tracker.py) 손 인식
- 기구학 패키지(`kinematics/kinematics.py`, `urdf_loader.py`)는 **표준 라이브러리만** 사용 — numpy 등 불필요

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

각 기능 객체(하드웨어, Gemini, mediapipe)는 자기 의존성이 없어도 예외 없이 만들어지고
"그 기능만 비활성" 상태가 된다 — 위 의존성을 전부 설치하지 않아도 앱 자체는 항상 뜬다.

### 환경 변수

프로젝트 루트에 `.env` 파일이 필요하다.

```env
# 하드웨어 (실제 서보 제어 시에만 필요)
DEVICE_NAME=tty01        # /dev/ 접두어는 제외하고 작성
BAUDRATE=1000000
PROTOCOL_VERSION=1.0

# Gemini (채팅/요약/STT/문서 읽기 — 없으면 해당 기능만 비활성)
GEMINI_API_KEY=
```

## 파일 구조

하드웨어 제어(`hardware/`), 기구학+3D 프리뷰(`kinematics/`), AI 인지 기능(`perception/`),
웹 제어 대시보드(`src/`)를 별도 패키지로 분리했다. 서로 독립적이며, `logger.py`만 전체가
공통으로 참조하는 루트 모듈이다.

```text
desky/
├── main.py             # 진입점 — src.app.run() 실행. 하드웨어 유무 무관
├── logger.py           # 공통 [TAG] message 콘솔 로거
├── requirements.txt    # 전체 의존성
├── hardware/           # 시리얼/서보 제어 (실제 하드웨어 필요)
│   ├── controller.py
│   ├── control_table.py
│   └── actuator.py
├── kinematics/         # FK/IK + 3D 프리뷰 (하드웨어 불필요)
│   ├── kinematics.py
│   ├── urdf_loader.py
│   ├── simulate.py
│   ├── mujoco_sim.py
│   ├── find_joint_limits.py
│   ├── configure/
│   │   ├── desky.urdf
│   │   ├── joints.json
│   │   └── manifest.json
│   └── meshes/*.stl
├── perception/         # AI 인지 기능 (하드웨어 불필요, 카메라 프레임만 입력)
│   ├── hand_tracker.py
│   └── document_scanner.py
└── src/                # Flask 제어 대시보드 + 모바일 카메라/음성/문서 스캔
    ├── app.py
    ├── api/
    │   ├── gemini.py
    │   ├── camera.py
    │   └── scan.py
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
| [kinematics/simulate.py](kinematics/simulate.py) | 인터랙티브 FK 슬라이더 + IK 입력 3D 미리보기 (matplotlib) |
| [kinematics/configure/desky.urdf](kinematics/configure/desky.urdf) | **로봇 구성의 단일 소스 오브 트루스** (URDF, XML) |
| [perception/hand_tracker.py](perception/hand_tracker.py) | `HandTracker` — mediapipe 손 인식 + end-effector FK 기반 3D 월드 좌표 변환 |
| [perception/document_scanner.py](perception/document_scanner.py) | `DocumentScanner` — 카메라 프레임에서 문서 사각형 검출/원근 보정 (OCR 없음) |
| [src/app.py](src/app.py) | Flask 앱 + `run()` — 위치/관절각 제어 대시보드(`/`), 모바일 카메라 웹소켓(`/ws/camera`, `/mobile`), Gemini/문서스캔 API, 서버 로컬 cv2 미리보기 창(손 인식 오버레이 포함) |
| [src/api/gemini.py](src/api/gemini.py) | `Gemini` — 채팅/요약(`/api/ask`), STT, 문서 이미지 글자 읽기 |
| [src/api/camera.py](src/api/camera.py) | `Camera` — `/ws/camera` 프레임/음성 클립 수신 + 관련 라우트 |
| [src/api/scan.py](src/api/scan.py) | `ScanAPI` — 문서 스캔 3라우트(`/api/scan/preview.jpg`, `/detect`, `/parse`) |
| [src/templates/index.html](src/templates/index.html) | 대시보드 UI (위치/관절 입력, 상태, 카메라 프리뷰) |
| [src/templates/mobile.html](src/templates/mobile.html) | 팔에 장착된 휴대폰에서 여는 페이지 — 카메라 스트리밍, Gemini 채팅/요약, 음성 대화·명령, 문서 스캔 UI |

> **항상 저장소 루트에서 실행할 것** (`python main.py`, `python -m kinematics.kinematics`,
> `python -m kinematics.simulate`, `python -m src.app` 등). 하위 폴더의 파일을 직접
> 실행하면(`python kinematics/kinematics.py`) 루트가 `sys.path`에 없어 `logger` 임포트가
> 깨진다.

### 계층 구조

```text
main.py
  └─ src.app (Flask) ── hardware.actuator.ArmController ── hardware.actuator.Actuator ×5 ── hardware.controller.Controller ── hardware.control_table.AX_18A + dynamixel_sdk
       │                                       └─ kinematics.kinematics.Arm ◄── kinematics.urdf_loader.load_arm()
       ├─ src.api.gemini.Gemini
       ├─ src.api.camera.Camera ── src.api.gemini.Gemini (음성 클립 STT)
       ├─ src.api.scan.ScanAPI ── perception.document_scanner.DocumentScanner + src.api.gemini.Gemini
       └─ perception.hand_tracker.HandTracker (로컬 cv2 미리보기 전용)
```

`main.py`는 `src.app`을 임포트해 실행하는 얇은 launcher다 — 실제 초기화(하드웨어, Gemini)는
`src/app.py` 쪽에서 일어난다 (하드웨어/Gemini가 없으면 실패를 잡아 "미연결/미구성" 상태로
대체 — 아래 절 참고).

`ArmController`가 FK/IK(`kinematics.Arm`)와 실제 `Actuator` 5개를 연결한다. `Actuator.id`
(DYNAMIXEL id)와 URDF `<dynamixel id="">`가 일치하는 조인트끼리 매칭하므로, 생성자에 넘기는
`actuators` 리스트는 순서에 상관없다.

```python
from hardware.controller import Controller
from hardware.actuator import Actuator, ArmController

controller = Controller()
actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
arm_ctrl = ArmController(actuators)                  # arm=load_arm() 기본값 사용

q, ok = arm_ctrl.goto_position((0.3, 0.05, 0.15))    # IK 계산 → 5개 서보에 goto() 디스패치
pos = arm_ctrl.get_position()                         # 5개 서보 현재각 read → FK로 위치 계산
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

- 관절별 서보각 슬라이더 → FK 미리보기가 실시간으로 갱신, "Move actuators"로 실제 이동
- 목표 위치 (x, y, z, 미터) 입력 → IK 계산, 슬라이더가 해로 스냅
- 1.5초마다 현재 위치를 폴링해 표시
- 팔에 장착된 휴대폰이 `/mobile`에서 스트리밍 중이면 카메라 프리뷰도 함께 표시(아래 참고)

라우트: `GET /`(대시보드), `GET /mobile`, `GET /api/status`, `POST /api/fk`, `POST /api/ik`,
`POST /api/render`, `POST /api/goto_position`, `POST /api/goto_joints`, `POST /api/goto_joint`,
`WS /ws/camera`, `GET /api/camera/latest.jpg`, `GET /api/camera/status`, `POST /api/ask`,
`GET /api/scan/preview.jpg`, `GET /api/scan/detect`, `POST /api/scan/parse`.

> **참고:** `app.run(..., port=8000, debug=False, threaded=True, ssl_context="adhoc")`로
> 고정되어 있다.
>
> - 포트 5000이 아니라 8000을 쓰는 이유는 macOS AirPlay Receiver가 5000번을 기본으로 점유하기
>   때문.
> - `debug=False`인 이유는 Flask 리로더가 모듈을 다시 임포트하면 시리얼 포트가 두 번 열리기
>   때문 — `debug=True`로 바꾸지 말 것.
> - `threaded=True`인 이유는 `/ws/camera` 웹소켓이 세션 내내 연결을 유지해서, 스레드 하나로만
>   서빙하면 다른 HTTP 요청이 막히기 때문.
> - `ssl_context="adhoc"`인 이유는 아래 모바일 카메라 스트리밍 절 참고.

### 모바일 카메라 스트리밍 (`/mobile`, `/ws/camera`)

팔 끝(end-effector)에 장착된 휴대폰에서 `https://<이 서버의 LAN IP>:8000/mobile`을 열면:

1. "Start camera + streaming" 버튼을 누르면 `getUserMedia({video: {facingMode: {exact: "user"}},
   audio: false})`로 전면 카메라 권한을 요청한다(마운트 구조상 전면 카메라만 써야 하므로
   `exact`로 강제 — 전면 카메라가 없으면 후면으로 조용히 넘어가지 않고 바로 에러가 난다. 음성
   대화는 브라우저 내장 STT가 별도로 마이크 권한을 요청한다).
2. 승인되면 `<video>`로 로컬 프리뷰를 띄우고, 200ms(약 5fps)마다 `<canvas>`에 프레임을 그려
   `canvas.toBlob('image/jpeg')`로 JPEG를 만들어 `wss://<host>/ws/camera`로 바이너리 그대로
   전송한다. 프레임마다 완결된 JPEG라서 서버가 컨테이너/코덱을 재조립할 필요가 없다.

서버([src/api/camera.py](src/api/camera.py))는 `flask_sock.Sock`으로 `/ws/camera`를 raw
WebSocket으로 처리한다:

- 들어오는 바이너리가 JPEG면 최신 프레임으로 저장(락으로 보호), 프레임 수·마지막 수신 시각·
  연결된 클라이언트 수도 함께 추적. WebM/Opus 음성 클립이면 Gemini로 받아쓴다(레거시 경로 —
  현재 UI는 브라우저 내장 STT를 쓰므로 기본적으로는 이 경로를 타지 않는다).
- `GET /api/camera/latest.jpg` — 최신 프레임을 `image/jpeg`로 반환(아직 없으면 404).
- `GET /api/camera/status` — `{streaming, clients, frame_count, age_seconds}`.
- 대시보드(`/`)는 `/api/camera/status`를 0.5초마다 폴링하고, 스트리밍 중이면 이미지 태그를
  갱신해 실시간처럼 보여준다(진짜 스트리밍이 아니라 폴링 기반 프리뷰 — 데모 목적으로는 충분).

**HTTPS가 필요한 이유:** 브라우저는 secure context(HTTPS 또는 `localhost`)에서만
`getUserMedia`/음성 인식을 허용한다. 휴대폰은 LAN IP로 접속하므로 `localhost`가 아니어서
HTTPS가 필수다. `ssl_context="adhoc"`(pyOpenSSL)으로 실행할 때마다 자체 서명 인증서를 즉석에서
만들어 쓰므로 브라우저가 "안전하지 않음" 경고를 한 번 띄운다 — 데스크톱 대시보드, 모바일
페이지 둘 다에서 "고급 → 계속 진행"으로 넘어가야 한다.

### 손 인식 (`perception/hand_tracker.py`)

`src.app.run()`의 로컬 미리보기 창이 매 프레임 mediapipe로 손을 인식해 카메라 창에 골격을
오버레이하고, 3D 씬 창에도 손의 추정 월드 좌표를 함께 그린다. 휴대폰 카메라가 end-effector에
달려 있다는 전제로, 손목~엄지CMC 길이(3.5cm 가정)와 화면상 크기로 카메라까지 거리를
역산(핀홀 모델)한 뒤 현재 end-effector의 FK 변환행렬로 손 랜드마크를 월드 좌표에 올린다. 실제
캘리브레이션한 값이 아니므로 거리는 추정치다. mediapipe가 없으면 손 오버레이만 빠지고 카메라
창은 그대로 뜬다.

### Gemini 채팅/요약/STT/문서 읽기 (`/mobile`, `POST /api/ask`)

[src/templates/mobile.html](src/templates/mobile.html)에 "Ask Gemini" 섹션이 있다
(대시보드 `/`에는 없음) — 텍스트를 입력하고 Chat/Summarize 모드를 골라 `POST /api/ask`로
보낸다. [src/app.py](src/app.py)는 하드웨어와 똑같은 패턴으로 시작 시 Gemini 클라이언트 구성을
시도하고, 실패해도 앱은 계속 뜬다:

- `GEMINI_API_KEY` 환경 변수(`.env`에 넣으면 `python-dotenv`가 있을 때 자동 로드됨)와
  `google-genai` 패키지가 있어야 `Gemini.configured`가 `True`가 된다.
- 실패하면(키 없음, 패키지 미설치 등) `Gemini.error`에 원인 저장 — `/mobile`은 "⚠ Gemini not
  configured — <원인>" 배너를 띄우고 입력/버튼을 비활성화한다.
- 답은 음성으로 그대로 읽히는 것을 전제로 짧고(100자 이내), 목록·기호·마크다운 없이 말하듯
  쓰도록 지시돼 있다.
- `POST /api/ask` — 바디 `{"text": "...", "mode": "chat"|"summary"}`. `mode`에 따라 다른
  system instruction을 사용해 Gemini를 호출한다.
  - 미구성 시 `503 {"error": "Gemini not configured: ..."}`.
  - `text` 누락 시 `400 {"error": "text is required"}`.
  - Gemini 호출 자체가 실패하면 `502 {"error": "..."}`.
  - 성공 시 `200 {"answer": "..."}`.

### 문서 스캔 · 요약 (`/mobile`, `/api/scan/*`)

`/mobile`의 "문서 스캔" 섹션은 카메라로 종이 문서를 찾아 Gemini에게 읽힌다:

1. 실시간 미리보기(`GET /api/scan/preview.jpg`)가 감지된 문서를 초록 사각형 + 번호로 보여준다.
2. "스캔하기"(`GET /api/scan/detect`)를 누르면 그 순간의 프레임/좌표를 고정해 선택 화면을
   띄운다 — 사각형을 눌러 문서를 선택한다.
3. "선택 문서 파싱"/"전체 파싱"(`POST /api/scan/parse`)이 선택 영역을 원근 보정해 잘라
   Gemini에게 글자를 읽히거나(mode="text") 짧게 요약해 읽어준다(mode="summary").

문서 검출 자체(`perception/document_scanner.py`)는 OpenCV만으로 하는 사각형 검출이며 OCR은
하지 않는다 — 실제 글자 인식은 잘라낸 이미지를 Gemini에 넘겨 처리한다.

### 음성 대화 · 음성 명령 (`/mobile`)

마이크 버튼("🎙 대화 시작")을 누르면 브라우저 내장 `SpeechRecognition`으로 연속 음성 인식을
시작한다(서버로 오디오를 보내지 않음). 5초간 말이 없으면 그때까지 들은 문장을 확정해:

- 알려진 명령("스캔해줘", "카메라 시작", "요약 모드로 바꿔", "N번 읽어줘", "대화 종료" 등)과
  일치하면 Gemini에 묻지 않고 그 기능을 바로 실행한다.
- 명령이 아니면 `/api/ask`로 Gemini에게 묻고, 답을 브라우저 TTS(`speechSynthesis`,
  한국어)로 읽어준 뒤 자동으로 다시 듣기 상태로 돌아간다 — 손을 쓰지 않고 계속 대화할 수 있다.

### 서버 로컬 카메라 미리보기 창 (`src.app.run`)

`python main.py`와 `python -m src.app` 둘 다 [src/app.py](src/app.py)의 `run()` 함수 하나를
호출한다(중복 구현 방지). `run()`이 하는 일:

1. Flask 서버(`app.run(...)`)를 **백그라운드 스레드**로 띄운다 — `app.run()`은 영원히 블록되는
   호출이라, 메인 스레드에서 그대로 부르면 그 뒤 코드가 절대 실행되지 않는다.
2. 메인 스레드에서 `/ws/camera`로 들어온 최신 프레임을 손 인식 오버레이와 함께
   `cv2.imshow("Mobile camera", ...)`로, 로봇팔 3D 자세 + 인식된 손 위치를 "3D scene
   (robot + hand)" 창으로 서버가 실행 중인 컴퓨터 화면에 띄운다. **cv2 창 관련 호출은 반드시
   메인 스레드에서 실행해야 한다** — macOS에서는 OpenCV HighGUI가 메인 스레드가 아니면
   `imshow`/`waitKey`를 조용히 무시한다.
3. `cv2.waitKey(1)`로 HighGUI 이벤트 루프를 돌리면서(이게 없으면 창이 아예 갱신되지 않는다)
   `'q'`/Esc 입력 시 종료한다. `frame_count`가 실제로 바뀐 경우에만 디코드/표시하도록 체크한다
   — `waitKey(1)`은 "최대 1ms"일 뿐 보장이 아니라서, 이 체크가 없으면 휴대폰이 보내는 실제
   프레임(약 5fps)보다 훨씬 빠르게 같은 프레임을 반복 디코드/표시하게 된다.

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

## 3D 시뮬레이션 (하드웨어 없이 미리보기)

```bash
python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt   # 최초 1회
python -m kinematics.simulate   # 저장소 루트에서 실행
```

[kinematics/simulate.py](kinematics/simulate.py)는 `kinematics.urdf_loader.load_arm()`으로
로봇 형상([kinematics/configure/desky.urdf](kinematics/configure/desky.urdf))을 읽어 인터랙티브
3D 창을 띄운다. 관절마다 서보각 슬라이더가 있어 움직이면 FK로 자세가 즉시 갱신되고, x/y/z를
입력해 IK를 풀면 슬라이더가 해로 스냅된다. 웹 대시보드의 `/api/render`도 이 파일의
`draw_pose`/`workspace_bounds`를 그대로 재사용한다.

> **참고:** `kinematics/simulate.py`는 서보각을 URDF의 `<limit>`(IK용 소프트 리미트)로
> 클램프하지 않는다 — 실제 `Actuator.goto()`도 물리 범위(0~300°) 안이면 그 리미트를 무시하고
> 그대로 움직이므로, 시뮬레이션도 동일하게 동작해야 "실제 로봇과 정확히 같은" 모션이 된다.

메시 기반(정확한 형상) 시뮬레이션이 필요하면 `python -m kinematics.mujoco_sim`(MuJoCo 필요,
선택 의존성).

## 기구학 사용법

```python
from kinematics.urdf_loader import load_arm
arm = load_arm()                        # kinematics/configure/desky.urdf에서 구성 로드 (권장)
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
python -m kinematics.kinematics     # FK/IK 라운드트립 self-test
python -m kinematics.urdf_loader    # URDF 로드 + FK/IK 라운드트립
```

기대 결과: `converged=True`, 위치 오차 < 1mm.

## 로봇 구성 수정 방법

로봇 치수·서보 캘리브레이션을 바꿀 때는 코드가 아니라
**[kinematics/configure/desky.urdf](kinematics/configure/desky.urdf) 하나만 수정**한다.
`urdf_loader`가 이를 읽어 `Arm`을 구성한다.

- 링크 길이 → 각 `<joint>`의 `<origin xyz>` (URDF 관례상 **미터** 단위)
- 관절 한계 → `<limit lower upper>` (라디안)
- 축 → `<axis xyz>`
- DYNAMIXEL id/모델/서보 캘리브레이션 → 프로젝트 확장 태그
  `<dynamixel id="" model="" home_deg="" direction="">` (표준 URDF 툴은 무시하고, `urdf_loader`만 읽음)

## 주의사항 / 알려진 특성

- `hardware/controller.py`의 read/write는 통신·하드웨어 에러가 나도 **예외로 죽지 않고**
  경고를 출력한 뒤 계속 진행한다. `set_*` 계열은 성공 여부를 `bool`로, `get_present_position`은
  실패 시 `None`을 반환하므로 **호출부에서 반드시 `None`을 처리**해야 한다.
- `Controller`는 `with Controller() as c:` 형태의 컨텍스트 매니저를 지원한다 (GC에 맡기지
  않고 포트를 결정적으로 정리).
- 각도 ↔ 유닛 변환은 `/360` 계수를 사용한다 (물리 범위 0~300°는 리미트로 의도적으로 처리됨 —
  변경하지 말 것).
- URDF를 직접 편집할 때 XML 주석 안에 `--`(이중 하이픈)를 넣으면 파싱 에러가 나므로 주의.
- `HandTracker`/`Gemini`/`DocumentScanner`는 하드웨어와 같은 "부분 실패" 패턴을 따른다 —
  의존성이 없어도 앱 전체가 죽지 않고 그 기능만 비활성화된다.
- 모든 모듈의 콘솔 출력은 `logger.Logger.log(tag, message)`를 거쳐 `[TAG] message` 형식으로
  통일된다. `Logger.enabled` 플래그 하나로 전체 로그를 끌 수 있다.
