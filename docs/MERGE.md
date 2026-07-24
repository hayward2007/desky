# 병합 기록 — 두 브랜치를 하나로

## 0. 먼저 확인한 것: 둘은 별개 프로그램이 아니다

받은 두 폴더는 **같은 `desky` 저장소의 서로 다른 두 브랜치**였다.

| | `desky-develop` (이하 **추적**) | `desky-new_mobile_1017ver` (이하 **웹**) |
|---|---|---|
| 성격 | 팔 추적 알고리즘 계열 | 웹 기능 계열 |
| 고유 파일 | `perception/face_tracker.py`, `perception/follow_controller.py`, `perception/camera_geometry.py`, `fundamental/const.py`, `fundamental/logger.py` | `src/api/calendar.py`, `src/api/light.py`, `perception/gesture.py`, `raspberry-server/`, `hardware/control_table.py`, `logger.py` |
| 공통 파일 상태 | 상수·로거를 `fundamental/`로 뽑아낸 **리팩터링 완료본** | 상수가 각 파일에 하드코딩된 **리팩터링 이전본** |

기구학(`kinematics/`)·하드웨어(`hardware/`) 파일들의 차이를 전부 확인한 결과,
**알고리즘 차이는 없고 리팩터링 차이뿐**이었다. 예를 들어 두 브랜치의
`kinematics/kinematics.py` 차이는 아래가 전부다.

```diff
- BASE_HEIGHT_MM = KinematicsConst.BASE_HEIGHT_MM   # 추적 브랜치
+ BASE_HEIGHT_MM = 50.0                             # 웹 브랜치
```

그래서 병합 전략을 이렇게 잡았다:

> **추적 브랜치를 뼈대로 삼고, 웹 브랜치의 고유 기능만 그 위에 이식한다.**
> 이식할 때는 하드코딩된 상수를 `fundamental/const.py`로 옮겨 뼈대의 관례에 맞춘다.

---

## 1. 파일별 채택 내역

| 대상 | 채택 | 이유 |
|------|------|------|
| `kinematics/*`, `hardware/controller.py`, `hardware/actuator.py` | **추적** 그대로 | 상수 외부화가 끝난 버전. 웹 브랜치엔 없는 `Actuator.goto` 속도 재기록 생략 최적화도 포함 |
| `hardware/control_table.py` | **삭제**(추적 방식) | 이미 `fundamental/const.py`의 `ActuatorControlTable`/`AX_18A`로 옮겨져 있었다 |
| `logger.py` (루트) | **삭제**, `fundamental/logger.py`로 통일 | 두 위치에 같은 클래스가 있으면 import 경로가 갈린다 |
| `perception/face_tracker.py`, `follow_controller.py`, `camera_geometry.py`, `hand_tracker.py` | **추적** 그대로 | 웹 브랜치에 없는 기능 |
| `perception/gesture.py` | **웹**에서 이식 | 아래 §2.1 |
| `src/api/calendar.py`, `light.py` | **웹**에서 이식 | 로거 경로 + 상수 외부화만 수정 |
| `src/api/camera.py` | **두 브랜치 합침** | 아래 §2.2 |
| `src/app.py` | **전면 재구성** | 아래 §2.3 |
| `src/templates/mobile.html` | **두 브랜치 합침** | 아래 §2.4 |
| `src/templates/index.html` | 동일 — 그대로 | 두 브랜치 간 차이 없음 |
| `raspberry-server/` | **웹**에서 그대로 복사 | 파이에서 따로 도는 독립 서버라 이 앱 코드와 얽히지 않는다 |

---

## 2. 충돌과 해결

### 2.1 손 인식이 두 곳에서 돌던 문제 ★ 가장 큰 충돌

**상태**

- 추적 브랜치: 손 인식을 **휴대폰**이 한다(MediaPipe Tasks Vision). 서버는 얼굴만 인식.
  이렇게 나눈 이유가 코드에 남아 있다 — 컴퓨터 하나가 얼굴+손 추론에 3D 렌더링,
  하드웨어 제어까지 매 프레임 처리하니 폰이 보내는 ~20fps를 못 따라갔다.
- 웹 브랜치: 손 인식을 **서버**의 `mediapipe.solutions.hands`로 직접 하고,
  그 결과로 가위바위보를 판정했다.

그대로 합치면 손 인식이 두 번 돌아 **서로 다른 두 개의 손 목록**이 생기고,
서버 쪽 mediapipe 비용 때문에 추적 브랜치가 해결해 둔 프레임레이트 문제가 되돌아온다.

**해결 — 인식 경로를 하나로 통일하고, 결과를 두 소비자가 나눠 쓴다**

```text
휴대폰(MediaPipe Tasks Vision)
   └─ /ws/camera ─▶ Camera.hand_landmarks
        └─ HandTracker.process_landmarks()   ← 좌표 변환만(모델 추론 아님)
             ├─▶ FollowController   팔이 손을 따라감
             └─▶ GestureBridge      가위바위보 판정
```

이게 가능했던 이유: 가위바위보 판정에 필요한 건 랜드마크 21개의 `x`/`y`뿐이고,
그 21개는 서버 mediapipe든 휴대폰 Tasks Vision이든 **같은 손 모델의 같은 인덱스**다.
`gesture.py`의 `_landmarks_of()`가 `Hand` 객체·mediapipe 객체·순수 리스트를 모두
받아내도록 되어 있어서, 판정 로직 자체는 한 줄도 고치지 않았다.

**부수 변경**: `mobile.html`의 `HAND_TRACKING_ENABLED`를 `false` → `true`로 켰다.
추적 브랜치는 실험을 위해 꺼 둔 상태였는데, 병합 후에는 이 스위치가 손 추종과
제스처를 **함께** 좌우한다(끄면 둘 다 비활성, 얼굴 추적·idle 두리번거림·나머지 웹
기능은 그대로).

### 2.2 `src/api/camera.py` — 소켓이 단방향 vs 양방향

- 추적 브랜치: 폰 → 서버 방향에 **텍스트 메시지(손 랜드마크 JSON)** 처리를 추가.
- 웹 브랜치: 서버 → 폰 방향으로 명령을 밀어 넣는 **`broadcast()` + 소켓 목록 + 전송 락**을 추가.

둘은 서로 다른 방향이라 기능상 충돌이 아니었다. 양쪽을 모두 넣고, 전송 경로만
`_send()` 하나로 모았다 — 전송 락을 한 곳에서만 잡게 되고, 끊긴 소켓 정리도
한 곳에서 처리된다.

> 락을 두 개(`lock`, `send_lock`) 쓰는 이유는 웹 브랜치 주석 그대로다: 전송은
> 네트워크 대기가 있어 오래 걸릴 수 있는데, 그동안 `snapshot()` 같은 짧은 상태
> 읽기까지 막으면 미리보기 루프가 통째로 멈춘다.

### 2.3 `src/app.py` — 텍스트로는 합칠 수 없던 파일 ★ 객체화

두 브랜치 모두 이 자리에 500줄이 넘는 **모듈 전역 스크립트**를 두고 있었다.
전역 `app`, `arm_ctrl`, `arm`, `gemini`, `camera`, `_q_cache`와
`@app.route`가 붙은 전역 함수들, 그리고 두 브랜치가 각자 다르게 고친 `run()`.
같은 전역과 같은 함수를 양쪽이 수정했기 때문에 텍스트 병합이 불가능했다.

그래서 기능별로 객체를 떼어내고, `app.py`에는 조립만 남겼다.

| 새 파일 | 병합 전 위치 | 담당 |
|---------|--------------|------|
| `src/arm_service.py` | `app.py`의 전역 `arm_ctrl`/`arm`/`_q_cache`/`_current_q()`/`_servo_degs_within_limits()` | 팔 상태·하드웨어·안전 검사 |
| `src/render.py` | `app.py`의 전역 `_root_link`/`_chain`/`_visuals`/`_render_bounds` + `run()` 안의 `fig3d`/`canvas3d`/`ax3d` | 자세 → PNG, 로컬 3D 창 |
| `src/api/arm.py` | `app.py`의 `@app.route` 전역 함수 7개 | 팔 제어 HTTP 라우트 |
| `src/perception_loop.py` | 두 브랜치의 `run()` 안쪽 | 카메라 → 인식 → 추종 → 미리보기 |
| `src/gesture_bridge.py` | 웹 브랜치의 `_make_gesture_recognizer()` 클로저 | 제스처 → 폰 명령/조명 |
| `src/app.py` (재작성) | — | 조립·페이지·실행만 |

이렇게 하니 **두 계열의 기능이 서로의 코드를 건드리지 않고 나란히 존재**한다.
공유 자원은 `Camera`(소켓)와 `ArmService`(팔 상태) 둘뿐이고, 둘 다 락으로 감싸져
있어 웹 요청 스레드와 카메라 루프가 안전하게 나눠 쓴다.

라우트 등록 방식도 통일했다 — 각 기능 객체가 `register(app)`으로 자기 라우트를
직접 붙인다. 새 기능을 추가할 때 `app.py`를 고칠 필요가 없어지고, 경로와 처리
함수를 같은 파일에서 볼 수 있다.

**동작을 일부러 보존한 지점**: `ArmService`의 이동 메서드들은 관절각 캐시를
무효화하지 않는다. 캐시 미스 한 번이 시리얼 왕복 5회이고 추종 루프는 최대
0.15초마다 명령을 내리므로, 움직일 때마다 캐시를 버리면 캐시가 없는 것과 같아진다.
추적 브랜치가 TTL을 몇 초로 길게 잡은 것도 같은 이유라 그 판단을 그대로 지켰다.

### 2.4 `mobile.html` — 같은 파일의 다른 절을 각자 수정

- 웹 브랜치가 추가: 캘린더 모달 + 자연어 날짜 파싱(7.5절), 제스처 수신 + 조명(8절)
- 추적 브랜치가 추가: 손 랜드마크 전송 모듈 스크립트, `captureLoop()` 안의 전송 호출

웹 브랜치를 바탕으로 추적 브랜치의 두 부분을 얹고 절 번호를 정리했다
(7.5 캘린더 → 8 제스처 → 9 카메라 자동 시작 → **10 손 인식**).

손 랜드마크는 **방향 보정이 끝난 캔버스**에서 뽑아야 한다는 제약이 있어,
`captureLoop()`에서 `ctx.restore()` 직후·`toBlob()` 직전이라는 위치를 지켰다 —
서버가 받는 프레임과 같은 좌표계여야 깊이 추정·월드 변환·제스처 판정이 어긋나지 않는다.

### 2.5 제스처 확정 프레임 수

웹 브랜치 원본은 `hold_frames=66`에 `hold_overrides={PAPER: 10}`이었다.
주석은 "4프레임이면 1초 조금 안 된다", "'보'만 더 오래 들고 있어야 한다"고 적혀
있는데 실제 값은 주석과 반대이고 서로 맞지 않는다(테스트 중 남은 값으로 보인다).

병합 후에는 손 인식이 휴대폰에서 프레임마다(~20fps) 돌아 **같은 프레임 수가 훨씬
짧은 시간**이 되므로, 주석의 의도대로 시간 기준으로 다시 잡았다:

```python
HOLD_FRAMES = 10        # 약 0.5초
HOLD_FRAMES_PAPER = 20  # 약 1초 — 카메라를 끄면 제스처로 되돌릴 수 없으므로 더 길게
```

값은 `fundamental/const.py`의 `GestureConst`에 있으니 실제로 써 보고 조정하면 된다.

### 2.6 기타

| 항목 | 처리 |
|------|------|
| `requirements.txt` | `requests` 추가(`light.py`가 사용). mediapipe 설명을 "얼굴 인식용"으로 정정 |
| `.gitignore` | 웹 브랜치의 `calendar_events.json` 항목 유지 |
| `main.py` | `from logger import Logger` → `from fundamental.logger import Logger` |
| 라우트 엔드포인트 이름 | 20개 전부 충돌 없음을 실행으로 확인 |

---

## 3. 검증 내역

하드웨어·mediapipe·Gemini·라즈베리파이가 **하나도 없는** 환경에서 실제로 실행해 확인했다.

| 확인 항목 | 결과 |
|-----------|------|
| 앱 기동 (`create_app()`) | 정상 — 하드웨어/Gemini/mediapipe 미설치를 각각 "해당 기능만 비활성"으로 처리 |
| 라우트 20개 등록 | 두 계열 모두 충돌 없이 공존 |
| `/`, `/mobile` 렌더링 | 200 |
| `/api/fk`, `/api/ik`, `/api/render` | 정상(PNG 바이트까지 확인) |
| `/api/goto_*` (하드웨어 없음) | 503 + 사유 |
| 잘못된 요청 본문 | 400 / 404 정상 |
| `/api/calendar/*` 추가·조회·삭제 | 정상, 파일 영속화 확인 |
| 폰 형식 랜드마크 → 가위/바위/보 판정 | 세 가지 모두 정확히 분류 |
| 같은 랜드마크로 팔 추종 명령 생성 | `("position", ...)` 정상, idle 복귀도 정상 |
| 제스처 엣지 트리거 | 10프레임에서 1회만 발동, 계속 들고 있어도 재발동 없음 |
| 제스처 쿨다운 | 2.5초 내 후속 제스처 차단 확인 |
| 제스처 끄기 스위치 | 발동 없음 확인 |
| `PerceptionLoop.process_frame()` | 실제 JPEG 한 장으로 예외 없이 통과 |

---

## 4. 실제 하드웨어에서 확인이 필요한 것

시뮬레이션으로는 검증할 수 없는 항목들이다.

1. **제스처 확정 시간** — `GestureConst.HOLD_FRAMES`(§2.5). 폰의 실제 전송
   프레임레이트에 따라 체감이 달라진다.
2. **얼굴 추적 회전 방향** — `FaceFollowerConst.YAW_GAIN`의 부호는 원본에서도
   실측 미확인 상태다. 반대로 돌면 부호만 뒤집으면 된다.
3. **손 추종과 제스처의 동시 동작** — 손을 들면 팔이 그 손을 따라가면서 동시에
   제스처도 판정된다. 의도된 동작이지만, 실제로 써 보고 산만하면
   `GestureBridge`를 만들 때 `enabled=False`로 두거나 폰의 "제스처 끔" 버튼을 쓰면 된다.
4. **조명 각도** — `LightConst.ON_ANGLE`/`OFF_ANGLE`은 스위치마다 다르다.
5. **`PI_URL`** — 파이의 실제 주소로 환경변수를 설정해야 한다.
