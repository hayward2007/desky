# 함수 설명서 — desky 통합본

병합된 코드베이스의 **모든 공개 클래스와 함수**를 모듈별로 정리했다. 각 항목은
"무엇을 하는가"와 함께, 필요한 경우 "왜 그렇게 했는가"를 적었다. 상세한 내용은
각 파일의 docstring에 그대로 들어 있으므로, 이 문서는 전체 지도 역할을 한다.

표기: `[추적]` = 팔 추적 계열에서 온 것, `[웹]` = 웹 기능 계열에서 온 것,
`[병합]` = 합치면서 새로 만들거나 크게 고친 것, 표시가 없으면 두 계열 공통.

---

## 1. 실행 흐름 한눈에

```text
main.py:run()
  └─ src.app.DeskyApp.__init__()      서비스 생성 + 라우트 등록
       └─ .run()
            ├─ start_server()          Flask를 백그라운드 스레드로 (블로킹이므로)
            └─ loop.run_forever()      메인 스레드에서 카메라 루프 (cv2 창 제약)
                 └─ process_frame()    ← 프레임 한 장이 들어올 때마다
                      ├─ detect_faces()      서버 mediapipe FaceMesh
                      ├─ collect_hands()     폰이 보낸 랜드마크 → 월드 좌표
                      ├─ drive_arm()         얼굴>손>idle 판단 → 서보 명령
                      ├─ update_gestures()   가위바위보 확정 → 폰에 명령 전송
                      └─ update_preview()    cv2 창 2개 갱신
```

---

## 2. `fundamental/` — 전역 공통

### `logger.py`
| 함수 | 설명 |
|------|------|
| `Logger.log(tag, message)` | `[TAG] message` 형식으로 출력. `Logger.enabled` 하나로 전 모듈의 로그를 끄고 켠다 |

### `const.py`
값과 그 값의 의미만 모아 둔 선언적 데이터 파일. 알고리즘은 원래 모듈에 남아 있고,
각 모듈이 여기서 import해 자기 클래스 속성으로 다시 건다. 파일 맨 위에서 `.env`를
로드한다 — 아래 클래스들이 import 시점에 바로 `os.environ.get(...)`을 읽으므로,
그보다 늦게 로드하면 값이 조용히 무시된다.

| 클래스 | 담당 |
|--------|------|
| `ActuatorControlTable`, `AX_18A` | DYNAMIXEL 레지스터 주소/바이트 크기 |
| `HardwareActuatorConst` | 이동 속도 기본값, 속도 쓰기 후 대기 시간 |
| `KinematicsConst`, `ArmConst` | 링크 길이, 관절 축, IK 관절 가중치 |
| `FindJointLimitsConst`, `SimulateConst`, `MujocoSimConst` | 한계 탐색·시각화 파라미터 |
| `CameraGeometryConst` | 초점거리, 화면 중앙 데드존, IK x/y 제한 (얼굴·손 공용) |
| `FaceTrackerConst`, `FaceFollowerConst` | 눈 사이 거리 기준자, yaw/높이 이득과 스텝 상한 |
| `HandTrackerConst`, `HandFollowerConst` | 손목~엄지 기준자, 추종 거리, 깊이 보정 비율 |
| `FollowControllerConst` | idle 좌표, 복귀 판정 오차, 두리번거리기 진폭·주기 |
| `GestureConst` `[웹→병합]` | 랜드마크 인덱스, 확정 프레임 수, 쿨다운 |
| `CameraConst`, `GeminiConst` | WebM 매직 바이트, 모델명·시스템 프롬프트 |
| `CalendarConst` `[웹→병합]` | 저장 파일명, 날짜/시간 형식 |
| `LightConst` `[웹→병합]` | 파이 주소, 스위치 각도, 타임아웃, 인터넷 중계 설정(`RELAY_TOPIC`/`RELAY_URL`) |
| `AppConst` | 캐시 TTL, 인식 해상도 상한, 명령/렌더 간격, 서버 바인딩 |

### `network.py`

| 함수 | 설명 |
|------|------|
| `lan_addresses()` | 이 PC가 같은 공유기에서 보이는 사설 IPv4 목록(가장 유력한 것이 맨 앞) |
| `_primary_ipv4()` | UDP 소켓을 공인 주소로 `connect()`해(패킷 없음) 라우팅 테이블 기준 출발지 IP를 얻는다 |
| `server_urls(port, scheme)` | 폰 주소창에 입력할 URL 목록 |
| `startup_banner(port, scheme)` | 서버 시작 시 콘솔에 찍는 안내 상자(접속 안 될 때 확인할 것 포함) |

---

## 3. `hardware/` — 실제 서보 제어

### `controller.py` — `Controller`
시리얼 포트 한 개를 감싸는 저수준 계층.

| 함수 | 설명 |
|------|------|
| `__init__(device_name, baudrate, protocol_version)` | `.env` 값으로 포트를 열고 실패하면 예외 — 이 예외를 `ArmService`가 잡아 "하드웨어 없음"으로 처리한다 |
| `set_speed(id, speed, control_table)` | 이동 속도(%)를 레지스터에 쓴다 |
| `set_goal_position(id, position, control_table)` | 목표 각도(도)를 유닛값으로 바꿔 쓴다 |
| `get_present_position(id, control_table)` | 현재 각도를 읽는다. 통신 실패 시 `None` |
| `_check_comm(...)` | 통신 결과/에러 코드를 해석해 로그를 남기고 성공 여부 반환 |
| `close()`, `__enter__`, `__exit__`, `__del__` | 포트 정리 (with 문 사용 가능) |

### `actuator.py`
| 함수 | 설명 |
|------|------|
| `Actuator.goto(degree, speed)` | 서보 하나를 움직인다. **속도가 직전과 같으면 다시 쓰지 않는다** — 매번 쓰면 관절당 왕복 1회와 25ms 대기가 추가돼 프레임마다 명령하는 추종 루프가 멈춘다 |
| `Actuator.get_position()` | 현재 각도(도) |
| `ArmController.__init__(actuators, arm)` | URDF의 관절 id와 `Actuator.id`를 맞춰 정렬 — 리스트 순서는 상관없다 |
| `ArmController.goto_position(target_pos, ...)` | IK를 풀어 5개 서보에 분배. 수렴하지 않으면 **움직이지 않는다** |
| `ArmController.goto_joints(servo_degs, speed)` | 관절각을 그대로 명령하고 FK 결과 위치를 반환 |
| `ArmController.get_position()` | 5개 서보를 읽어 FK로 현재 위치 계산(하나라도 실패하면 `None`) |

---

## 4. `kinematics/` — 기구학 (하드웨어 불필요, 표준 라이브러리만)

### `kinematics.py`
| 함수 | 설명 |
|------|------|
| `Joint.servo_deg(q)` / `q_from_servo(deg)` | 내부 각(rad) ↔ 서보 각(도) 변환 |
| `Joint.bounds(q_other)` | 자기충돌 안전 범위. 결합 관절은 상대 관절 각도에 따라 달라진다 |
| `Joint.clamp(q, q_other)` | 범위 안으로 자른다 |
| `Arm.fk_matrix(q)` / `fk(q)` / `fk_all(q)` | 순기구학 — 4x4 행렬 / 위치 / 전 관절 프레임 |
| `Arm.ik(target_pos, target_rot, seed, ...)` | damped least squares 역기구학. `seed`로 현재 자세 근처 해를 유도 |
| `Arm.q_to_servo_deg(q)` / `servo_deg_to_q(degs)` | 벡터 단위 변환 |
| `_matmul`, `_rot_axis`, `_rpy`, `_inverse` 등 | 순수 파이썬 4x4 선형대수 도우미 |

### `urdf_loader.py`
| 함수 | 설명 |
|------|------|
| `load_arm(path)` | `configure/desky.urdf`를 파싱해 `Arm`을 만든다 — **로봇 구성의 단일 소스** |

### `simulate.py`
| 함수 | 설명 |
|------|------|
| `parse_urdf(path)` | 링크/조인트/시각 기하를 뽑아낸다 |
| `workspace_bounds(...)` | 화면 범위 고정용 작업공간 크기 |
| `draw_pose(ax, ...)` | 자세 하나를 3D 축에 그린다 (웹 `/api/render`와 로컬 창이 **같은 함수**를 쓴다) |
| `draw_points(ax, points, connections)` | 손 골격처럼 점+연결선을 그린다 |

### `mujoco_sim.py`, `find_joint_limits.py`
MuJoCo로 자기충돌을 쓸어 관절 한계를 찾아 URDF에 반영하는 오프라인 도구.

---

## 5. `perception/` — 인지 (하드웨어를 모른다)

> 이 패키지의 공통 규칙: **"어디로 움직일지"만 계산하고 실제 명령은 하지 않는다.**
> 하드웨어 호출은 전부 `src/` 쪽(`ArmService.execute`)이 한다.

### `camera_geometry.py`
| 함수 | 설명 |
|------|------|
| `camera_frame(T_ee)` | end-effector 행렬에서 카메라 기준 축 3개와 원점을 뽑는다. 얼굴·손 트래커가 **같은 정의**를 공유하도록 한 곳에 모았다 |
| `clamp_xy(position, limit)` | IK 목표의 x, y를 ±limit로 자른다 — 좌우 정렬은 이동이 아니라 yaw 회전으로 흡수하기 때문 |
| `to_pixel(landmark, width, height, margin)` | 정규화 좌표 → 화면 픽셀 정수. NaN/무한대/범위 밖 값으로 OpenCV가 죽는 것을 막는 안전 변환(`draw_overlay`들이 공용) |

### `face_tracker.py` `[추적]`
| 함수 | 설명 |
|------|------|
| `FaceTracker.__init__(...)` | mediapipe FaceMesh 세션 생성. 실패하면 `available=False`로 조용히 남는다 |
| `FaceTracker.process(rgb, T_ee, shape)` | 얼굴을 찾아 `Face` 목록으로 (깊이·월드 중심·화면 오프셋 포함) |
| `FaceTracker.estimate_depth(...)` | 두 눈 바깥 끝 사이 거리를 기준자로 핀홀 역산 |
| `FaceTracker.landmark_to_world(...)` | 랜드마크 → 월드 좌표 |
| `FaceTracker.draw_overlay/draw_faces_3d` | 2D 윤곽 / 3D 파란 점 |
| `FaceFollower.primary_face(faces)` | 여러 얼굴 중 하나만 고른다 |
| `FaceFollower.next_command(faces, T_ee, q)` | 데드존 밖이면 `("joints", 서보각)`. **좌우는 yaw 회전, 상하는 높이(z) 이동**으로 나눠 중앙 정렬 |

### `hand_tracker.py` `[추적]`
| 함수 | 설명 |
|------|------|
| `HandTracker.process_landmarks(raw_hands, T_ee, shape)` | **폰이 보낸** 랜드마크를 `Hand` 목록으로. 모델 추론은 하지 않는다 |
| `HandTracker._to_landmark(point)` | `[x,y,z]`와 `{"x":..}` 두 형식을 모두 받아 정규화 |
| `HandTracker.estimate_depth(...)` | 손목~엄지CMC 길이를 기준자로 거리 역산 |
| `HandTracker.landmark_to_world(...)` | 랜드마크 → 월드 좌표(핀홀 스케일 재적용) |
| `HandTracker.draw_overlay/draw_hands_3d/draw_forward_axis_debug` | 골격·손바닥 사각형 / 3D 골격 / 카메라 방향 화살표 |
| `HandFollower.combined_target(hands)` | 한 손이면 그 중심, 두 손이면 중점 |
| `HandFollower.combined_screen_offset(hands)` | 화면 중앙 대비 오프셋 |
| `HandFollower.next_ee_target(hands, T_ee)` | 데드존 밖이면 새 목표 좌표. 좌우/상하는 완전 정렬, **깊이는 일부만** 보정해 과하게 따라오지 않게 한다 |

### `follow_controller.py` `[추적]`
| 함수 | 설명 |
|------|------|
| `FollowController.next_command(faces, hands, T_ee, q)` | **얼굴 > 손 > idle** 우선순위로 다음 명령 `(kind, payload)` 결정. 보이더라도 중앙 근처면 `None`(가만히) |
| `FollowController._idle_step(T_ee, q)` | idle 좌표로 복귀 → 도착하면 1번 관절을 사인파로 왕복(두리번거리기). 대상이 나타나면 즉시 추적 복귀 |

### `gesture.py` `[웹→병합]`
| 함수 | 설명 |
|------|------|
| `classify(landmarks, aspect)` | 21개 랜드마크를 가위/바위/보/UNKNOWN으로 분류 |
| `classify_hands(hands, frame_shape)` | 여러 손 중 화면에 가장 크게(=가깝게) 보이는 손만 사용 |
| `finger_states(pts, margin)` | 네 손가락이 펴졌는지 — **손목 기준 상대 거리**라 손이 기울어도 안정적 |
| `thumb_extended(pts, margin)` | 엄지는 새끼 MCP를 기준으로 판정(좌우 손 구분 불필요) |
| `_landmarks_of(hand)` | `Hand`든 mediapipe 객체든 리스트든 21개를 꺼낸다 — **병합에서 손 데이터 출처가 바뀌어도 이 아래 로직이 그대로 동작하는 이유** |
| `GestureRecognizer.update(hands, shape)` | 프레임마다 호출. N프레임 유지 시 확정하고, 바뀌는 순간에만 콜백 1회(엣지 트리거) |
| `GestureRecognizer.reset()` | 확정 상태를 비운다(껐다 켜면 같은 모양을 다시 인식하도록) |
| `draw_gesture(frame, gesture)` | 미리보기 좌상단에 결과 표시 |

---

## 6. `src/` — 앱 조립과 웹

### `app.py` — `DeskyApp` `[병합]`
| 함수 | 설명 |
|------|------|
| `__init__(show_preview, gestures_enabled)` | 서비스 생성 → 라우트 등록 → 인식 루프 준비 |
| `_build_services(...)` | 기능 객체들을 만들어 서로 연결. 생성 순서가 곧 의존 방향이며, 어떤 객체도 전역을 읽지 않는다 |
| `_register_routes()` | 페이지 2개를 붙이고, 나머지는 각 객체의 `register()`에 위임 |
| `index()` / `mobile()` | `/` 대시보드 / `/mobile` 폰 페이지 렌더링 |
| `network_info()` | `GET /api/network` — 폰이 입력할 LAN 주소 목록(대시보드 카드가 폴링) |
| `calendar_page()` | `/calendar` 일정 화면 렌더링. 데이터는 넣어 주지 않는다 — 페이지가 자기 시계로 이번 달을 정해 직접 받아 온다 |
| `start_server()` | Flask를 백그라운드 스레드로 실행(`app.run()`이 블로킹이므로). 시작하면서 `network.startup_banner()`를 콘솔에 출력 |
| `run()` | 서버를 띄우고 메인 스레드에서 인식 루프를 돈다 |
| `initialize_position()` | 시작 자세 지정용 훅(기본은 아무것도 안 함 — 켜자마자 팔이 움직이면 위험) |
| `create_app(**kw)` / `run(**kw)` | 모듈 수준 진입점. **import만으로 하드웨어를 잡지 않도록** 생성은 호출 시점으로 미룬다 |

### `arm_service.py` — `ArmService` `[병합]`
| 함수 | 설명 |
|------|------|
| `__init__(actuator_ids, model)` | 하드웨어 연결 시도 → 실패하면 `connected=False`로 남고 팔 모델만 로드 |
| `connected` / `joints` / `actuators` | 상태 조회 프로퍼티 |
| `current_q()` | 현재 관절각(캐시). **실패한 읽기에도 타임스탬프를 갱신**해 통신 오류 시 재시도 폭주를 막는다 |
| `current_q_or_home()` | 모르면 전부 0으로 대체 |
| `ee_matrix(q)` | end-effector 4x4 — 곧 "카메라가 어디서 어디를 보는가" |
| `position()` | 현재 end-effector 위치 |
| `joint_slider_range(joint)` | UI 슬라이더용 (최소, 최대, 홈) |
| `servo_degs_within_limits(degs)` | 자기충돌 안전 범위 검사. 결합 관절은 **이번에 함께 명령될 값**으로 푼다 |
| `servo_degs_with_one_changed(id, deg)` | 관절 하나만 바꾼 가상 벡터(단일 관절 명령 검사용) |
| `goto_position/goto_joints/goto_joint` | 실제 이동. 하드웨어가 없으면 `HardwareUnavailable` |
| `execute(command)` | `FollowController`의 `(kind, payload)`를 실제 명령으로 번역 |
| `invalidate_cache()` | 캐시 강제 무효화 — 이동 메서드는 **일부러 부르지 않는다**(캐시 미스 1회 = 시리얼 왕복 5회) |

### `render.py` `[병합]`
| 함수 | 설명 |
|------|------|
| `ArmRenderer.__init__(arm, urdf_path, ...)` | URDF와 작업공간 범위를 **1회만** 파싱(범위 고정이라 자세가 바뀌어도 화면이 흔들리지 않음) |
| `ArmRenderer.draw_into(ax, q)` | 주어진 축에 팔을 그린다 |
| `ArmRenderer.render_png(q)` | PNG 바이트 반환. 요청마다 새 Figure — 여러 요청이 하나를 공유하면 서로 덮어쓴다 |
| `ArmRenderer.arrow_length(ratio)` | 작업공간에 비례한 디버그 화살표 길이 |
| `ScenePreview.draw(q, T_ee, hands, faces, ...)` | 로컬 3D 창 한 장 생성. Figure를 **재사용**한다(루프에서 반복 호출되므로) |
| `ScenePreview._to_bgr()` | matplotlib RGBA 버퍼 → cv2 BGR |

### `perception_loop.py` — `PerceptionLoop` `[병합]`
| 함수 | 설명 |
|------|------|
| `run_forever()` | 새 프레임을 기다리며 도는 메인 루프. `waitKey`가 HighGUI 이벤트도 돌린다 |
| `process_frame(frame_bytes)` | 디코드 → 인식 → 팔 명령 → 제스처 → 미리보기 |
| `detect_faces(frame, T_ee)` | 축소본으로 mediapipe 실행, 깊이 계산엔 **원본 크기** 사용 |
| `collect_hands(T_ee, shape)` | 폰이 보낸 랜드마크를 월드 좌표로. 얼굴이 잡혀도 건너뛰지 않는다(제스처가 손을 써야 하므로) |
| `drive_arm(faces, hands, T_ee, q, now)` | 판단은 매 프레임, **실제 명령은 간격 제한** — 매 프레임 명령하면 팔이 덜덜거린다 |
| `update_gestures(hands, shape)` | 가위바위보 확정 처리 |
| `update_preview(...)` | cv2 창 2개 갱신. 3D 렌더가 비싸 **따로 더 느리게** 갱신한다 |
| `close()` | 창 닫기 + mediapipe 세션 정리 |

### `gesture_bridge.py` — `GestureBridge` `[병합]`
| 함수 | 설명 |
|------|------|
| `_build_recognizer()` | 제스처 → 동작 표를 붙인 인식기 생성 |
| `_action(action, light)` | 폰에 명령을 보내고 필요하면 조명도 바꾸는 콜백 생성. 조명은 **별도 스레드**(네트워크 대기로 영상이 멈추지 않게) |
| `update(hands, shape)` | 프레임마다 판정 위임 |
| `draw(frame, gesture)` | 미리보기 표시 |
| `set_enabled(bool)` | 켜고 끄기. 끌 때 확정 상태도 비운다(엣지 트리거라 안 비우면 다시 켜도 반응 없음) |

### `src/api/`
| 파일 · 함수 | 설명 |
|-------------|------|
| `arm.ArmAPI.status/fk/ik/render` | 조회·미리보기 — **하드웨어 없이도 동작** |
| `arm.ArmAPI.goto_position/goto_joints/goto_joint` | 실제 이동. 범위 검사 후 명령, 하드웨어 없으면 503 |
| `arm.ArmAPI._parse_degrees/_parse_xyz` | 요청 본문 검증(여러 라우트 공용) |
| `camera.Camera.ws_camera(ws)` | `/ws/camera` 핸들러 — 내용으로 JPEG/음성/텍스트를 구분 |
| `camera.Camera._handle_text_message(text)` | 손 랜드마크 JSON 처리. 잘못된 메시지는 무시(연결을 죽이지 않는다) |
| `camera.Camera.broadcast(payload)` `[웹→병합]` | 서버→폰 명령 전송. 끊긴 소켓은 걷어낸다 |
| `camera.Camera._send(ws, msg)` `[병합]` | 모든 전송의 단일 통로 — 여기서만 전송 락을 잡는다 |
| `camera.Camera.latest_frame/status/snapshot/latest_hand_landmarks` | 프레임·상태·랜드마크 스레드 안전 조회 |
| `gemini.Gemini.ask()` | `/api/ask` 채팅·요약 |
| `gemini.Gemini.transcribe(audio)` | 음성 클립 → 한국어 텍스트 |
| `gemini.Gemini.parse_document(image, mode)` | 문서 이미지 → 전문 또는 요약 |
| `gemini.Gemini.answer_text(response)` | 응답에서 본문만 안전하게 추출 |
| `scan.ScanAPI.parse` | 지금 라이브 프레임을 통째로 Gemini에 보내 본문을 읽는다(문서 검출/선택 없음) |
| `calendar.Calendar.list_events/add_event/delete_event` `[웹]` | 일정 조회(날짜·기간)/추가/삭제 |
| `calendar.Calendar._load/_save` `[웹]` | JSON 영속화. 임시파일→rename으로 원자적 저장 |
| `light.Light._press(angle)` | 중계(`relay.enabled`)가 켜져 있으면 `Relay.press()`로, 아니면 `PI_URL`로 직접 HTTP 호출 — 결과 형태가 같아 호출부는 경로를 몰라도 된다 |
| `light.Light.set(state)` `[웹]` | 파이에 눌러 달라고 요청(타임아웃 짧게) |
| `light.Light.set_async(state)` `[웹]` | 카메라 루프에서 부를 때 쓰는 비동기 버전 |
| `light.Light.command/status` `[웹]` | `/api/light`, `/api/light/status`(현재 경로 `route`와 `pi_alive` 포함) |
| `relay.Relay.press(angle, rest, hold_ms)` | 중계소로 명령을 보내고 파이의 ack를 기다린다(성공여부, 에러) |
| `relay.Relay.enabled` | `DESKY_RELAY_TOPIC`이 설정돼 있는지 — `Light`가 경로를 고르는 기준 |
| `relay.Relay.pi_alive` | 파이의 마지막 생존 신호가 `RELAY_ALIVE_S` 안인지(시연 전 점검용) |
| `relay.Relay._listen_forever/_handle` | 파이 응답 주제를 구독해 대기 중인 명령에 결과를 연결한다. 끊기면 재접속 |
| 각 클래스의 `register(app)` `[병합]` | 자기 라우트를 직접 등록 — 경로와 함수를 같은 파일에서 볼 수 있게 |

### `src/templates/mobile.html` `[병합]`
번호 순서대로 읽으면 된다.

| 절 | 내용 |
|----|------|
| 1 | 카메라 스트리밍(방향 보정 후 JPEG 전송, 손 랜드마크도 같은 캔버스에서) |
| 2~6 | 소켓 연결, Gemini 질의, TTS, 문서 스캔 UI |
| 7 | 음성 명령 해석(조명 → 일정 → 스캔 순으로 먼저 걸러내고 나머지는 Gemini) |
| 7.5 `[웹]` | 캘린더 — 자연어 날짜/시간 파싱, 모달 UI, 음성 추가/삭제/조회 |
| 8 `[웹]` | 제스처 명령 수신 + 조명 버튼 |
| 9 | 페이지 로드 시 카메라 자동 시작 |
| 10 `[추적]` | MediaPipe Tasks Vision으로 손 인식 → 랜드마크만 소켓 전송 |

### `src/templates/calendar.html` — 일정 화면
| 함수 | 설명 |
|------|------|
| `fetchMonth()` | 격자 42칸이 걸친 범위를 `?from=&to=`로 **한 번에** 받아 온다(칸마다 부르지 않는다) |
| `groupByDate()` | 날짜별로 묶어 둔다 — 42칸을 그릴 때 칸마다 전체 목록을 훑지 않도록 |
| `renderGrid()` | 6주 고정(42칸) 달력을 그린다. 칸 수가 고정이라 달을 넘겨도 높이가 흔들리지 않는다. 칸당 일정 2건까지, 나머지는 `+N` |
| `renderSide()` | 고른 날의 일정 목록(시간순, 종일은 뒤) |
| `addEvent()` / `deleteEvent(id)` | 고른 날에 추가 / 한 건 삭제 후 새로고침 |
| `moveMonth(delta)` | 달 이동. 고른 날이 화면 밖이면 그 달 1일로 옮긴다 |
| 날짜 도우미 | `atMidnight`/`addDays`/`fmtISO`/`isoToDate`/`fmtKor` — mobile.html의 같은 이름 함수들과 규칙 동일 |

### `src/static/theme.css` — 공통 디자인 시스템
색 토큰(`--paper`/`--ink`/`--graphite`/`--signal`/`--alert` 등), 글꼴, 그리고 두 화면이
공유하는 컴포넌트(`.tag` `.num` `.chip` `.big` `.field`)를 담는다. 어두운 면 위에서는
그 면에 `on-ink` 클래스를 붙이면 칩 색이 자동으로 뒤집힌다.

---

## 7. `raspberry-server/` — 파이에서 따로 실행 `[웹]`

| 함수 | 설명 |
|------|------|
| `Servo._pick_backend()` | pigpio → lgpio → 모의 모드 자동 선택. lgpio는 후보 칩 번호마다 실제로 서보 핀을 **점유해 보고** 성공한 칩을 쓴다(`_claim_output`) — 열기만으로는 맞는 칩인지 알 수 없어서 |
| `Servo.goto(angle)` | 각도로 이동 후 일정 시간 뒤 신호를 끊는다(떨림·발열 방지) |
| `Servo.press(angle, rest, hold_ms)` | 눌렀다 원위치 — 벽 스위치를 누르는 동작 |
| `/api/press`, `/api/servo`, `/api/status` | 메인 서버가 호출하는 HTTP 인터페이스 |
| `RelayClient.start()` | `DESKY_RELAY_TOPIC` 설정 시 로컬 HTTP 서버와 **동시에** 백그라운드로 시작 |
| `RelayClient._subscribe_forever()` | 중계소의 명령 주제를 구독(긴 HTTP GET). 끊기면 3초 후 재접속 |
| `RelayClient._heartbeat_forever()` | 30초마다 생존 신호 발행 — PC가 조명 버튼을 누르기 전에 파이가 켜져 있는지 미리 확인할 수 있게 |
| `RelayClient._handle(line)` | 중계소가 보낸 명령 한 줄 처리 → 서보 실행 → 결과(ack)를 응답 주제로 발행 |
