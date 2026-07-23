"""프로젝트 전역에서 쓰는 상수 모음.

클래스 하나가 원래 어느 모듈/클래스가 쓰던 상수 묶음인지에 대응한다(이름도
그대로 `<원래 클래스명>Const`, 클래스가 아니라 모듈 하나였던 경우는
`<모듈명>Const`). 그 모듈/클래스는 여기서 해당 클래스를 import해서 자기
클래스 속성(또는 모듈 전역 변수)으로 다시 건다 — 예:

    from fundamental.const import FaceFollowerConst

    class FaceFollower:
        YAW_GAIN = FaceFollowerConst.YAW_GAIN

이렇게 하면 원래 모듈 안의 참조(`self.YAW_GAIN`, `FaceFollower.YAW_GAIN`,
생성자 기본값 등)는 전부 그대로 동작한다 — 실제 동작(알고리즘)은 원래
모듈에 그대로 남아있고, 여기는 "값 하나 + 그 값이 뭘 의미하는지 설명"만
모아둔 선언적 데이터다.

예외적으로 옮기지 않은 것:
- kinematics.kinematics.DEFAULT_JOINTS/TOOL_OFFSET — 이 파일에 정의된 Joint
  클래스의 인스턴스를 직접 생성하는 코드라, 값이 아니라 구성 로직에 가깝다.
  const.py가 kinematics.kinematics를 import하는 순환 의존을 피하려고 원래
  자리에 남겨뒀다(단, 그 안에 쓰이는 원시 상수인 링크 길이/축 벡터는
  KinematicsConst로 옮겼다).
- kinematics.urdf_loader._DEFAULT_URDF_PATH — 숫자/튜닝값이 아니라
  `os.path.dirname(__file__)` 기준으로 계산되는 경로라서, kinematics 밖으로
  옮기면 의미가 깨진다.

fundamental/ 밑에 logger.py와 함께 있는 이유: 둘 다 프로젝트 전역에서
참조되는 공통 루트 모듈이기 때문.
"""

import math


# =============================================================================
# hardware/control_table.py였던 DYNAMIXEL 컨트롤 테이블(레지스터 주소 · 바이트
# 크기). 원래도 순수 데이터 클래스였으므로 파일째로 여기로 옮겼다.
# =============================================================================
class ActuatorControlTable:
    """컨트롤 테이블의 공통 인터페이스(타입 힌트 전용) — 실제 값은 모델별
    서브클래스(예: AX_18A)가 채운다."""

    Unit_Number: int
    Protocol_Version: float

    class Address:
        Model_Number: int
        Firmware_Version: int
        ID: int
        Baud_Rate: int
        Return_Delay_Time: int
        CW_Angle_Limit: int
        CCW_Angle_Limit: int
        Temperature_Limit: int
        Min_Voltage_Limit: int
        Max_Voltage_Limit: int
        Max_Torque: int
        Status_Return_Level: int
        Alarm_LED: int
        Shutdown: int
        Torque_Enable: int
        LED: int
        CW_Compliance_Margin: int
        CCW_Compliance_Margin: int
        CW_Compliance_Slope: int
        CCW_Compliance_Slope: int
        Goal_Position: int
        Moving_Speed: int
        Torque_Limit: int
        Present_Position: int
        Present_Speed: int
        Present_Load: int
        Present_Voltage: int
        Present_Temperature: int
        Registered: int

    class Size:
        Model_Number: int
        Firmware_Version: int
        ID: int
        Baud_Rate: int
        Return_Delay_Time: int
        CW_Angle_Limit: int
        CCW_Angle_Limit: int
        Temperature_Limit: int
        Min_Voltage_Limit: int
        Max_Voltage_Limit: int
        Max_Torque: int
        Status_Return_Level: int
        Alarm_LED: int
        Shutdown: int
        Torque_Enable: int
        LED: int
        CW_Compliance_Margin: int
        CCW_Compliance_Margin: int
        CW_Compliance_Slope: int
        CCW_Compliance_Slope: int
        Goal_Position: int
        Moving_Speed: int
        Torque_Limit: int
        Present_Position: int
        Present_Speed: int
        Present_Load: int
        Present_Voltage: int
        Present_Temperature: int
        Registered: int


class AX_18A(ActuatorControlTable):
    """AX-18A 컨트롤 테이블 실값. 레지스터 주소(Address)와 바이트 크기(Size)."""

    Unit_Number = 1023
    Protocol_Version = 1.0

    class Address:
        Model_Number = 0
        Firmware_Version = 2
        ID = 3
        Baud_Rate = 4
        Return_Delay_Time = 5
        CW_Angle_Limit = 6
        CCW_Angle_Limit = 8
        Temperature_Limit = 11
        Min_Voltage_Limit = 12
        Max_Voltage_Limit = 13
        Max_Torque = 14
        Status_Return_Level = 16
        Alarm_LED = 17
        Shutdown = 18
        Torque_Enable = 24
        LED = 25
        CW_Compliance_Margin = 26
        CCW_Compliance_Margin = 27
        CW_Compliance_Slope = 28
        CCW_Compliance_Slope = 29
        Goal_Position = 30
        Moving_Speed = 32
        Torque_Limit = 34
        Present_Position = 36
        Present_Speed = 38
        Present_Load = 40
        Present_Voltage = 42
        Present_Temperature = 43
        Registered = 44
        Moving = 46
        Lock = 47
        Punch = 48

    class Size:
        Model_Number = 2
        Firmware_Version = 1
        ID = 1
        Baud_Rate = 1
        Return_Delay_Time = 1
        CW_Angle_Limit = 2
        CCW_Angle_Limit = 2
        Temperature_Limit = 1
        Min_Voltage_Limit = 1
        Max_Voltage_Limit = 1
        Max_Torque = 2
        Status_Return_Level = 1
        Alarm_LED = 1
        Shutdown = 1
        Torque_Enable = 1
        LED = 1
        CW_Compliance_Margin = 1
        CCW_Compliance_Margin = 1
        CW_Compliance_Slope = 1
        CCW_Compliance_Slope = 1
        Goal_Position = 2
        Moving_Speed = 2
        Torque_Limit = 2
        Present_Position = 2
        Present_Speed = 2
        Present_Load = 2
        Present_Voltage = 1
        Present_Temperature = 1
        Registered = 1
        Moving = 1
        Lock = 1
        Punch = 2


# =============================================================================
# hardware/actuator.py
# =============================================================================
class HardwareActuatorConst:
    """hardware.actuator.Actuator/ArmController가 쓰는 상수."""

    # Moving_Speed 레지스터를 쓴 뒤 Goal_Position을 쓰기 전 대기 시간(초) —
    # 속도 변경이 실제로 적용될 시간을 준다. speed가 이전 goto() 호출과 같으면
    # (흔한 경우) 아예 다시 쓰지 않으므로 이 대기도 건너뛴다(Actuator.goto 참고).
    TIME_INTERVAL_S = 0.025
    # Actuator 생성 시 기본으로 세팅해두는 최소 속도(%, 0~100) — 첫 goto() 전
    # 안전값.
    MIN_SPEED_PERCENT = 1
    # goto()/ArmController.goto_position()/goto_joints()가 speed를 명시적으로
    # 받지 않았을 때 쓰는 기본 이동 속도(%, 0~100). 5에서 낮춤 — 인식/렌더링이
    # 밀렸다가 몰아서 명령이 들어올 때, 속도가 높으면 매번 목표까지 확
    # 움직여서 산만하고 갑작스러워 보인다. 낮은 속도는 명령 간격이 고르지
    # 않아도 움직임 자체를 완만하게 만든다(AppConst.COMMAND_MIN_INTERVAL_S로
    # 명령 빈도 자체도 같은 이유로 제한한다).
    DEFAULT_SPEED_PERCENT = 10


# =============================================================================
# kinematics/kinematics.py
# =============================================================================
class KinematicsConst:
    """kinematics.kinematics 모듈의 링크 길이 placeholder + 관절 축 정의.

    링크 길이는 실측 전 placeholder 값이다(모듈 docstring 참고) — 실제 치수를
    알게 되면 이 값들만 바꾸면 FK/IK가 자동으로 반영한다. 각 오프셋은 이전
    관절 프레임에서 이 관절 원점까지의 이동량(이 관절이 회전하기 전 기준).
    """

    BASE_HEIGHT_MM = 50.0   # base bottom -> yaw joint (along Z)
    RISER_MM = 30.0         # yaw joint  -> roll joint (along Z)
    SHOULDER_MM = 40.0      # roll joint -> first pitch joint (id3), along Z
    UPPER_ARM_MM = 120.0    # id3 -> id4, along local X
    FOREARM_MM = 100.0      # id4 -> id5, along local X
    TOOL_MM = 80.0          # id5 -> phone mount (end-effector), along local X

    # URDF <axis xyz="..."/> 관례에 맞춘 관절 축 단위벡터.
    YAW = (0.0, 0.0, 1.0)    # about Z
    ROLL = (1.0, 0.0, 0.0)   # about X
    PITCH = (0.0, 1.0, 0.0)  # about Y


class ArmConst:
    """kinematics.kinematics.Arm이 쓰는 상수."""

    # 관절별 IK 가중치(id -> weight, 기본 1.0). 높을수록 damped least squares가
    # 그 관절을 덜 움직이려 한다(이 5-DOF 팔은 3-DOF 위치 타겟에 대해 널스페이스
    # 2 자유도의 중복이 있음). joint2(roll)는 자기충돌 회피 범위가 좁고
    # joint3의 현재 각도에 종속적이라(Joint.coupled_table), joint1(yaw)이 같은
    # 도달 범위를 더 많이 커버하도록 가중치를 높였다 — joint2가 결합 한계에
    # 덜 몰리게 된다.
    DEFAULT_JOINT_WEIGHTS = {2: 4.0}


# =============================================================================
# kinematics/find_joint_limits.py
# =============================================================================
class FindJointLimitsConst:
    """kinematics.find_joint_limits가 쓰는 상수."""

    STEP_DEG = 0.5           # 스윕 정밀도(각도 단위 스텝 크기)
    SAFETY_MARGIN_DEG = 2.0  # 찾은 충돌 경계에서 이만큼 더 물러난다
    COUPLE_SAMPLES = 9       # joint2의 coupled_limit 테이블을 만들 때 쓰는 joint3 샘플 개수

    # joint5는 휴대폰/카메라 마운트를 달고 있다. MuJoCo는 메시 자기충돌만 알고
    # 카메라나 케이블은 모르므로, 극단적인 손목 회전이 케이블을 꼬거나 카메라를
    # 엉뚱한 곳으로 향하게 하는 것을 스스로 발견할 수 없다 — 그래서 이건
    # 독립적인 하드 캡이고, 자기충돌 스윕 결과와 교집합으로 적용된다.
    JOINT5_CAMERA_SAFE_SERVO_DEG = (90.0, 270.0)


# =============================================================================
# kinematics/simulate.py
# =============================================================================
class SimulateConst:
    """kinematics.simulate가 쓰는 시각화 상수."""

    # 박스 면은 항상 이 순서로 나온다; index i -> 라벨(i+1).
    FACE_NAMES = ["+Z (top)", "-Z (bottom)", "+X", "-X", "+Y", "-Y"]
    FACE_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4"]
    CYLINDER_COLOR = "#9e9e9e"


# =============================================================================
# kinematics/mujoco_sim.py
# =============================================================================
class MujocoSimConst:
    """kinematics.mujoco_sim의 관절 축 오버레이 상수."""

    AXIS_LEN = 0.08
    AXIS_WIDTH = 0.003
    # 관절마다 하나씩 순환하는 색 — yaw/roll/pitch를 구분하는 의미는 없고,
    # 5개 화살표를 시각적으로 구별하기 위한 것.
    AXIS_COLORS = [
        (0.90, 0.10, 0.10, 1.0),
        (0.10, 0.80, 0.20, 1.0),
        (0.15, 0.45, 0.95, 1.0),
        (0.95, 0.80, 0.10, 1.0),
        (0.85, 0.15, 0.85, 1.0),
    ]


# =============================================================================
# perception/camera_geometry.py
# =============================================================================
class CameraGeometryConst:
    """perception.camera_geometry가 쓰는 상수 — FaceTracker/HandTracker,
    FaceFollower/HandFollower가 물리적으로 같은 이유로 같은 값을 쓰는 상수도
    (예전엔 양쪽에 따로 선언돼 있었다) 여기 하나로 모아 중복을 없앴다."""

    # IK 타겟의 좌우/앞뒤(x, y) 성분에 적용하는 한계(m). 이 팔은 베이스 회전축
    # (z축, x=y=0) 부근에서 1번 관절 회전 + 높이(z) 변화 위주로 움직이도록
    # 설계됐다(FollowControllerConst.IDLE_POSITION이 x=y=0인 것도 같은 이유) —
    # 화면 좌우 오프셋은 x/y 이동이 아니라 yaw 회전으로 흡수해야 하므로,
    # FaceFollower/HandFollower 둘 다 IK에 넘기는 타겟의 x, y는 이 한계로
    # clamp한다.
    IK_XY_LIMIT_M = 0.05

    # 이미지 '가로'로 정규화한 핀홀 초점거리 — FaceTracker와 HandTracker가
    # 둘 다 같은 물리 카메라(휴대폰 전면 카메라)를 가정하므로 같은 값이어야
    # 한다(우연히 같은 게 아니다). 1.0 ~= 수평 화각 53도, 어림값 — 실제
    # 캘리브레이션한 값은 아니므로 거리 추정은 측정이 아니라 추정치다.
    FOCAL_NORM = 1.0

    # 화면 중앙 데드존 반경(정규화 좌표, hypot(dx, dy) 기준) — FaceFollower/
    # HandFollower 둘 다 "화면 중앙에서 이만큼 벗어나야 따라간다"는 같은
    # 개념이라 같은 값을 쓴다. 이보다 가까우면 그대로 둔다.
    CENTER_OFFSET_THRESHOLD = 0.15


# =============================================================================
# perception/face_tracker.py
# =============================================================================
class FaceTrackerConst:
    """perception.face_tracker.FaceTracker가 쓰는 상수.

    FOCAL_NORM은 여기 없다 — HandTrackerConst.FOCAL_NORM과 같은 값이라
    CameraGeometryConst.FOCAL_NORM 하나로 통합했다(중복 상수 통일).
    """

    # 양쪽 눈 바깥쪽 끝(landmark 33=오른쪽, 263=왼쪽) 사이 실제 거리 추정치.
    # 실측 캘리브레이션 값이 아니므로 거리 추정은 참고치 — HandTrackerConst의
    # WRIST_TO_THUMB_CMC_M와 같은 방식(핀홀 역산의 기준자).
    EYE_OUTER_DISTANCE_M = 0.09
    LEFT_EYE_OUTER = 33
    RIGHT_EYE_OUTER = 263
    # 얼굴 중심으로 쓰는 랜드마크 — FaceMesh 표준 index, 코끝 근처.
    CENTER_LANDMARK = 1


class FaceFollowerConst:
    """perception.face_tracker.FaceFollower가 쓰는 상수.

    CENTER_OFFSET_THRESHOLD는 여기 없다 — HandFollowerConst.CENTER_OFFSET_THRESHOLD와
    같은 값이라 CameraGeometryConst.CENTER_OFFSET_THRESHOLD 하나로 통합했다.
    """

    # 화면 좌우 오프셋(대략 -0.5~0.5)을 1번 관절 각도 보정량(rad)으로 바꾸는
    # 비례 이득 — 실측으로 튜닝 필요.
    YAW_GAIN = 0.8
    # 한 번의 갱신에서 1번 관절이 움직일 수 있는 최대 각도(rad) — 큰 오프셋이
    # 갑자기 튀어도 로봇이 한 번에 확 돌지 않도록 하는 안전판.
    YAW_STEP_LIMIT = math.radians(20)
    # 화면 상하 오프셋(대략 -0.5~0.5)을 높이(z) 보정량(m)으로 바꾸는 비례
    # 이득 — 실측으로 튜닝 필요.
    HEIGHT_GAIN = 0.2
    # 한 번의 갱신에서 높이(z)가 움직일 수 있는 최대 거리(m) — YAW_STEP_LIMIT과
    # 같은 이유의 안전판.
    HEIGHT_STEP_LIMIT = 0.03


# =============================================================================
# perception/hand_tracker.py
# =============================================================================
class HandTrackerConst:
    """perception.hand_tracker.HandTracker가 쓰는 상수.

    FOCAL_NORM은 여기 없다 — FaceTrackerConst.FOCAL_NORM과 같은 값이라
    CameraGeometryConst.FOCAL_NORM 하나로 통합했다(중복 상수 통일).
    """

    # 손목(랜드마크 0) ~ 엄지 CMC(랜드마크 1) 실제 길이. 거리 역산의 기준자.
    WRIST_TO_THUMB_CMC_M = 0.035

    # 손바닥 사각형을 그릴 때 잇는 랜드마크 (손목 - 엄지CMC - 검지MCP - 새끼MCP)
    PALM_QUAD = [0, 1, 5, 17]


class HandFollowerConst:
    """perception.hand_tracker.HandFollower가 쓰는 상수.

    CENTER_OFFSET_THRESHOLD는 여기 없다 — FaceFollowerConst.CENTER_OFFSET_THRESHOLD와
    같은 값이라 CameraGeometryConst.CENTER_OFFSET_THRESHOLD 하나로 통합했다.
    """

    # 앞뒤(깊이) 목표 거리(m) — 손이 이 거리에 있도록 일부만 보정한다.
    FOLLOW_DISTANCE_M = 0.45
    # 깊이(앞뒤) 오차를 한 번에 보정하는 비율. 1.0이면 즉시 FOLLOW_DISTANCE_M로
    # 스냅, 0에 가까울수록 거의 따라가지 않는다.
    DEPTH_FOLLOW_GAIN = 0.3


# =============================================================================
# perception/follow_controller.py
# =============================================================================
class FollowControllerConst:
    """perception.follow_controller.FollowController가 쓰는 상수."""

    # 얼굴/손이 안 보일 때 복귀하는 idle 좌표(x, y, z). x=y=0인 이유는
    # CameraGeometryConst.IK_XY_LIMIT_M과 같다 — 베이스 회전축 위의 점이라
    # 어떤 yaw로도 도달 가능하다.
    IDLE_POSITION = (0.0, 0.0, 0.34)
    # IDLE_POSITION 도달 판정 허용 오차(m) — 이 안에 들어오면 복귀 완료로 본다.
    RETURN_TOLERANCE_M = 0.02
    # 룩어라운드에서 1번 관절(yaw)이 idle 자세 기준으로 왕복하는 최대 각도(rad).
    LOOKAROUND_AMPLITUDE = math.radians(40)
    # 왕복 한 사이클(가운데->오른쪽->가운데->왼쪽->가운데) 걸리는 시간(초).
    LOOKAROUND_PERIOD_S = 6.0


# =============================================================================
# perception/document_scanner.py
# =============================================================================
class DocumentScannerConst:
    """perception.document_scanner.DocumentScanner가 쓰는 상수(튜닝 파라미터).

    DocumentScanner(**overrides)로 개별 값을 덮어쓸 수 있다 — 예:
    DocumentScanner(MIN_AREA_RATIO=0.01).
    """

    PROC_HEIGHT = 500.0      # 검출용으로 축소할 높이. 결과 좌표는 원본 크기로 되돌린다
    CANNY_LO = 75            # Canny 하한 임계값 (사진이 흐릿하면 낮춘다)
    CANNY_HI = 200           # Canny 상한 임계값
    MIN_AREA_RATIO = 0.04    # 문서로 인정할 최소 면적(화면의 4%). 작은 문서를 잡으려면 낮춘다
    RECT_MIN = 0.60          # 사각형성(윤곽 면적 / 최소회전사각형 면적) 하한. 손·불규칙 형태 제거용
    DEDUP_DIST_RATIO = 0.05  # 두 사각형 중심이 화면 대각선의 이 비율보다 가까우면 같은 문서로 본다


# =============================================================================
# src/api/camera.py
# =============================================================================
class CameraConst:
    """src.api.camera.Camera가 쓰는 상수."""

    # WebM/Opus 음성 클립의 EBML 헤더(매직 바이트) — JPEG(SOI 마커로 시작)와
    # 구분하는 기준.
    WEBM_EBML_HEADER = b"\x1a\x45\xdf\xa3"


# =============================================================================
# src/api/gemini.py
# =============================================================================
class GeminiConst:
    """src.api.gemini.Gemini가 쓰는 상수(모델명 + 시스템 프롬프트 + 토큰 한도)."""

    MODEL = "gemini-flash-latest"  # 빠르고 무료 한도가 넉넉함

    # 답이 TTS로 그대로 읽히므로 짧고 말하듯이. 목록/기호/마크다운은 소리로
    # 읽으면 이상하니 금지한다.
    CHAT_INSTRUCTION = (
        "너는 음성으로 대답하는 한국어 AI 비서야. 대답은 그대로 소리 내어 읽히니까, "
        "최대한 짧게, 100자 이내로 꼭 필요한 핵심만 자연스러운 구어체로 말해. 덧붙이는 설명은 생략해. "
        "목록·번호·기호·마크다운·이모지는 쓰지 말고 말하듯이 이어서 답해."
    )
    SUMMARY_INSTRUCTION = (
        "너는 문서를 음성으로 요약해주는 한국어 비서야. 핵심만 3문장 이내로, "
        "목록·기호 없이 말하듯 간결하게 정리해."
    )
    STT_INSTRUCTION = "이 오디오를 한국어 텍스트로 정확히 받아써줘. 설명 없이 텍스트만 출력해."

    # 문서 전체 텍스트를 그대로 뽑을 때 (화면에 표시용)
    DOC_PARSE_INSTRUCTION = (
        "이 이미지는 종이 문서를 촬영한 것이다. 문서에 적힌 모든 텍스트를 정확히 "
        "읽어서 그대로 출력해라. 원본의 줄바꿈을 최대한 유지하고, 설명이나 요약 없이 "
        "텍스트만 출력해라. 손글씨도 최대한 읽어라. 문서에 글자가 없으면 '(텍스트 없음)'"
        "이라고만 답해라."
    )
    # 문서를 음성으로 읽어줄 때 (짧은 요약, TTS 친화적)
    DOC_READ_INSTRUCTION = (
        "이 이미지는 사용자가 가리킨 종이 문서다. 내용을 읽고 핵심만 한국어로 "
        "간결하게 3문장 이내로, 목록·기호·마크다운 없이 말하듯이 설명해라. "
        "글자가 없으면 '문서에서 글자를 못 찾았어요'라고만 답해라."
    )

    # 사고를 억지로 누르면 이 모델은 사고 과정을 답변 본문에 적어버린다(페르소나·
    # 포맷 체크리스트가 새어 나옴). 그래서 사고 수준은 기본값에 맡기고 천장만
    # 넉넉히 줘서 '사고 + 답'이 잘리지 않게 한다. 천장을 올려도 답이 길어지진 않는다.
    MAX_OUTPUT_TOKENS = 8192


# =============================================================================
# src/app.py
# =============================================================================
class AppConst:
    """src.app이 쓰는 상수."""

    # _current_q()의 서보 위치 읽기 캐시 TTL(초) — /api/status 폴링과 run()의
    # 3D 오버레이(모바일 카메라 프레임마다 재계산)가 각자 실제 서보를 읽지
    # 않고 공유하는 캐시 유효 시간.
    Q_CACHE_TTL_S = 3.0
    # run()의 미리보기 루프에서 mediapipe(얼굴 인식)에 넘기는 프레임 복사본의
    # 최대 가로 폭(px) — mediapipe 비용은 픽셀 수에 비례하므로, 휴대폰이 그보다
    # 고해상도로 보내면 이 폭으로 다운스케일한 복사본에서만 인식을 돌린다
    # (원본 프레임/치수는 오버레이 표시와 깊이 추정에 그대로 쓰인다). 손 인식은
    # 더 이상 여기서 안 돈다 — 휴대폰(mobile.html)이 자체적으로 MediaPipe
    # Tasks Vision HandLandmarker를 돌려 랜드마크만 웹소켓으로 보낸다(서버는
    # 얼굴 인식만 전담) — 안 그러면 폰이 실제로 보내는 ~20fps를 서버가
    # 얼굴+손 인식을 모두 mediapipe로 처리하며 따라가지 못했다.
    MEDIAPIPE_MAX_WIDTH = 480

    # run()이 실제로 하드웨어에 명령(goto_position/goto_joints)을 내리는 최소
    # 간격(초). 결정 루프(인식 → 추종 명령)는 프레임이 들어올 때마다(최대
    # ~20fps) 매번 돌지만, 목표가 프레임마다 조금씩 바뀔 때 그때그때 전부
    # 재명령하면 로봇이 계속 새로 움직이기 시작해서 산만해 보인다 — 이
    # 간격보다 자주는 실제 서보 명령을 보내지 않는다(HardwareActuatorConst.
    # DEFAULT_SPEED_PERCENT를 낮춘 것과 같은 목적).
    COMMAND_MIN_INTERVAL_S = 0.15

    # 로컬 미리보기 창(cv2 카메라 창 + matplotlib 3D 씬)을 다시 그리는 최소
    # 간격(초). matplotlib 3D 렌더링(ax.cla() + Poly3DCollection 재구성 +
    # 범례/라벨 텍스트 + canvas.draw())은 얼굴 인식보다 훨씬 비싸다 — 결정
    # 루프와 같은 빈도로 매 프레임 다시 그리면 그 렌더링 자체가 병목이 되어
    # 폰이 보내는 ~20fps를 못 따라간다. 이 창은 로봇 동작에 영향을 주지
    # 않는 디버그용 미리보기이므로 훨씬 느리게 갱신해도 무방하다.
    VIS_MIN_INTERVAL_S = 0.1
