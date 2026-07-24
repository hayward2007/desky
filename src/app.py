"""desky 통합 앱 — 두 브랜치(팔 추적 / 웹 기능)를 하나로 조립하는 최상위 객체.

이 파일은 **조립만** 한다. 실제 일은 전부 아래 객체들이 나눠 가진다:

    ArmService      팔 모델 + 하드웨어 + 관절각 캐시 + 안전 검사   (src/arm_service.py)
    ArmRenderer     URDF 기하 파싱, 자세 → PNG                     (src/render.py)
    ArmAPI          팔 제어 HTTP 라우트                            (src/api/arm.py)
    Camera          /ws/camera 양방향 소켓(프레임·음성·랜드마크·명령) (src/api/camera.py)
    Gemini          채팅/요약/STT/문서 읽기                        (src/api/gemini.py)
    ScanAPI         문서 검출·선택·읽기 라우트                     (src/api/scan.py)
    Calendar        일정 저장소 + 라우트                           (src/api/calendar.py)
    Light           라즈베리파이 조명 스위치 프록시                (src/api/light.py)
    GestureBridge   가위바위보 → 폰 명령/조명                      (src/gesture_bridge.py)
    PerceptionLoop  카메라→인식→추종→미리보기 루프                 (src/perception_loop.py)

[병합 메모] 병합 전에는 두 브랜치 모두 이 자리에 500줄이 넘는 모듈 전역 스크립트를
두고 있었다 — 전역 `app`, `arm_ctrl`, `gemini`, `camera` …와 `@app.route`가 붙은
전역 함수들. 두 브랜치가 같은 전역과 같은 `run()` 함수를 각자 수정했기 때문에
텍스트로는 도저히 합쳐지지 않았다. 그래서 각 기능을 객체로 떼어 낸 뒤, 이
파일에는 "무엇을 만들어 어디에 연결하는가"만 남겼다. 그 결과 두 브랜치의 기능이
서로의 코드를 건드리지 않고 나란히 존재한다:

  · 팔 추적 계열(얼굴/손 추종, 두리번거리기)  → PerceptionLoop + FollowController
  · 웹 기능 계열(일정·조명·스캔·대화·제스처)  → 각 API 객체 + GestureBridge
  · 둘의 유일한 접점                          → Camera(소켓)와 ArmService(팔 상태)

두 계열이 공유하는 자원은 이 둘뿐이고, 둘 다 스레드 안전하게 감싸져 있다.

페이지는 셋이다: `/`(PC 대시보드), `/mobile`(팔에 달린 폰), `/calendar`(일정).

실행:
    python main.py            (권장)
    python -m src.app         (같은 동작)
그다음 브라우저에서 https://localhost:8000, 팔에 달린 폰에서는
https://<이 PC의 LAN IP>:8000/mobile 을 연다.
"""

import threading

from flask import Flask, render_template
from flask_sock import Sock

from fundamental.const import AppConst
from fundamental.logger import Logger
from perception.document_scanner import DocumentScanner
from src.api.arm import ArmAPI
from src.api.calendar import Calendar
from src.api.camera import Camera
from src.api.gemini import Gemini
from src.api.light import Light
from src.api.scan import ScanAPI
from src.arm_service import ArmService
from src.gesture_bridge import GestureBridge
from src.perception_loop import PerceptionLoop
from src.render import ArmRenderer

try:
    # 선택 의존성: .env에서 GEMINI_API_KEY(와 하드웨어의 DEVICE_NAME)를 읽을
    # 때만 필요하다. 없으면 진짜 환경변수로 이미 설정돼 있어야 한다.
    from dotenv import load_dotenv

    load_dotenv()
except ImportError:
    pass


class DeskyApp:
    """Flask 앱과 모든 기능 객체를 조립해 들고 있는 최상위 객체."""

    def __init__(self, show_preview=True, gestures_enabled=True):
        """서비스들을 만들고 라우트를 붙인다(서버는 아직 안 띄운다).

        show_preview     : 로컬 cv2 미리보기 창을 띄울지(헤드리스 서버면 False).
        gestures_enabled : 가위바위보 제스처를 켤지.
        """
        self.flask = Flask(__name__)
        self.sock = Sock(self.flask)

        self._build_services(gestures_enabled)
        self._register_routes()
        self.loop = PerceptionLoop(
            arm_service=self.arm_service,
            camera=self.camera,
            renderer=self.renderer,
            gesture_bridge=self.gesture_bridge,
            show_preview=show_preview,
        )

    # ------------------------------------------------------------------
    # 조립
    # ------------------------------------------------------------------
    def _build_services(self, gestures_enabled):
        """기능 객체들을 만들어 서로 연결한다.

        생성 순서가 곧 의존 방향이다: 하드웨어/팔 → 렌더러 → Gemini → 카메라
        → 스캔 → 일정/조명 → 제스처. 어느 것도 전역을 읽지 않고 필요한 것을
        생성자로 받으므로, 나중에 하나만 가짜로 바꿔 끼우기 쉽다.
        """
        # 팔 — 하드웨어가 없어도 모델만으로 계속 동작한다.
        self.arm_service = ArmService()
        self.renderer = ArmRenderer(self.arm_service.arm)

        # AI — GEMINI_API_KEY가 없으면 configured=False로 남고, 관련 기능만 빠진다.
        self.gemini = Gemini()

        # 폰과의 양방향 소켓(프레임·음성·손 랜드마크 ↔ transcript·제스처 명령).
        self.camera = Camera(self.gemini)

        # 문서 스캔 = 검출(OpenCV) + 읽기(Gemini).
        self.scanner = DocumentScanner()
        self.scan_api = ScanAPI(self.camera, self.scanner, self.gemini)

        # 웹 기능 계열.
        self.calendar = Calendar()
        self.light = Light()

        # 제스처 — 카메라 소켓으로 폰에 명령을 보내고, 조명도 함께 끈다.
        self.gesture_bridge = GestureBridge(self.camera, self.light,
                                            enabled=gestures_enabled)

        # 팔 제어 HTTP 라우트.
        self.arm_api = ArmAPI(self.arm_service, self.renderer)

    def _register_routes(self):
        """페이지 두 개와 각 기능 객체의 라우트를 등록한다.

        기능별 라우트는 각 객체의 `register()`가 스스로 붙인다 — 어떤 경로가
        어떤 함수인지는 그 객체 옆에 적혀 있는 편이 찾기 쉽고, 새 기능을
        추가할 때 이 파일을 고칠 필요도 없다.
        """
        self.flask.route("/")(self.index)
        self.flask.route("/mobile")(self.mobile)
        self.flask.route("/calendar", endpoint="calendar_page")(self.calendar_page)

        self.arm_api.register(self.flask)
        self.gemini.register(self.flask)
        self.camera.register(self.flask, self.sock)
        self.scan_api.register(self.flask)
        self.calendar.register(self.flask)
        self.light.register(self.flask)

    # ------------------------------------------------------------------
    # 페이지
    # ------------------------------------------------------------------
    def index(self):
        """GET / — PC용 제어 대시보드(관절 슬라이더, 3D 미리보기, 카메라 화면)."""
        arm = self.arm_service.arm
        return render_template(
            "index.html",
            joint_ids=[joint.id for joint in arm.joints],
            joint_ranges={joint.id: self.arm_service.joint_slider_range(joint)
                          for joint in arm.joints},
            coupled_joints={joint.id: joint.coupled_with for joint in arm.joints
                            if joint.coupled_with is not None},
            hardware_connected=self.arm_service.connected,
            hardware_error=self.arm_service.hardware_error,
        )

    def mobile(self):
        """GET /mobile — 팔에 장착한 휴대폰에서 여는 페이지.

        카메라/마이크 권한을 받아 프레임·음성·손 랜드마크를 소켓으로 보내고,
        서버가 되돌려 주는 transcript와 제스처 명령을 실행한다. 일정·조명·문서
        스캔 UI도 이 페이지에 있다.
        """
        arm = self.arm_service.arm
        return render_template(
            "mobile.html",
            joint_ids=[joint.id for joint in arm.joints],
            hardware_connected=self.arm_service.connected,
            hardware_error=self.arm_service.hardware_error,
            gemini_configured=self.gemini.configured,
            gemini_error=self.gemini.error,
        )

    def calendar_page(self):
        """GET /calendar — 한 달 달력이 화면 전체를 쓰는 일정 화면.

        /mobile의 모달이 아니라 **별도 페이지**인 이유: 좁은 폰 화면에서 달력과
        카메라 영상을 같이 놓으면 둘 다 작아지기만 한다. 대신 이 화면으로
        넘어가면 /mobile이 내려가면서 카메라·웹소켓 세션도 끊기고, 뒤로가기로
        돌아오면 자동으로 다시 켜진다(mobile.html 9번 섹션).

        데이터는 서버가 미리 넣어 주지 않는다 — 페이지가 열린 뒤 자기 시계로
        '이번 달'을 정하고 `/api/calendar/events?from=&to=`로 직접 받아 온다.
        서버와 폰의 시간대가 다를 때 '오늘'이 어긋나지 않게 하려는 것으로,
        음성 명령이 날짜를 해석하는 규칙과 같다.
        """
        return render_template("calendar.html")

    # ------------------------------------------------------------------
    # 실행
    # ------------------------------------------------------------------
    def initialize_position(self):
        """시작할 때 팔을 정해진 자세로 보내고 싶으면 여기에 적는다.

        기본은 아무것도 하지 않는다 — 전원을 켠 순간 팔이 갑자기 움직이면
        위험하고, 무엇보다 하드웨어가 없을 때도 이 경로를 지나야 하기 때문.
        예: self.arm_service.goto_position((0.0, 0.0, 0.34))
        """

    def start_server(self):
        """Flask 개발 서버를 백그라운드 스레드에서 띄운다.

        debug=False     : 리로더가 이 모듈을 자식 프로세스에서 다시 import하면
                          시리얼 포트를 두 번 열게 된다.
        threaded=True   : /ws/camera 소켓이 세션 내내 열려 있으므로, 그 동안에도
                          대시보드의 HTTP 폴링을 받으려면 요청마다 스레드가 필요.
        ssl_context     : getUserMedia는 보안 컨텍스트에서만 동작한다. 폰이 LAN
                          IP로 접속하려면 HTTPS가 필요해 자체 서명 인증서를
                          즉석에서 만든다(브라우저가 한 번 경고를 띄운다).

        `app.run()`은 영원히 블로킹하므로 반드시 별도 스레드여야 한다 — 메인
        스레드는 cv2 창(미리보기 루프)이 써야 하기 때문(macOS 제약).
        """
        thread = threading.Thread(
            target=lambda: self.flask.run(
                host=AppConst.SERVER_HOST,
                port=AppConst.SERVER_PORT,
                debug=False,
                threaded=True,
                ssl_context=AppConst.SSL_CONTEXT,
            ),
            daemon=True,
        )
        thread.start()
        Logger.log("APP", f"Serving on https://{AppConst.SERVER_HOST}:{AppConst.SERVER_PORT}")
        return thread

    def run(self):
        """서버를 띄우고, 메인 스레드에서 인식 루프를 돈다(여기서 블로킹된다)."""
        self.start_server()
        self.initialize_position()
        self.loop.run_forever()


# ----------------------------------------------------------------------
# 모듈 수준 진입점 (main.py / `python -m src.app` 공용)
# ----------------------------------------------------------------------
def create_app(**kwargs) -> DeskyApp:
    """`DeskyApp`을 만들어 돌려준다.

    앱 생성이 import 시점이 아니라 호출 시점에 일어나는 게 중요하다 — 생성자가
    시리얼 포트를 열려고 시도하므로, 이 모듈을 단순히 import했다는 이유만으로
    하드웨어를 잡으면 안 된다(테스트·도구에서 import만 하는 경우가 있다).
    """
    return DeskyApp(**kwargs)


def run(**kwargs):
    """앱을 만들고 실행한다 — main.py가 부르는 함수."""
    create_app(**kwargs).run()


if __name__ == "__main__":
    run()
