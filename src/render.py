"""팔의 3D 그림을 만드는 두 객체 — 웹용 PNG와 로컬 미리보기 창.

[병합 메모] 병합 전에는 `src/app.py` 모듈 전역(`_root_link`, `_chain`,
`_visuals`, `_render_bounds`)과 `run()` 함수 안의 지역 변수(`fig3d`,
`canvas3d`, `ax3d`)로 흩어져 있었다. 두 브랜치가 이 지역 변수들 사이에 서로
다른 그리기 코드를 끼워 넣어 충돌했기 때문에(develop은 얼굴+손을 트래커에게
맡겨 그리고, mobile 브랜치는 루프 안에서 직접 quiver/draw_points를 호출),
"URDF를 한 번 파싱해 두고 계속 그리는 물건"을 아래 두 클래스로 뽑아 정리했다.

- `ArmRenderer` : URDF 기하와 화면 범위를 한 번만 파싱해 들고, 자세 하나를
                  PNG 바이트로 렌더링한다(웹 `/api/render`).
- `ScenePreview`: 창 하나를 계속 다시 그리는 상태 객체(로컬 3D 미리보기).
                  matplotlib figure를 매번 새로 만들지 않고 재사용한다.

둘 다 Agg(화면 없는) 백엔드로 그린다 — 실제 창 띄우기는 cv2가 맡는다.
matplotlib의 GUI 백엔드를 쓰면 OpenCV 창과 별개의 이벤트 루프가 필요해지고,
macOS에서는 그 루프가 메인 스레드를 요구해 서로 부딪힌다.
"""

import io

import matplotlib
matplotlib.use("Agg")  # headless: 창 없이 이미지로만 렌더링
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from kinematics.simulate import parse_urdf, draw_pose, workspace_bounds
from kinematics.urdf_loader import _DEFAULT_URDF_PATH


class ArmRenderer:
    """URDF 기하를 한 번 파싱해 두고 자세를 그림으로 만드는 객체."""

    def __init__(self, arm, urdf_path=_DEFAULT_URDF_PATH, elev=22, azim=-55):
        """URDF를 파싱하고 작업공간 범위를 계산해 둔다(생성 시 1회).

        범위(`bounds`)를 미리 잡아 두는 이유: 자세마다 축 범위를 다시 맞추면
        팔이 움직일 때 화면이 같이 확대/축소돼 보기 어렵다. 고정 범위여야
        "팔이 움직인다"로 보인다.
        """
        self.arm = arm
        self.root_link, self.chain, self.visuals = parse_urdf(urdf_path)
        self.bounds = workspace_bounds(arm, self.root_link, self.chain, self.visuals)
        self.elev = elev
        self.azim = azim

    def draw_into(self, ax, q):
        """이미 있는 3D 축에 주어진 자세의 팔을 그린다(축은 지우고 다시 그림)."""
        draw_pose(ax, self.arm, self.root_link, self.chain, self.visuals, q, self.bounds)

    def render_png(self, q, figsize=(6, 6), dpi=90) -> bytes:
        """자세 하나를 PNG 바이트로 렌더링한다(`/api/render`가 그대로 응답).

        요청마다 새 Figure를 만든다 — Flask가 threaded=True로 돌아 여러 요청이
        동시에 들어올 수 있는데, matplotlib Figure 하나를 여러 스레드가 공유하면
        서로의 그림을 덮어쓴다. 아래 `ScenePreview`가 figure를 재사용하는 것과
        대비되는 지점(그쪽은 단일 스레드 루프 전용).
        """
        fig = Figure(figsize=figsize)
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
        ax.view_init(elev=self.elev, azim=self.azim)
        self.draw_into(ax, q)

        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=dpi)
        return buf.getvalue()

    def arrow_length(self, ratio=0.25):
        """작업공간 크기에 비례한 화살표 길이 — 카메라 방향 디버그 표시용."""
        return self.bounds[3] * ratio


class ScenePreview:
    """로컬 3D 미리보기 창 하나를 계속 다시 그리는 상태 객체.

    Figure/Canvas/Axes를 생성 시 한 번만 만들고 프레임마다 재사용한다 — 이
    창은 인식 루프에서 반복 호출되므로 매번 새로 만들면 그 비용만으로 루프가
    느려진다(그래서 호출부도 별도 간격으로 throttle한다:
    fundamental.const.AppConst.VIS_MIN_INTERVAL_S).
    """

    def __init__(self, renderer, figsize=(6, 6)):
        self.renderer = renderer
        self.fig = Figure(figsize=figsize)
        self.canvas = FigureCanvasAgg(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d", computed_zorder=False)
        self.ax.view_init(elev=renderer.elev, azim=renderer.azim)

    def draw(self, q, T_ee, hands, faces, hand_tracker, face_tracker):
        """팔 + 손 + 얼굴을 한 장으로 그려 BGR 이미지(numpy)로 돌려준다.

        q            : 현재 관절각(rad)
        T_ee         : end-effector 4x4 월드 변환(카메라 위치/방향)
        hands, faces : 이번 프레임의 인식 결과
        hand_tracker, face_tracker: 각자의 3D 그리기 메서드를 가진 트래커

        그리는 주체를 트래커에게 맡기는 이유: "손 골격을 어떻게 잇는지",
        "얼굴 중심을 어떤 색 점으로 찍는지"는 그 트래커가 만든 자료구조를 가장
        잘 아는 쪽이 정해야 한다. 이 클래스는 순서와 캔버스만 관리한다.
        """
        self.renderer.draw_into(self.ax, q)
        # 카메라가 본다고 가정하는 방향(청록 화살표) — 이 가정이 실제와
        # 어긋나면 손/얼굴이 엉뚱한 위치에 찍히므로 눈으로 확인할 수 있게 둔다.
        hand_tracker.draw_forward_axis_debug(self.ax, T_ee, self.renderer.arrow_length())
        hand_tracker.draw_hands_3d(self.ax, hands)
        face_tracker.draw_faces_3d(self.ax, faces)
        self.canvas.draw()
        return self._to_bgr()

    def _to_bgr(self):
        """캔버스의 RGBA 버퍼를 cv2가 쓰는 BGR 배열로 바꾼다."""
        import cv2

        return cv2.cvtColor(np.asarray(self.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR)
