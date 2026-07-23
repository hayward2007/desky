"""카메라 프레임에서 종이 문서(사각형)를 여러 개 찾아내는 모듈. (OCR 없음)

검출 파이프라인:
    Canny 엣지 → 모폴로지로 끊긴 테두리 연결 → 윤곽선 검출
    → 면적/사각형성 필터 → 중복 제거 → 큰 순서로 번호 매기기

실제 글자 읽기(OCR)는 여기서 하지 않는다. `four_point_transform()`으로 잘라낸
이미지를 `src.api.gemini.Gemini.parse_document()`에 넘겨서 처리한다.

의존성: opencv-python, numpy
"""

import cv2
import numpy as np

from fundamental.const import DocumentScannerConst
from fundamental.logger import Logger


class Document:
    """감지된 문서 한 장. `id`는 화면에 표시되는 번호(면적이 큰 것부터 1번),
    `quad`는 원본 프레임 좌표계의 꼭짓점 4개 (좌상, 우상, 우하, 좌하 순)."""

    def __init__(self, id: int, quad: np.ndarray):
        """id: 화면 표시 번호, quad: 원본 좌표계의 꼭짓점 4개 (4x2 배열)."""
        self.id = id
        self.quad = quad

    @property
    def area(self) -> float:
        """이 문서 사각형의 넓이(픽셀^2). 문서 정렬 기준으로 쓴다."""
        return float(cv2.contourArea(self.quad.astype(np.float32)))

    @property
    def center(self) -> np.ndarray:
        """사각형 네 꼭짓점의 평균 = 중심점 (x, y). 중복 제거와 번호 표시 위치에 쓴다."""
        return self.quad.astype(np.float32).mean(axis=0)

    def to_dict(self) -> dict:
        """JSON 응답용으로 정수 좌표 리스트로 변환. `/api/scan/detect`가 사용."""
        return {"id": self.id, "quad": self.quad.astype(int).tolist()}


class DocumentScanner:
    """프레임에서 문서를 찾고, 미리보기를 그리고, 선택한 문서를 반듯하게 펴는 객체."""

    # 상수 설명은 fundamental.const.DocumentScannerConst 참고.
    PROC_HEIGHT = DocumentScannerConst.PROC_HEIGHT
    CANNY_LO = DocumentScannerConst.CANNY_LO
    CANNY_HI = DocumentScannerConst.CANNY_HI
    MIN_AREA_RATIO = DocumentScannerConst.MIN_AREA_RATIO
    RECT_MIN = DocumentScannerConst.RECT_MIN
    DEDUP_DIST_RATIO = DocumentScannerConst.DEDUP_DIST_RATIO

    def __init__(self, **overrides):
        """튜닝 파라미터를 키워드 인자로 덮어쓸 수 있다.
        예: `DocumentScanner(MIN_AREA_RATIO=0.01)` → 더 작은 문서까지 감지."""
        for key, value in overrides.items():
            if not hasattr(type(self), key):
                raise ValueError(f"unknown tuning parameter: {key}")
            setattr(self, key, value)

    # ------------------------------------------------------------------
    # 기하 유틸
    # ------------------------------------------------------------------
    @staticmethod
    def order_points(pts) -> np.ndarray:
        """네 점을 항상 [좌상, 우상, 우하, 좌하] 순서로 정렬한다.
        x+y가 최소면 좌상, 최대면 우하 / x-y가 최소면 우상, 최대면 좌하."""
        pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]      # 좌상
        rect[2] = pts[np.argmax(s)]      # 우하
        diff = np.diff(pts, axis=1).ravel()
        rect[1] = pts[np.argmin(diff)]   # 우상
        rect[3] = pts[np.argmax(diff)]   # 좌하
        return rect

    # ------------------------------------------------------------------
    # 검출
    # ------------------------------------------------------------------
    def detect(self, frame) -> list:
        """프레임에서 문서 후보를 모두 찾아 `Document` 리스트로 반환한다.
        면적이 큰 문서부터 id 1, 2, 3... 순으로 번호가 붙는다."""
        ratio = frame.shape[0] / self.PROC_HEIGHT
        small = cv2.resize(frame, (int(frame.shape[1] / ratio), int(self.PROC_HEIGHT)))

        edged = self._edges(small)
        quads = self._quads_from_edges(edged, ratio)
        kept = self._dedup(quads, frame)

        kept.sort(key=lambda q: cv2.contourArea(q.astype(np.float32)), reverse=True)
        return [Document(i + 1, q) for i, q in enumerate(kept)]

    def _edges(self, small):
        """축소한 프레임 → 엣지 이미지. 흑백 변환 + 블러 + Canny 후,
        커널 7x7 dilate/close로 끊어진 문서 테두리를 하나의 닫힌 덩어리로 잇는다."""
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(gray, self.CANNY_LO, self.CANNY_HI)
        k = np.ones((7, 7), np.uint8)
        edged = cv2.dilate(edged, k, iterations=2)
        return cv2.morphologyEx(edged, cv2.MORPH_CLOSE, k, iterations=2)

    def _quads_from_edges(self, edged, ratio) -> list:
        """엣지 이미지의 외곽 윤곽선들을 사각형 4점으로 근사한다.
        1순위는 approxPolyDP로 꼭짓점이 정확히 4개인 볼록 다각형(반듯한 문서),
        2순위는 최소회전사각형(기울거나 살짝 구겨진 문서). 두 경우 모두
        `ratio`를 곱해 원본 프레임 좌표로 되돌린다."""
        contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.MIN_AREA_RATIO * edged.shape[0] * edged.shape[1]

        quads = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                quad = self.order_points(approx.reshape(4, 2))
            else:
                rect = cv2.minAreaRect(c)
                (w, h) = rect[1]
                if w < 1 or h < 1 or area / (w * h) < self.RECT_MIN:
                    continue
                quad = self.order_points(cv2.boxPoints(rect))
            quads.append(quad * ratio)
        return quads

    def _dedup(self, quads, frame) -> list:
        """같은 문서를 안쪽 테두리/바깥쪽 테두리로 두 번 잡은 경우를 걸러낸다.
        이미 채택한 사각형과 중심 거리가 화면 대각선의 DEDUP_DIST_RATIO 미만이면 버린다."""
        diag = float(np.hypot(frame.shape[0], frame.shape[1]))
        min_dist = self.DEDUP_DIST_RATIO * diag
        kept = []
        for q in quads:
            c = q.astype(np.float32).mean(axis=0)
            if all(np.linalg.norm(c - k.astype(np.float32).mean(axis=0)) > min_dist for k in kept):
                kept.append(q)
        return kept

    # ------------------------------------------------------------------
    # 시각화 / 잘라내기
    # ------------------------------------------------------------------
    def draw(self, frame, documents, selected_id=None):
        """감지된 문서들을 사각형 + 번호로 그린 미리보기 프레임을 반환한다.
        `selected_id`와 일치하는 문서만 빨간색으로 강조한다(나머지는 초록)."""
        out = frame.copy()
        for doc in documents:
            q = doc.quad.astype(np.int32)
            selected = (selected_id is not None and doc.id == selected_id)
            color = (0, 0, 255) if selected else (0, 220, 0)
            cv2.polylines(out, [q], True, color, 4 if selected else 3)
            cx, cy = doc.center.astype(int)
            cv2.putText(out, f"#{doc.id}", (int(cx) - 15, int(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, color, 3, cv2.LINE_AA)
        return out

    def four_point_transform(self, frame, quad):
        """문서 네 점을 반듯한 직사각형 이미지로 원근 보정(warp)해서 잘라낸다.
        비스듬히 찍힌 문서도 정면에서 본 것처럼 펴지므로 Gemini의 글자 인식률이 올라간다."""
        rect = self.order_points(quad)
        (tl, tr, br, bl) = rect
        max_w = max(int(max(np.linalg.norm(br - bl), np.linalg.norm(tr - tl))), 1)
        max_h = max(int(max(np.linalg.norm(tr - br), np.linalg.norm(tl - bl))), 1)
        dst = np.array([[0, 0], [max_w - 1, 0], [max_w - 1, max_h - 1], [0, max_h - 1]], np.float32)
        M = cv2.getPerspectiveTransform(rect, dst)
        Logger.log("DOCSCAN", f"crop {max_w}x{max_h}")
        return cv2.warpPerspective(frame, M, (max_w, max_h))
