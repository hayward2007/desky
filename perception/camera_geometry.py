"""end-effector에 장착된 카메라 기준 로컬 좌표축 정의.

HandTracker와 FaceTracker 둘 다 쓰는 핀홀 투영 지오메트리(카메라가 보는
방향과 나머지 두 화면 축)를 한 곳에 모아, 두 트래커가 같은 정의를 공유하게
한다 — 한쪽만 고치고 다른 쪽을 깜빡하는 실수를 막기 위함. 둘 다 이제 인식
자체는 휴대폰(MediaPipe Tasks Vision)이 하고, 서버는 그 결과(랜드마크 좌표)를
받아 여기 정의된 지오메트리로 3D 월드 변환만 한다 — `to_landmark()`가 그
공통 입력 파싱(휴대폰이 보낸 `[x,y,z]`/`{"x","y","z"}`)도 같이 맡는다.
"""

import math
from collections import namedtuple

from fundamental.const import CameraGeometryConst

# IK 타겟의 좌우/앞뒤(x, y) 성분에 적용하는 한계(m) — 설명은
# fundamental.const.CameraGeometryConst 참고.
IK_XY_LIMIT_M = CameraGeometryConst.IK_XY_LIMIT_M

Landmark = namedtuple("Landmark", ["x", "y", "z"])


def to_landmark(point) -> Landmark:
    """휴대폰이 보낸 랜드마크 하나를 `Landmark`(x, y, z)로 정규화한다.

    `[x, y, z]`(리스트/튜플, MediaPipe Tasks Vision JS가 보내는 형태를 그대로
    JSON 배열로 옮긴 것)와 `{"x":.., "y":.., "z":..}`(딕셔너리) 둘 다 받는다.
    HandTracker/FaceTracker 둘 다 서버에서 더 이상 인식을 하지 않으므로(휴대폰이
    한다), 랜드마크를 그 트래커들이 원래 쓰던 .x/.y/.z 속성 접근 코드에 맞게
    감싸는 이 변환이 둘의 유일한 "인식 결과 수신" 지점이다.
    """
    if isinstance(point, dict):
        return Landmark(float(point["x"]), float(point["y"]), float(point.get("z", 0.0)))
    x, y, z = point[0], point[1], (point[2] if len(point) > 2 else 0.0)
    return Landmark(float(x), float(y), float(z))


def clamp_xy(position, limit=IK_XY_LIMIT_M):
    """(x, y, z) 타겟의 x, y만 ±limit(m)로 clamp하고 z는 그대로 둔다."""
    x, y, z = position
    return (max(-limit, min(limit, x)), max(-limit, min(limit, y)), z)


def camera_frame(T_ee):
    """T_ee(Arm.fk_matrix(q))에서 카메라 기준 로컬 축 3개와 원점을 뽑아낸다.

    forward_axis(로컬 +Y)가 카메라가 실제로 보는 방향 — 실측으로 확인: 모든
    관절이 서보각 180도일 때 이 벡터가 world +Z를 가리켜야 함. up_axis(로컬
    +X), side_axis(로컬 +Z)는 화면 오프셋을 얹는 나머지 두 축.
    """
    forward_axis = (T_ee[0][1], T_ee[1][1], T_ee[2][1])
    up_axis = (T_ee[0][0], T_ee[1][0], T_ee[2][0])
    side_axis = (T_ee[0][2], T_ee[1][2], T_ee[2][2])
    origin = (T_ee[0][3], T_ee[1][3], T_ee[2][3])
    return forward_axis, up_axis, side_axis, origin


def to_pixel(landmark, width, height, margin=1.0):
    """정규화 좌표(0~1)를 화면 픽셀 (x, y) 정수로 바꾼다. 그릴 수 없으면 None.

    미리보기에 무언가를 그리기 전에 **반드시** 이걸 거쳐야 한다. 그냥
    `int(lm.x * w)`로 바꿔 넘기면 OpenCV가 이런 오류를 내며 터진다:

        Can't parse 'pt2'. Sequence item with index 0 has a wrong type

    "타입이 틀렸다"는 메시지지만 실제 원인은 대개 **값이 int32 범위를 벗어난
    것**이다. OpenCV의 좌표는 32비트 정수라 아주 큰 수를 넘기면 변환 자체가
    실패하고, 파이썬 int라서 타입은 맞는데도 위 문구가 나온다. 인식이 순간적으로
    엉뚱한 값을 뱉거나(가림·역광·프레임 손상) 폰이 이상한 값을 보내면 바로
    이 상황이 된다.

    그래서 두 가지를 한다:
      · 유한한 숫자가 아니면(NaN, 무한대) 그리지 않는다 — None을 돌려준다.
      · 화면 밖으로 나간 값은 화면 크기의 `margin`배만큼 여유를 둔 선에서
        자른다. 완전히 화면 안으로 욱여넣지 않는 이유는, 얼굴이 가장자리를
        살짝 벗어났을 때 선이 바깥을 향하는 모습이 그대로 보이는 편이
        디버깅에 유용하기 때문이다.
    """
    try:
        x = float(landmark.x) * width
        y = float(landmark.y) * height
    except (TypeError, ValueError, AttributeError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    x_limit, y_limit = width * (1.0 + margin), height * (1.0 + margin)
    return (int(max(-x_limit, min(x_limit, x))),
            int(max(-y_limit, min(y_limit, y))))
