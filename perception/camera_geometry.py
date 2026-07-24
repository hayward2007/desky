"""end-effector에 장착된 카메라 기준 로컬 좌표축 정의.

HandTracker와 FaceTracker 둘 다 쓰는 핀홀 투영 지오메트리(카메라가 보는
방향과 나머지 두 화면 축)를 한 곳에 모아, 두 트래커가 같은 정의를 공유하게
한다 — 한쪽만 고치고 다른 쪽을 깜빡하는 실수를 막기 위함. 둘 다 이제 인식
자체는 휴대폰(MediaPipe Tasks Vision)이 하고, 서버는 그 결과(랜드마크 좌표)를
받아 여기 정의된 지오메트리로 3D 월드 변환만 한다 — `to_landmark()`가 그
공통 입력 파싱(휴대폰이 보낸 `[x,y,z]`/`{"x","y","z"}`)도 같이 맡는다.
"""

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
