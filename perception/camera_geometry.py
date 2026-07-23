"""end-effector에 장착된 카메라 기준 로컬 좌표축 정의.

HandTracker와 FaceTracker 둘 다 쓰는 핀홀 투영 지오메트리(카메라가 보는
방향과 나머지 두 화면 축)를 한 곳에 모아, 두 트래커가 같은 정의를 공유하게
한다 — 한쪽만 고치고 다른 쪽을 깜빡하는 실수를 막기 위함.
"""


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
