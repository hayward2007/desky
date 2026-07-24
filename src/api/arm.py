"""팔 제어 HTTP 라우트 묶음 — 상태 조회, FK/IK 미리보기, 실제 이동.

[병합 메모] 병합 전에는 이 라우트들이 `src/app.py`에 `@app.route` 데코레이터가
붙은 모듈 전역 함수로 흩어져 있었고, 전역 `arm_ctrl`/`arm`을 직접 참조했다.
두 브랜치가 같은 파일의 같은 영역에 서로 다른 라우트를 추가해 충돌이 컸던
자리라, 다른 기능 객체(Gemini/Camera/ScanAPI/Calendar/Light)와 똑같이
"객체 하나 + `register(app)`" 형태로 통일했다.

역할 분담:
- 이 클래스는 **HTTP만** 안다 — 요청 본문 검증, 상태코드, jsonify.
- 실제 상태와 안전 검사는 전부 `src.arm_service.ArmService`가 한다.
- 3D 렌더링은 `src.render.ArmRenderer`가 한다.

라우트 목록:
    GET  /api/status         현재 연결 상태와 end-effector 위치
    POST /api/fk             서보각 → 위치 (움직이지 않음)
    POST /api/ik             위치 → 서보각 (움직이지 않음)
    POST /api/render         서보각 → 팔 그림 PNG (움직이지 않음)
    POST /api/goto_position  IK로 목표 위치까지 실제 이동
    POST /api/goto_joints    모든 관절을 지정 서보각으로 실제 이동
    POST /api/goto_joint     관절 하나만 실제 이동
"""

from flask import Response, jsonify, request

from src.arm_service import HardwareUnavailable


class ArmAPI:
    """`ArmService`/`ArmRenderer`를 HTTP로 노출하는 라우트 묶음."""

    def __init__(self, arm_service, renderer):
        self.arm_service = arm_service
        self.renderer = renderer

    # ------------------------------------------------------------------
    # 공통 도우미
    # ------------------------------------------------------------------
    @property
    def arm(self):
        """팔 모델(관절 목록/FK/IK) 바로가기."""
        return self.arm_service.arm

    def _parse_degrees(self, data):
        """관절 순서대로 서보각 하나씩 담긴 본문을 검증한다.

        (degrees, error_response)를 돌려주며 둘 중 정확히 하나만 None이 아니다 —
        여러 라우트가 같은 검증을 반복하지 않도록 뽑아 둔 것.
        """
        degs = data.get("degrees") if data else None
        if not isinstance(degs, list) or len(degs) != len(self.arm.joints):
            return None, (jsonify({
                "error": f"degrees must be a list of {len(self.arm.joints)} numbers"
            }), 400)
        try:
            return [float(d) for d in degs], None
        except (TypeError, ValueError):
            return None, (jsonify({"error": "degrees must be numbers"}), 400)

    @staticmethod
    def _parse_xyz(data):
        """본문에서 목표 좌표 (x, y, z)를 꺼낸다. 형식이 틀리면 (None, 응답)."""
        try:
            return (float(data["x"]), float(data["y"]), float(data["z"])), None
        except (KeyError, TypeError, ValueError):
            return None, (jsonify({"error": "x, y, z must be numbers"}), 400)

    # ------------------------------------------------------------------
    # 조회 / 미리보기 (하드웨어 없이도 동작)
    # ------------------------------------------------------------------
    def status(self):
        """GET /api/status — 하드웨어 연결 여부와 현재 end-effector 위치."""
        if not self.arm_service.connected:
            return jsonify({
                "connected": False,
                "position": None,
                "error": self.arm_service.hardware_error,
            })
        return jsonify({"connected": True, "position": self.arm_service.position()})

    def fk(self):
        """POST /api/fk — 순기구학만. 주어진 서보각의 위치를 **움직이지 않고** 계산.

        하드웨어가 없어도 동작하므로, 대시보드가 명령을 확정하기 전에 결과를
        미리 볼 수 있다. 한계 검사 결과도 함께 돌려준다.
        """
        degs, err = self._parse_degrees(request.get_json(force=True))
        if err:
            return err
        q = self.arm.servo_deg_to_q(degs)
        return jsonify({
            "position": list(self.arm.fk(q)),
            "within_limits": self.arm_service.servo_degs_within_limits(degs) is None,
        })

    def ik(self):
        """POST /api/ik — 목표 (x, y, z)의 역기구학 해를 서보각으로 돌려준다.

        아무것도 움직이지 않는다(웹 시뮬레이터의 미리보기용). `seed`(선택)는
        현재 서보각으로, IK의 시작 추정값으로 쓰인다 — 5자유도 팔이 3자유도
        위치 목표에 대해 여러 해를 갖기 때문에, 지금 자세에서 가까운 해를
        고르게 하는 힌트다.
        """
        data = request.get_json(force=True)
        target, err = self._parse_xyz(data)
        if err:
            return err

        seed = None
        raw_seed = data.get("seed") if data else None
        if isinstance(raw_seed, list) and len(raw_seed) == len(self.arm.joints):
            try:
                seed = self.arm.servo_deg_to_q([float(d) for d in raw_seed])
            except (TypeError, ValueError):
                seed = None

        q, converged = self.arm.ik(target, seed=seed)
        return jsonify({"converged": converged, "servo_deg": self.arm.q_to_servo_deg(q)})

    def render(self):
        """POST /api/render — 주어진 서보각의 팔을 PNG로 그려 돌려준다.

        데스크톱 시뮬레이션과 **같은** 그리기 코드를 서버에서 돌린다
        (`src.render.ArmRenderer`). 하드웨어가 필요 없다.
        """
        degs, err = self._parse_degrees(request.get_json(force=True))
        if err:
            return err
        png = self.renderer.render_png(self.arm.servo_deg_to_q(degs))
        return Response(png, mimetype="image/png")

    # ------------------------------------------------------------------
    # 실제 이동 (하드웨어 필요)
    # ------------------------------------------------------------------
    def goto_position(self):
        """POST /api/goto_position — IK로 목표 (x, y, z)까지 실제로 이동."""
        data = request.get_json(force=True)
        target, err = self._parse_xyz(data)
        if err:
            return err
        try:
            q, converged = self.arm_service.goto_position(target)
        except HardwareUnavailable as e:
            return jsonify({"error": "no hardware connected", "detail": str(e)}), 503
        servo_deg = self.arm.q_to_servo_deg(q) if converged else None
        return jsonify({"converged": converged, "servo_deg": servo_deg})

    def goto_joints(self):
        """POST /api/goto_joints — 모든 관절을 지정 서보각으로 동시에 이동.

        움직이기 전에 자기충돌 안전 범위를 검사한다. 결합 관절(joint2)의 한계는
        이번에 함께 명령되는 값들로 풀린다(`ArmService.servo_degs_within_limits`).
        """
        degs, err = self._parse_degrees(request.get_json(force=True))
        if err:
            return err

        limit_error = self.arm_service.servo_degs_within_limits(degs)
        if limit_error:
            return jsonify({"error": limit_error}), 400

        try:
            pos = self.arm_service.goto_joints(degs)
        except HardwareUnavailable as e:
            return jsonify({"error": "no hardware connected", "detail": str(e)}), 503
        except ValueError as e:      # 예: 서보각이 0~300 밖
            return jsonify({"error": str(e)}), 400
        return jsonify({"position": list(pos)})

    def goto_joint(self):
        """POST /api/goto_joint — 관절 하나만 이동 {id, degree}.

        이 관절만 움직여도 결합 관절의 안전 범위는 나머지 관절의 현재 위치에
        따라 달라지므로, 현재 자세에 이 값만 갈아끼운 벡터로 검사한다.
        """
        data = request.get_json(force=True)
        try:
            joint_id = int(data["id"])
            degree = float(data["degree"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "id and degree must be numbers"}), 400

        if joint_id not in self.arm.id_to_index:
            return jsonify({"error": f"no joint with id {joint_id}"}), 404

        limit_error = self.arm_service.servo_degs_within_limits(
            self.arm_service.servo_degs_with_one_changed(joint_id, degree))
        if limit_error:
            return jsonify({"error": limit_error}), 400

        try:
            self.arm_service.goto_joint(joint_id, degree)
        except HardwareUnavailable as e:
            return jsonify({"error": "no hardware connected", "detail": str(e)}), 503
        except KeyError:
            return jsonify({"error": f"no actuator with id {joint_id}"}), 404
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        return jsonify({"ok": True})

    # ------------------------------------------------------------------
    # 등록
    # ------------------------------------------------------------------
    def register(self, app):
        """이 객체가 담당하는 라우트를 Flask 앱에 붙인다."""
        app.route("/api/status", endpoint="status")(self.status)
        app.route("/api/fk", methods=["POST"], endpoint="fk")(self.fk)
        app.route("/api/ik", methods=["POST"], endpoint="ik")(self.ik)
        app.route("/api/render", methods=["POST"], endpoint="render")(self.render)
        app.route("/api/goto_position", methods=["POST"], endpoint="goto_position")(self.goto_position)
        app.route("/api/goto_joints", methods=["POST"], endpoint="goto_joints")(self.goto_joints)
        app.route("/api/goto_joint", methods=["POST"], endpoint="goto_joint")(self.goto_joint)
