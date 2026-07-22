"""Flask control dashboard for the desky arm.

Enter a target (x, y, z) position or a per-joint servo degree in the browser
and drive the real actuators through hardware.util.ArmController. Requires
actual hardware — Controller() opens a serial port on import, same as
main.py — and a configured .env (see README.md).

Run from the repository root:
    python3 -m webapp.app
"""

from flask import Flask, jsonify, render_template, request

from hardware.controller import Controller
from hardware.util import Actuator, ArmController
from logger import Logger

Logger.enabled = True

app = Flask(__name__)

controller = Controller()
actuators = [Actuator(id=i, model="AX-18A", controller=controller) for i in range(1, 6)]
arm_ctrl = ArmController(actuators)


@app.route("/")
def index():
    return render_template("index.html", joint_ids=[joint.id for joint in arm_ctrl.arm.joints])


@app.route("/api/status")
def status():
    position = arm_ctrl.get_position()
    return jsonify({"position": position})


@app.route("/api/goto_position", methods=["POST"])
def goto_position():
    data = request.get_json(force=True)
    try:
        target = (float(data["x"]), float(data["y"]), float(data["z"]))
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "x, y, z must be numbers"}), 400

    Logger.log("WEBAPP", f"goto_position request: target={target}")
    q, converged = arm_ctrl.goto_position(target)
    servo_deg = arm_ctrl.arm.q_to_servo_deg(q) if converged else None
    return jsonify({"converged": converged, "servo_deg": servo_deg})


@app.route("/api/goto_joint", methods=["POST"])
def goto_joint():
    data = request.get_json(force=True)
    try:
        joint_id = int(data["id"])
        degree = float(data["degree"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "id and degree must be numbers"}), 400

    actuator = next((a for a in arm_ctrl.actuators if a.id == joint_id), None)
    if actuator is None:
        return jsonify({"error": f"no actuator with id {joint_id}"}), 404

    Logger.log("WEBAPP", f"goto_joint request: id={joint_id} degree={degree}")
    try:
        actuator.goto(degree)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    return jsonify({"ok": True})


if __name__ == "__main__":
    # debug=False: the Flask reloader re-imports this module in a subprocess,
    # which would open the serial port twice.
    app.run(host="0.0.0.0", port=5000, debug=False)
