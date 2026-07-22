"""3D preview of the desky arm — no hardware required.

Uses the exact same geometry source as the real robot (kinematics/desky.urdf,
via kinematics.urdf_loader.load_arm) and the exact same motion waypoints the
real robot is commanded through (scenario.DEMO_SEQUENCE, shared with main.py),
so the animation matches what main.py would do on physical hardware.

Requires matplotlib (not a dependency of the kinematics package itself):
    pip install matplotlib
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from kinematics.urdf_loader import load_arm
from logger import Logger
from scenario import DEMO_SEQUENCE

FRAMES_PER_STEP = 30  # animation smoothness; independent of the real `hold` durations


def _interp(a, b, t):
    return [ai + (bi - ai) * t for ai, bi in zip(a, b)]


def build_waypoints(arm):
    """Turn DEMO_SEQUENCE (servo-degree waypoints, same as main.py) into a
    list of joint-angle vectors q (radians), starting from the home pose."""
    current_servo = {joint.id: joint.home_deg for joint in arm.joints}
    waypoints = [arm.servo_deg_to_q([current_servo[j.id] for j in arm.joints])]
    for step in DEMO_SEQUENCE:
        current_servo.update(step["positions"])
        waypoints.append(arm.servo_deg_to_q([current_servo[j.id] for j in arm.joints]))
    return waypoints


def main():
    arm = load_arm()
    waypoints = build_waypoints(arm)
    segments = list(zip(waypoints[:-1], waypoints[1:]))
    total_frames = max(FRAMES_PER_STEP * len(segments), 1)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    all_points = [p for q in waypoints for p in arm.fk_all(q)]
    xs, ys, zs = zip(*all_points)
    margin = 0.05
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_zlim(0, max(zs) + margin)

    line, = ax.plot([], [], [], "o-", linewidth=3, markersize=6)

    def update(frame):
        if not segments:
            q = waypoints[0]
        else:
            seg_idx = min(frame // FRAMES_PER_STEP, len(segments) - 1)
            t = (frame % FRAMES_PER_STEP) / max(FRAMES_PER_STEP - 1, 1)
            q = _interp(*segments[seg_idx], t)
        points = arm.fk_all(q)
        px, py, pz = zip(*points)
        line.set_data(px, py)
        line.set_3d_properties(pz)
        return (line,)

    Logger.log("SIM", f"Animating {len(waypoints)} waypoint(s) from DEMO_SEQUENCE")
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False, repeat=False)
    plt.show()


if __name__ == "__main__":
    main()
