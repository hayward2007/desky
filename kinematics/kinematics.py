"""Forward and inverse kinematics for the desky 5-DOF arm.

Joint configuration (confirmed with the build):
    id1 = yaw   (rotation about the base vertical axis, Z)
    id2 = roll  (rotation about the radial/forward axis, X)
    id3 = pitch (rotation about Y)
    id4 = pitch (rotation about Y)
    id5 = pitch (rotation about Y)

The 5 actuators give 5 DOF, so the arm can reach a 3D position plus a partial
orientation (it cannot hit an arbitrary full 6-DOF pose). The IK solver below
uses a damped least-squares iteration, which handles both the redundancy
(position-only targets) and the 5-DOF orientation deficiency gracefully.

Pure Python (math only) — no numpy dependency, so it runs anywhere the rest of
the project runs.

------------------------------------------------------------------------------
CONFIG: link geometry is not measured yet. Edit the `*_MM` / offset constants
and the servo mapping (`home_deg`, `direction`, limits) in DEFAULT_JOINTS once
you have the real numbers. FK/IK adapt automatically to whatever you set.
------------------------------------------------------------------------------
"""

import math

from logger import Logger

# ---------------------------------------------------------------------------
# Link geometry — PLACEHOLDERS. Replace with measured values (any consistent
# length unit; mm assumed). Each offset is the translation from the previous
# joint's frame to this joint's origin, expressed in the previous frame BEFORE
# this joint rotates.
# ---------------------------------------------------------------------------
BASE_HEIGHT_MM = 50.0   # base bottom -> yaw joint (along Z)
RISER_MM       = 30.0   # yaw joint  -> roll joint (along Z)
SHOULDER_MM    = 40.0   # roll joint -> first pitch joint (id3), along Z
UPPER_ARM_MM   = 120.0  # id3 -> id4, along local X
FOREARM_MM     = 100.0  # id4 -> id5, along local X
TOOL_MM        = 80.0   # id5 -> phone mount (end-effector), along local X


# ---------------------------------------------------------------------------
# Small linear-algebra helpers (4x4 homogeneous transforms + generic NxN).
# Matrices are lists of row-lists.
# ---------------------------------------------------------------------------
def _identity4():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _matmul(A, B):
    n, m, p = len(A), len(B), len(B[0])
    out = [[0.0] * p for _ in range(n)]
    for i in range(n):
        Ai = A[i]
        for k in range(m):
            a = Ai[k]
            if a == 0.0:
                continue
            Bk = B[k]
            oi = out[i]
            for j in range(p):
                oi[j] += a * Bk[j]
    return out


def _transpose(A):
    return [[A[i][j] for i in range(len(A))] for j in range(len(A[0]))]


def _translate(x, y, z):
    T = _identity4()
    T[0][3], T[1][3], T[2][3] = x, y, z
    return T


def _rot_axis(axis, theta):
    """4x4 rotation of `theta` radians about an arbitrary unit axis (Rodrigues).

    `axis` is a 3-vector; it is normalized here. This matches URDF, where each
    joint rotates about an arbitrary <axis xyz="..."/>.
    """
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("Zero-length rotation axis")
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    T = _identity4()
    T[0][0], T[0][1], T[0][2] = c + x * x * C,     x * y * C - z * s, x * z * C + y * s
    T[1][0], T[1][1], T[1][2] = y * x * C + z * s, c + y * y * C,     y * z * C - x * s
    T[2][0], T[2][1], T[2][2] = z * x * C - y * s, z * y * C + x * s, c + z * z * C
    return T


def _rpy(roll, pitch, yaw):
    """4x4 fixed rotation from URDF rpy (extrinsic X-Y-Z, i.e. Rz*Ry*Rx)."""
    return _matmul(_matmul(_rot_axis((0, 0, 1), yaw), _rot_axis((0, 1, 0), pitch)),
                   _rot_axis((1, 0, 0), roll))


def _inverse(M):
    """Gauss-Jordan inverse of a square matrix (used for the small DLS system)."""
    n = len(M)
    A = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(M)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[pivot][col]) < 1e-12:
            raise ValueError("Singular matrix in DLS solve")
        A[col], A[pivot] = A[pivot], A[col]
        piv = A[col][col]
        A[col] = [v / piv for v in A[col]]
        for r in range(n):
            if r == col:
                continue
            factor = A[r][col]
            if factor != 0.0:
                A[r] = [a - factor * b for a, b in zip(A[r], A[col])]
    return [row[n:] for row in A]


# ---------------------------------------------------------------------------
# Joint model
# ---------------------------------------------------------------------------
# Axis unit vectors for the three joint roles (URDF <axis xyz="..."/> convention).
YAW = (0.0, 0.0, 1.0)    # about Z
ROLL = (1.0, 0.0, 0.0)   # about X
PITCH = (0.0, 1.0, 0.0)  # about Y


class Joint:
    """One revolute joint (mirrors a URDF <joint type="revolute">).

    axis      : (x, y, z) unit rotation axis in the joint frame (YAW/ROLL/PITCH)
    offset    : (x, y, z) origin translation from the parent joint frame
    rpy       : (roll, pitch, yaw) fixed origin rotation (URDF origin rpy)
    home_deg  : servo angle in [0, 300] that corresponds to joint angle q = 0
    direction : +1 or -1; servo_deg = home_deg + direction * degrees(q)
    q_min/q_max : joint limits in radians (relative to home)
    name      : URDF joint name (for reference)
    """

    def __init__(self, id, axis, offset, rpy=(0.0, 0.0, 0.0),
                 home_deg=150.0, direction=1,
                 q_min=-math.pi / 2, q_max=math.pi / 2, name=None):
        self.id = id
        self.axis = axis
        self.offset = offset
        self.rpy = rpy
        self.home_deg = home_deg
        self.direction = direction
        self.q_min = q_min
        self.q_max = q_max
        self.name = name

    # --- conversions between joint angle q (rad) and servo angle (deg, 0..300) ---
    def servo_deg(self, q):
        return self.home_deg + self.direction * math.degrees(q)

    def q_from_servo(self, servo_deg):
        return self.direction * math.radians(servo_deg - self.home_deg)

    def clamp(self, q):
        return max(self.q_min, min(self.q_max, q))


# Default chain matching id1=yaw, id2=roll, id3/4/5=pitch.
# home_deg / direction / limits are placeholders — calibrate against the real arm.
DEFAULT_JOINTS = [
    Joint(id=1, axis=YAW,   offset=(0.0, 0.0, BASE_HEIGHT_MM)),
    Joint(id=2, axis=ROLL,  offset=(0.0, 0.0, RISER_MM)),
    Joint(id=3, axis=PITCH, offset=(0.0, 0.0, SHOULDER_MM)),
    Joint(id=4, axis=PITCH, offset=(UPPER_ARM_MM, 0.0, 0.0)),
    Joint(id=5, axis=PITCH, offset=(FOREARM_MM, 0.0, 0.0)),
]
TOOL_OFFSET = (TOOL_MM, 0.0, 0.0)  # id5 frame -> phone mount


class Arm:
    """5-DOF arm kinematics. `q` is always a list of 5 joint angles in radians."""

    def __init__(self, joints=None, tool_offset=TOOL_OFFSET):
        self.joints = joints if joints is not None else DEFAULT_JOINTS
        self.tool_offset = tool_offset

    # ---------------- Forward kinematics ----------------
    def fk_matrix(self, q):
        """Return the 4x4 end-effector pose given joint angles q (radians)."""
        T = _identity4()
        for joint, qi in zip(self.joints, q):
            T = _matmul(T, _translate(*joint.offset))
            if joint.rpy != (0.0, 0.0, 0.0):
                T = _matmul(T, _rpy(*joint.rpy))
            T = _matmul(T, _rot_axis(joint.axis, qi))
        T = _matmul(T, _translate(*self.tool_offset))
        return T

    def fk(self, q):
        """Return just the end-effector position (x, y, z)."""
        T = self.fk_matrix(q)
        return (T[0][3], T[1][3], T[2][3])

    # ---------------- Inverse kinematics ----------------
    def ik(self, target_pos, target_rot=None, seed=None,
           max_iter=200, tol=1e-3, damping=0.05):
        """Solve joint angles that place the end-effector at target_pos.

        target_pos : (x, y, z) desired position.
        target_rot : optional 3x3 desired orientation (list of rows). If given,
                     orientation is included in the objective (best-effort, since
                     5 DOF cannot satisfy a full 6-DOF pose exactly).
        seed       : starting joint angles (radians); defaults to all-zero.
        Returns (q, converged): the joint angles and whether tol was reached.

        Uses damped least squares:  dq = J^T (J J^T + λ² I)^-1  e
        """
        q = list(seed) if seed is not None else [0.0] * len(self.joints)
        n = len(self.joints)
        lam2 = damping * damping

        for _ in range(max_iter):
            T = self.fk_matrix(q)
            e = self._pose_error(T, target_pos, target_rot)
            if _norm(e) < tol:
                return q, True

            J = self._jacobian(q, use_rot=target_rot is not None)
            # dq = J^T (J J^T + λ² I)^-1 e
            JJt = _matmul(J, _transpose(J))
            for i in range(len(JJt)):
                JJt[i][i] += lam2
            inv = _inverse(JJt)
            Jt = _transpose(J)
            tmp = [sum(inv[i][k] * e[k] for k in range(len(e))) for i in range(len(inv))]
            dq = [sum(Jt[i][k] * tmp[k] for k in range(len(tmp))) for i in range(n)]

            for i in range(n):
                q[i] = self.joints[i].clamp(q[i] + dq[i])

        T = self.fk_matrix(q)
        return q, _norm(self._pose_error(T, target_pos, target_rot)) < tol

    # ---------------- helpers ----------------
    def _pose_error(self, T, target_pos, target_rot):
        pos_err = [target_pos[0] - T[0][3],
                   target_pos[1] - T[1][3],
                   target_pos[2] - T[2][3]]
        if target_rot is None:
            return pos_err
        # Orientation error via the axis-angle of R_target * R_current^T.
        R = [[T[i][j] for j in range(3)] for i in range(3)]
        Rt = target_rot
        Re = _matmul(Rt, _transpose(R))  # 3x3
        angle = math.acos(max(-1.0, min(1.0, (Re[0][0] + Re[1][1] + Re[2][2] - 1.0) / 2.0)))
        if abs(angle) < 1e-9:
            rot_err = [0.0, 0.0, 0.0]
        else:
            k = angle / (2.0 * math.sin(angle))
            rot_err = [k * (Re[2][1] - Re[1][2]),
                       k * (Re[0][2] - Re[2][0]),
                       k * (Re[1][0] - Re[0][1])]
        return pos_err + rot_err

    def _jacobian(self, q, use_rot, eps=1e-6):
        """Numerical Jacobian (rows = task dims, cols = joints) by finite diff."""
        base = self._pose_error_from_q(q, use_rot, zero_target=True)
        m = len(base)
        n = len(q)
        J = [[0.0] * n for _ in range(m)]
        for j in range(n):
            qp = q[:]
            qp[j] += eps
            pert = self._pose_error_from_q(qp, use_rot, zero_target=True)
            for i in range(m):
                # _pose_error_from_q returns the raw FK task vector, so this
                # finite difference is d(fk)/dq directly.
                J[i][j] = (pert[i] - base[i]) / eps
        return J

    def _pose_error_from_q(self, q, use_rot, zero_target):
        """Task-space vector at q (position, and log-map orientation if use_rot).

        With zero_target=True this returns the raw FK task vector (target = 0),
        used only for finite-difference Jacobian columns.
        """
        T = self.fk_matrix(q)
        vec = [T[0][3], T[1][3], T[2][3]]
        if use_rot:
            R = [[T[i][j] for j in range(3)] for i in range(3)]
            angle = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0)))
            if abs(angle) < 1e-9:
                vec += [0.0, 0.0, 0.0]
            else:
                k = angle / (2.0 * math.sin(angle))
                vec += [k * (R[2][1] - R[1][2]),
                        k * (R[0][2] - R[2][0]),
                        k * (R[1][0] - R[0][1])]
        return vec

    # ---------------- servo <-> joint helpers ----------------
    def q_to_servo_deg(self, q):
        """Convert joint angles (rad) to per-joint servo commands (deg, 0..300)."""
        return [j.servo_deg(qi) for j, qi in zip(self.joints, q)]

    def servo_deg_to_q(self, servo_degs):
        return [j.q_from_servo(d) for j, d in zip(self.joints, servo_degs)]


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


if __name__ == "__main__":
    # Quick self-check with placeholder geometry (no hardware needed).
    arm = Arm()

    q0 = [0.0, 0.0, 0.0, 0.0, 0.0]
    Logger.log("KINEMATICS", f"FK at home pose: {tuple(round(v, 2) for v in arm.fk(q0))}")

    # Round-trip: pick a reachable target from a known pose, then solve IK back.
    q_true = [0.3, 0.2, -0.4, 0.5, 0.1]
    target = arm.fk(q_true)
    Logger.log("KINEMATICS", f"Target position: {tuple(round(v, 2) for v in target)}")

    q_sol, ok = arm.ik(target, seed=[0.0] * 5)
    reached = arm.fk(q_sol)
    Logger.log("KINEMATICS", f"IK converged: {ok}")
    Logger.log("KINEMATICS", f"Reached position: {tuple(round(v, 2) for v in reached)}")
    Logger.log("KINEMATICS", f"Servo commands (deg): {[round(d, 1) for d in arm.q_to_servo_deg(q_sol)]}")
