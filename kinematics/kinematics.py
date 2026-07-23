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

from fundamental.const import KinematicsConst, ArmConst
from fundamental.logger import Logger

# ---------------------------------------------------------------------------
# Link geometry — PLACEHOLDERS. Replace with measured values (any consistent
# length unit; mm assumed). Each offset is the translation from the previous
# joint's frame to this joint's origin, expressed in the previous frame BEFORE
# this joint rotates. Actual values live in fundamental.const.KinematicsConst.
# ---------------------------------------------------------------------------
BASE_HEIGHT_MM = KinematicsConst.BASE_HEIGHT_MM   # base bottom -> yaw joint (along Z)
RISER_MM       = KinematicsConst.RISER_MM         # yaw joint  -> roll joint (along Z)
SHOULDER_MM    = KinematicsConst.SHOULDER_MM      # roll joint -> first pitch joint (id3), along Z
UPPER_ARM_MM   = KinematicsConst.UPPER_ARM_MM     # id3 -> id4, along local X
FOREARM_MM     = KinematicsConst.FOREARM_MM       # id4 -> id5, along local X
TOOL_MM        = KinematicsConst.TOOL_MM          # id5 -> phone mount (end-effector), along local X


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
YAW = KinematicsConst.YAW      # about Z
ROLL = KinematicsConst.ROLL    # about X
PITCH = KinematicsConst.PITCH  # about Y


class Joint:
    """One revolute joint (mirrors a URDF <joint type="revolute">).

    axis      : (x, y, z) unit rotation axis in the joint frame (YAW/ROLL/PITCH)
    offset    : (x, y, z) origin translation from the parent joint frame
    rpy       : (roll, pitch, yaw) fixed origin rotation (URDF origin rpy)
    home_deg  : servo angle in [0, 300] that corresponds to joint angle q = 0
    direction : +1 or -1; servo_deg = home_deg + direction * degrees(q)
    q_min/q_max : static joint limits in radians (relative to home) — for a
                  coupled joint (see below) this is the conservative bound
                  that holds regardless of the other joint's angle.
    name      : URDF joint name (for reference)

    coupled_with/coupled_table: some joints' *actual* self-collision-free
    range depends on a neighboring joint's current angle (e.g. this arm's
    joint2/joint3 overlap in a way a single static range can't capture
    without being needlessly conservative). When set, coupled_with is the
    other joint's `id` and coupled_table is a list of
    (q_other, lower, upper) points sorted by q_other, linearly interpolated
    by `bounds()`/`clamp()` when the other joint's current angle is known —
    see kinematics/find_joint_limits.py, which generates this table.
    """

    def __init__(self, id, axis, offset, rpy=(0.0, 0.0, 0.0),
                 home_deg=150.0, direction=1,
                 q_min=-math.pi / 2, q_max=math.pi / 2, name=None,
                 coupled_with=None, coupled_table=None):
        self.id = id
        self.axis = axis
        self.offset = offset
        self.rpy = rpy
        self.home_deg = home_deg
        self.direction = direction
        self.q_min = q_min
        self.q_max = q_max
        self.name = name
        self.coupled_with = coupled_with
        self.coupled_table = coupled_table

    # --- conversions between joint angle q (rad) and servo angle (deg, 0..300) ---
    def servo_deg(self, q):
        return self.home_deg + self.direction * math.degrees(q)

    def q_from_servo(self, servo_deg):
        return self.direction * math.radians(servo_deg - self.home_deg)

    def bounds(self, q_other=None):
        """Effective (q_min, q_max) — interpolated from coupled_table against
        q_other when this joint is coupled and q_other is given, else the
        static (q_min, q_max)."""
        if self.coupled_table is None or q_other is None:
            return self.q_min, self.q_max

        table = self.coupled_table
        if q_other <= table[0][0]:
            return table[0][1], table[0][2]
        if q_other >= table[-1][0]:
            return table[-1][1], table[-1][2]
        for (qa, lo_a, hi_a), (qb, lo_b, hi_b) in zip(table, table[1:]):
            if qa <= q_other <= qb:
                t = (q_other - qa) / (qb - qa) if qb != qa else 0.0
                return lo_a + t * (lo_b - lo_a), hi_a + t * (hi_b - hi_a)
        return self.q_min, self.q_max  # unreachable if table is sorted

    def clamp(self, q, q_other=None):
        lo, hi = self.bounds(q_other)
        return max(lo, min(hi, q))


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

    # Default per-joint IK weights, keyed by Joint.id. Higher = more "expensive"
    # for the damped-least-squares solver to move, so redundancy (this 5-DOF
    # arm has 2 null-space DOF against a 3-DOF position target) gets resolved
    # by favoring the other joints instead. joint2 (roll) is weighted up
    # because its self-collision-free range is narrow and depends on joint3's
    # angle (see Joint.coupled_table) — preferring joint1 (yaw) to cover the
    # same reach keeps joint2 away from its coupled limits more often.
    DEFAULT_JOINT_WEIGHTS = ArmConst.DEFAULT_JOINT_WEIGHTS

    def __init__(self, joints=None, tool_offset=TOOL_OFFSET, joint_weights=None):
        self.joints = joints if joints is not None else DEFAULT_JOINTS
        self.tool_offset = tool_offset
        self.id_to_index = {j.id: i for i, j in enumerate(self.joints)}
        if joint_weights is None:
            joint_weights = [self.DEFAULT_JOINT_WEIGHTS.get(j.id, 1.0) for j in self.joints]
        self.joint_weights = joint_weights

    # ---------------- Forward kinematics ----------------
    def _fk_frames(self, q):
        """Return every intermediate 4x4 transform: base identity, after each
        joint's full step, and finally after the tool offset."""
        frames = [_identity4()]
        T = frames[0]
        for joint, qi in zip(self.joints, q):
            T = _matmul(T, _translate(*joint.offset))
            if joint.rpy != (0.0, 0.0, 0.0):
                T = _matmul(T, _rpy(*joint.rpy))
            T = _matmul(T, _rot_axis(joint.axis, qi))
            frames.append(T)
        frames.append(_matmul(T, _translate(*self.tool_offset)))
        return frames

    def fk_matrix(self, q):
        """Return the 4x4 end-effector pose given joint angles q (radians)."""
        return self._fk_frames(q)[-1]

    def fk(self, q):
        """Return just the end-effector position (x, y, z)."""
        T = self.fk_matrix(q)
        return (T[0][3], T[1][3], T[2][3])

    def fk_all(self, q):
        """Return the (x, y, z) position at the base, after each joint, and
        at the tool tip — every vertex needed to draw the arm as a connected
        stick figure."""
        return [(T[0][3], T[1][3], T[2][3]) for T in self._fk_frames(q)]

    # ---------------- Inverse kinematics ----------------
    def ik(self, target_pos, target_rot=None, seed=None,
           max_iter=200, tol=1e-3, damping=0.05, joint_weights=None):
        """Solve joint angles that place the end-effector at target_pos.

        target_pos : (x, y, z) desired position.
        target_rot : optional 3x3 desired orientation (list of rows). If given,
                     orientation is included in the objective (best-effort, since
                     5 DOF cannot satisfy a full 6-DOF pose exactly).
        seed       : starting joint angles (radians); defaults to all-zero.
        joint_weights : per-joint cost used to resolve the redundancy this
                     5-DOF arm has against a 3-DOF position target (defaults
                     to self.joint_weights, see Arm.DEFAULT_JOINT_WEIGHTS) —
                     a joint weighted higher moves less, so the solver favors
                     moving the other joints to reach the same target.
        Returns (q, converged): the joint angles and whether tol was reached.

        Uses weighted damped least squares:
            dq = W⁻¹ Jᵀ (J W⁻¹ Jᵀ + λ² I)⁻¹ e
        which reduces to the standard dq = Jᵀ(JJᵀ + λ²I)⁻¹e when every
        weight is 1.
        """
        q = list(seed) if seed is not None else [0.0] * len(self.joints)
        n = len(self.joints)
        lam2 = damping * damping
        weights = joint_weights if joint_weights is not None else self.joint_weights
        w_inv = [1.0 / w for w in weights]

        for _ in range(max_iter):
            T = self.fk_matrix(q)
            e = self._pose_error(T, target_pos, target_rot)
            if _norm(e) < tol:
                return q, True

            J = self._jacobian(q, use_rot=target_rot is not None)
            m = len(J)
            # A = J W⁻¹ (scale each column j by w_inv[j]); J W⁻¹ Jᵀ = A Jᵀ
            A = [[J[i][j] * w_inv[j] for j in range(n)] for i in range(m)]
            AJt = _matmul(A, _transpose(J))
            for i in range(len(AJt)):
                AJt[i][i] += lam2
            inv = _inverse(AJt)
            z = [sum(inv[i][k] * e[k] for k in range(len(e))) for i in range(m)]
            # dq = W⁻¹ Jᵀ z = Aᵀ z
            dq = [sum(A[i][j] * z[i] for i in range(m)) for j in range(n)]

            for i in range(n):
                joint = self.joints[i]
                q_other = q[self.id_to_index[joint.coupled_with]] \
                    if joint.coupled_with is not None else None
                q[i] = joint.clamp(q[i] + dq[i], q_other=q_other)

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
