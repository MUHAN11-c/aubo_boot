"""PyKDL 运动学封装：FK + 自定义 DLS 数值IK，支持 URDF 手动构建 Chain。"""

import warnings
import numpy as np
import PyKDL

# urdf_parser_py 对非标准属性 "start_stop" 报警告，不影响功能
warnings.filterwarnings("ignore", message=".*start_stop.*")

from urdf_parser_py import urdf as urdf_parser  # noqa: E402


def _rpy_to_kdl_rotation(rpy):
    """URDF rpy (list of 3 floats) -> PyKDL.Rotation (fixed-axis X-Y-Z)."""
    return PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2]))


def _xyz_to_kdl_vector(xyz):
    """URDF xyz (list of 3 floats) -> PyKDL.Vector."""
    return PyKDL.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def _read_urdf_bytes(urdf_path):
    """读取 URDF 文件，移除 XML encoding 声明以兼容 lxml。"""
    with open(urdf_path, "rb") as f:
        raw = f.read()
    if raw.startswith(b"<?xml"):
        end = raw.find(b"?>") + 2
        raw = raw[end:]
    return raw


def rot_to_rpy(R):
    """PyKDL.Rotation 或 3x3 numpy -> [r, p, y] (fixed-axis X-Y-Z)."""
    if isinstance(R, PyKDL.Rotation):
        r, p, y = R.GetRPY()
        return [float(r), float(p), float(y)]
    # numpy matrix fallback: use the RPY decomposition
    # PyKDL-style RPY: R = RotZ(y) * RotY(p) * RotX(r)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-12
    if not singular:
        r = np.arctan2(R[2, 1], R[2, 2])
        p = np.arctan2(-R[2, 0], sy)
        y = np.arctan2(R[1, 0], R[0, 0])
    else:
        r = np.arctan2(-R[1, 2], R[1, 1])
        p = np.arctan2(-R[2, 0], sy)
        y = 0.0
    return [float(r), float(p), float(y)]


def log_so3(R):
    """SO(3) log map: 旋转矩阵 -> 旋转向量 (axis * angle)。"""
    if isinstance(R, PyKDL.Rotation):
        R_np = np.array([[R[0, 0], R[0, 1], R[0, 2]],
                          [R[1, 0], R[1, 1], R[1, 2]],
                          [R[2, 0], R[2, 1], R[2, 2]]])
    else:
        R_np = np.array(R)
    tr = np.trace(R_np)
    cos_theta = (tr - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if abs(theta) < 1e-12:
        return np.zeros(3)
    if abs(theta - np.pi) < 1e-6:
        # edge case: pi rotation
        B = (R_np + np.eye(3)) / 2.0
        # find non-zero column
        for j in range(3):
            v = B[:, j]
            if np.linalg.norm(v) > 0.3:
                w = v / np.linalg.norm(v)
                return w * theta
        return np.zeros(3)
    w = np.array([
        R_np[2, 1] - R_np[1, 2],
        R_np[0, 2] - R_np[2, 0],
        R_np[1, 0] - R_np[0, 1],
    ]) / (2.0 * np.sin(theta))
    return w * theta


def _kdl_frame_to_arrays(frame):
    """PyKDL.Frame -> (pos_3, R_3x3)."""
    R = np.array([
        [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
        [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
        [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
    ])
    p = np.array([frame.p[0], frame.p[1], frame.p[2]])
    return p, R


def build_chain_from_urdf(urdf_path, base_link, tip_link):
    """从 URDF 文件构建 PyKDL.Chain（只包含 revolute 关节）。"""
    raw = _read_urdf_bytes(urdf_path)
    robot = urdf_parser.URDF.from_xml_string(raw)

    parent_map = {}
    joint_map = {}
    for j in robot.joints:
        if j.type != "revolute":
            continue
        parent_map[j.child] = (j.name, j.parent)
        joint_map[j.name] = j

    path_joint_names = []
    cur = tip_link
    while cur != base_link:
        if cur not in parent_map:
            raise ValueError(
                f"Cannot trace from '{tip_link}' to '{base_link}': "
                f"'{cur}' has no parent in URDF revolute joints"
            )
        jname, parent = parent_map[cur]
        path_joint_names.insert(0, jname)
        cur = parent

    chain = PyKDL.Chain()
    for jname in path_joint_names:
        joint = joint_map[jname]
        origin_trans = _xyz_to_kdl_vector(joint.origin.xyz)
        origin_rot = _rpy_to_kdl_rotation(joint.origin.rpy)
        origin_frame = PyKDL.Frame(origin_rot, origin_trans)
        axis = PyKDL.Vector(
            float(joint.axis[0]),
            float(joint.axis[1]),
            float(joint.axis[2]),
        )
        kdl_joint = PyKDL.Joint(
            PyKDL.Vector(0, 0, 0), axis, PyKDL.Joint.RotAxis
        )
        segment = PyKDL.Segment(kdl_joint, origin_frame)
        chain.addSegment(segment)

    return chain, path_joint_names


def cartesian_error(frame_current, frame_target, pos_only=False):
    """计算 6D 笛卡尔误差（在 base frame 中）。

    Args:
        frame_current: PyKDL.Frame 当前位姿
        frame_target: PyKDL.Frame 目标位姿
        pos_only: True 则只计算位置误差（3D）

    Returns:
        np.ndarray: (6,) 或 (3,) 误差向量
    """
    p_cur, R_cur = _kdl_frame_to_arrays(frame_current)
    p_tgt, R_tgt = _kdl_frame_to_arrays(frame_target)

    pos_err = p_tgt - p_cur

    if pos_only:
        return pos_err

    R_err = R_cur.T @ R_tgt
    ori_err = R_cur @ log_so3(R_err)

    return np.concatenate([pos_err, ori_err])


class RobotModel:
    """机器人运动学模型：FK + 自定义 DLS 数值 IK。"""

    def __init__(self, urdf_path, base_link, tip_link):
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.tip_link = tip_link
        self.chain, self.joint_names = build_chain_from_urdf(
            urdf_path, base_link, tip_link
        )
        self.num_joints = self.chain.getNrOfJoints()
        self._fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)

        # Joint limits from URDF
        self._q_min, self._q_max = self._extract_joint_limits(urdf_path)

    def _extract_joint_limits(self, urdf_path):
        raw = _read_urdf_bytes(urdf_path)
        robot = urdf_parser.URDF.from_xml_string(raw)
        qmin, qmax = [], []
        jnames = self.joint_names
        jmap = {j.name: j for j in robot.joints if j.type == "revolute"}
        for name in jnames:
            joint = jmap[name]
            lo = float(joint.limit.lower)
            hi = float(joint.limit.upper)
            qmin.append(lo)
            qmax.append(hi)
        return np.array(qmin), np.array(qmax)

    def fk(self, q):
        if len(q) != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} joint angles, got {len(q)}"
            )
        kdl_q = PyKDL.JntArray(self.num_joints)
        for i, val in enumerate(q):
            kdl_q[i] = float(val)
        end_frame = PyKDL.Frame()
        self._fk_solver.JntToCart(kdl_q, end_frame)
        return end_frame

    def fk_batch(self, q_array):
        T = np.zeros((len(q_array), 4, 4))
        for i, q in enumerate(q_array):
            frame = self.fk(q)
            T[i] = self._frame_to_matrix(frame)
        return T

    def _jacobian_fd(self, q, delta=1e-6, pos_only=False):
        """有限差分计算 6×n 或 3×n 几何雅可比（在 base frame 中）。"""
        frame_nom = self.fk(q)
        p_nom, R_nom = _kdl_frame_to_arrays(frame_nom)
        m = 3 if pos_only else 6
        J = np.zeros((m, self.num_joints))

        for i in range(self.num_joints):
            q_pert = np.array(q, dtype=float)
            q_pert[i] += delta
            frame_pert = self.fk(q_pert)
            p_pert, R_pert = _kdl_frame_to_arrays(frame_pert)

            # 位置变化
            J[:3, i] = (p_pert - p_nom) / delta

            if not pos_only:
                # 姿态变化
                R_diff = R_nom.T @ R_pert
                ori_diff = R_nom @ log_so3(R_diff)
                J[3:6, i] = ori_diff / delta

        return J

    def ik(self, target_frame, q_init=None, maxiter=500, eps=1e-8,
           pos_only=False, damping=0.3, min_damping=1e-6):
        """DLS 逆运动学（有限差分雅可比 + 自适应阻尼）。

        Args:
            target_frame: PyKDL.Frame 目标位姿
            q_init: 初始猜测 (None 则用零位)
            maxiter: 最大迭代次数
            eps: 收敛阈值
            pos_only: True=只匹配位置
            damping: 初始阻尼因子
            min_damping: 最小阻尼

        Returns:
            tuple: (q_solution, success, error_norm)
        """
        if q_init is None:
            q = np.zeros(self.num_joints)
        else:
            q = np.array(q_init, dtype=float)

        d = damping
        best_err = float('inf')
        best_q = q.copy()

        for it in range(maxiter):
            frame_cur = self.fk(q)
            err = cartesian_error(frame_cur, target_frame, pos_only=pos_only)
            err_norm = float(np.linalg.norm(err))

            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()

            if err_norm < eps:
                return q.copy(), True, err_norm

            J = self._jacobian_fd(q, pos_only=pos_only)
            m = J.shape[0]

            # DLS: dq = J^T (J J^T + d^2 I)^(-1) e
            JJT = J @ J.T
            reg = JJT + (d * d) * np.eye(m)
            try:
                delta_q = J.T @ np.linalg.solve(reg, err)
            except np.linalg.LinAlgError:
                d = d * 2.0 + 0.1
                continue

            q_new = q + delta_q
            q_new = np.clip(q_new, self._q_min, self._q_max)

            # 评估新位姿
            frame_new = self.fk(q_new)
            err_new = cartesian_error(frame_new, target_frame, pos_only=pos_only)
            err_new_norm = float(np.linalg.norm(err_new))

            if err_new_norm < err_norm:
                q = q_new
                d = max(min_damping, d * 0.7)  # 缩小阻尼
            else:
                d = d * 1.5 + 0.1  # 增大阻尼

        return best_q.copy(), False, best_err

    def ik_with_restarts(self, target_frame, n_restarts=10, q_init=None, **kwargs):
        """随机初始化多次 IK，返回最优解。"""
        best_q = None
        best_err = float('inf')
        best_ok = False

        for i in range(n_restarts):
            if i == 0 and q_init is not None:
                init = q_init
            else:
                init = np.random.uniform(self._q_min, self._q_max)

            q_sol, ok, err = self.ik(target_frame, q_init=init, **kwargs)

            if ok:
                return q_sol, True, err  # 已收敛，直接返回
            if err < best_err:
                best_err = err
                best_q = q_sol
                best_ok = ok

        return best_q, best_ok, best_err

    def ik_trajectory(self, target_frames, q_init_first=None, pos_only=False,
                      n_restarts_first=20, **kwargs):
        """对轨迹逐帧 IK，使用 temporal warm-start。

        Args:
            target_frames: list of PyKDL.Frame
            q_init_first: 第一帧初始猜测
            pos_only: True=只匹配位置
            n_restarts_first: 第一帧的随机重启次数

        Returns:
            tuple: (q_traj (N, n_joints), failures list)
        """
        # 第一帧：多次随机重启
        q_sol, ok, err = self.ik_with_restarts(
            target_frames[0],
            q_init=q_init_first,
            n_restarts=n_restarts_first,
            pos_only=pos_only,
        )
        if not ok:
            q_sol, ok2, err2 = self.ik(
                target_frames[0], q_init=np.zeros(self.num_joints),
                pos_only=pos_only, maxiter=1000, damping=1.0, **kwargs
            )

        q_traj = []
        failures = []
        q_prev = q_sol.copy()

        for i, frame in enumerate(target_frames):
            q_sol, ok, err = self.ik(
                frame, q_init=q_prev, pos_only=pos_only, **kwargs
            )
            if ok:
                q_traj.append(q_sol)
                q_prev = q_sol
            else:
                # 失败时重新随机尝试
                q_sol2, ok2, err2 = self.ik_with_restarts(
                    frame, n_restarts=30, pos_only=pos_only, **kwargs
                )
                if ok2:
                    q_traj.append(q_sol2)
                    q_prev = q_sol2
                else:
                    q_traj.append(q_prev.copy())
                    failures.append(i)

        return np.array(q_traj), failures

    @staticmethod
    def _frame_to_matrix(frame):
        T = np.eye(4)
        for i in range(3):
            T[i, 3] = frame.p[i]
            for j in range(3):
                T[i, j] = frame.M[i, j]
        return T

    @staticmethod
    def matrix_to_frame(T):
        rot = PyKDL.Rotation(
            T[0, 0], T[0, 1], T[0, 2],
            T[1, 0], T[1, 1], T[1, 2],
            T[2, 0], T[2, 1], T[2, 2],
        )
        trans = PyKDL.Vector(T[0, 3], T[1, 3], T[2, 3])
        return PyKDL.Frame(rot, trans)
