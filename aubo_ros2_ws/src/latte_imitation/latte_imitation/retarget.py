"""笛卡尔空间重定向：将 RM65 拉花轨迹映射到 Aubo E5 工作空间。"""

import numpy as np
import PyKDL


def rpy_to_rotation(rpy):
    """URDF 约定：固定轴 X-Y-Z (roll-pitch-yaw) -> 3x3 旋转矩阵。"""
    return PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2]))


def rotation_to_rpy(R):
    """PyKDL.Rotation -> [r, p, y]."""
    r, p, y = R.GetRPY()
    return [float(r), float(p), float(y)]


class CartesianRetargeter:
    """RM65 -> Aubo E5 笛卡尔空间映射。

    策略：
      1. 以 RM65 轨迹质心为参考点
      2. 相对位置做各轴缩放 (position_scale)
      3. 平移到 Aubo 工作空间中心 (aubo_center)
      4. 末端姿态保持不变
    """

    def __init__(self, position_scale=None, aubo_center=None,
                 rotation_offset_rpy=None, auto_center=True):
        """
        Args:
            position_scale: [sx, sy, sz] 各轴缩放因子
            aubo_center: [x, y, z] Aubo 工作空间中心
            rotation_offset_rpy: [r, p, y] 姿态偏置
            auto_center: 是否从轨迹数据自动计算 RM65 质心
        """
        self.position_scale = (np.array(position_scale, dtype=float)
                               if position_scale is not None
                               else np.array([0.85, 0.85, 0.85]))
        self.aubo_center = (np.array(aubo_center, dtype=float)
                            if aubo_center is not None
                            else np.array([0.3, 0.0, 0.6]))
        self.auto_center = auto_center
        self.rm65_center = None  # 由轨迹数据计算

        if rotation_offset_rpy is not None:
            self.rotation_offset = rpy_to_rotation(rotation_offset_rpy)
        else:
            self.rotation_offset = PyKDL.Rotation.Identity()

    def fit(self, rm65_frames):
        """从 RM65 轨迹计算参考中心。

        Args:
            rm65_frames: list of PyKDL.Frame，用于计算位置质心
        """
        positions = np.array([[f.p[0], f.p[1], f.p[2]] for f in rm65_frames])
        self.rm65_center = positions.mean(axis=0)

    def retarget_frame(self, rm65_frame):
        """重定向单个 PyKDL.Frame。

        Args:
            rm65_frame: PyKDL.Frame 在 RM65 base 坐标系下

        Returns:
            PyKDL.Frame 在 Aubo base 坐标系下
        """
        if self.rm65_center is None:
            raise RuntimeError("必须先调用 fit() 计算 RM65 质心，或设置 auto_center=False")

        p_rm65 = np.array([rm65_frame.p[0], rm65_frame.p[1], rm65_frame.p[2]])
        p_centered = p_rm65 - self.rm65_center
        p_scaled = p_centered * self.position_scale
        p_aubo = p_scaled + self.aubo_center

        R_new = self.rotation_offset * rm65_frame.M

        return PyKDL.Frame(R_new, PyKDL.Vector(*p_aubo))

    def retarget_trajectory(self, rm65_frames):
        """重定向整条轨迹。

        Args:
            rm65_frames: list of PyKDL.Frame

        Returns:
            list of PyKDL.Frame
        """
        if self.auto_center:
            self.fit(rm65_frames)
        return [self.retarget_frame(f) for f in rm65_frames]

    @property
    def summary(self):
        """返回当前重定向参数的摘要。"""
        return {
            "rm65_center": (self.rm65_center.tolist()
                            if self.rm65_center is not None else None),
            "aubo_center": self.aubo_center.tolist(),
            "position_scale": self.position_scale.tolist(),
            "rotation_offset_rpy": rotation_to_rpy(self.rotation_offset),
            "auto_center": self.auto_center,
        }
