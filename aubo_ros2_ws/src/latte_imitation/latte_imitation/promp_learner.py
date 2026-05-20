"""
ProMP (Probabilistic Movement Primitive) — 时序基函数轨迹学习喵~

=== 与 DMP 的关键区别 ===

  DMP: 规范系统指数衰减 (x→0), forcing term 作用随时间消失
       → 不适合持续振荡 (心形摆动)
  ProMP: 基函数均匀分布在时间轴上, 无衰减
         → 可捕获任意形状 (包括持续振荡)

=== 公式 ===

  y(t) = Ψ(t)^T · w + ε

  其中:
    Ψ(t)  = [ψ_1(t), ψ_2(t), ..., ψ_K(t)]  — 基函数向量
    w      = [w_1, w_2, ..., w_K]^T         — 权重向量
    ε      ~ N(0, σ²)                        — 观测噪声

  基函数: 高斯径向基函数, 中心均匀分布在 [0,1]
    ψ_k(t) = exp(-(t - c_k)² / (2·h²))
    c_k = k/(K-1),  h = 1/(K-1)/2

  学习 (Ridge Regression):
    w = (Ψ^T·Ψ + λ·I)^{-1} · Ψ^T · y_demo

  生成 (新起点/终点泛化):
    y_new(t) = y_demo(0→new_start) 线性混合 + Ψ(t)^T·w (形状)
    或直接: y_new = Ψ_new · w

=== 参考文献 ===
  Paraschos et al., "Using Probabilistic Movement Primitives in Robotics",
  Autonomous Robots, 2018.
"""

import numpy as np
from scipy.interpolate import interp1d


class ProMP:
    """一维 ProMP — 多维度并联使用喵~

    Args:
        n_basis: 高斯径向基函数数量 (默认 20)
        sigma: 基函数宽度因子 (默认 0.03, 归一化时间)
    """

    def __init__(self, n_basis: int = 20, sigma: float = 0.03):
        self.n_basis = n_basis
        self.sigma = sigma
        self.weights = None
        self._y0 = None
        self._g = None
        self._T_learned = 0

    def _basis_matrix(self, T: int) -> np.ndarray:
        """生成基函数矩阵 Ψ (T, n_basis) 喵~

        基函数中心均匀分布在 [0, 1], 边界外的基函数也参与,
        保证轨迹起点和终点也能被良好建模喵~
        """
        t = np.linspace(0, 1, T)
        # 基函数中心: 在 [0, 1] 均匀分布 + 两侧各扩展半个间距
        c_ext = np.linspace(-0.5/self.n_basis, 1 + 0.5/self.n_basis, self.n_basis)
        # 宽度
        h = self.sigma

        psi = np.zeros((T, self.n_basis))
        for k in range(self.n_basis):
            psi[:, k] = np.exp(-0.5 * ((t - c_ext[k]) / h) ** 2)

        return psi

    def learn(self, y_demo: np.ndarray, reg_lambda: float = 1e-6):
        """从示教轨迹学习权重 (Ridge Regression) 喵~

        Args:
            y_demo: (T,) 示教轨迹
            reg_lambda: L2 正则化系数
        """
        T = len(y_demo)
        self._T_learned = T
        self._y0 = y_demo[0]
        self._g = y_demo[-1]

        psi = self._basis_matrix(T)  # (T, K)

        # Ridge Regression: w = (Ψ^T·Ψ + λ·I)^{-1}·Ψ^T·y
        K = self.n_basis
        A = psi.T @ psi + reg_lambda * np.eye(K)
        b = psi.T @ y_demo
        self.weights = np.linalg.solve(A, b)

    def generate(self, T: int = None, start: float = None, goal: float = None) -> np.ndarray:
        """生成轨迹 (可泛化到新起点/终点) 喵~

        泛化策略: 保持形状 (权重不变), 线性混合到新边界。
        y_new(t) = shape(t) + blend(t)·(new_start - y0) + blend(1-t)·(new_goal - g)

        Args:
            T: 输出帧数 (None=学习时帧数)
            start: 新起点 (None=不变)
            goal: 新终点 (None=不变)

        Returns:
            (T,) 生成的轨迹
        """
        if self.weights is None:
            raise RuntimeError("请先调用 learn() 喵~")

        if T is None:
            T = self._T_learned

        psi = self._basis_matrix(T)
        y_shape = psi @ self.weights  # (T,) 形状成分

        # 边界混合: 将起点/终点拉向目标值
        y = y_shape.copy()

        if start is not None:
            delta_start = start - y_shape[0]
            blend = np.linspace(1, 0, T)  # 从 1 衰减到 0
            y += delta_start * blend

        if goal is not None:
            delta_goal = goal - y_shape[-1]
            blend = np.linspace(0, 1, T)  # 从 0 增长到 1
            y += delta_goal * blend

        return y

    def learn_multiple(self, demos: list, reg_lambda: float = 1e-6):
        """从多条示教学习 (时间对齐后求均值权重) 喵~

        Args:
            demos: list of (T_i,) 示教轨迹
            reg_lambda: L2 正则化系数
        """
        T_target = int(np.median([len(d) for d in demos]))
        all_weights = []

        for demo in demos:
            T = len(demo)
            # 重采样到统一长度
            if T != T_target:
                t_orig = np.linspace(0, 1, T)
                t_new = np.linspace(0, 1, T_target)
                demo = interp1d(t_orig, demo, kind='linear')(t_new)

            psi = self._basis_matrix(T_target)
            K = self.n_basis
            A = psi.T @ psi + reg_lambda * np.eye(K)
            b = psi.T @ demo
            w = np.linalg.solve(A, b)
            all_weights.append(w)

        self._T_learned = T_target
        self.weights = np.mean(all_weights, axis=0)
        # y0/g 取均值
        self._y0 = np.mean([d[0] for d in demos])
        self._g = np.mean([d[-1] for d in demos])


class ProMP3D:
    """3D ProMP — 三个独立 ProMP 并联喵~

    Args:
        n_basis: 每维基函数数量
        sigma: 基函数宽度因子
    """

    def __init__(self, n_basis: int = 20, sigma: float = 0.03):
        self.promp_x = ProMP(n_basis=n_basis, sigma=sigma)
        self.promp_y = ProMP(n_basis=n_basis, sigma=sigma)
        self.promp_z = ProMP(n_basis=n_basis, sigma=sigma)

    def learn(self, trajectory: np.ndarray, reg_lambda: float = 1e-6):
        """从单条 3D 轨迹学习喵~"""
        self.promp_x.learn(trajectory[:, 0].copy(), reg_lambda)
        self.promp_y.learn(trajectory[:, 1].copy(), reg_lambda)
        self.promp_z.learn(trajectory[:, 2].copy(), reg_lambda)

    def learn_multiple(self, trajectories: list, reg_lambda: float = 1e-6):
        """从多条 3D 轨迹学习喵~"""
        self.promp_x.learn_multiple([t[:, 0] for t in trajectories], reg_lambda)
        self.promp_y.learn_multiple([t[:, 1] for t in trajectories], reg_lambda)
        self.promp_z.learn_multiple([t[:, 2] for t in trajectories], reg_lambda)

    def generate(self, T: int = None, start: np.ndarray = None,
                 goal: np.ndarray = None) -> np.ndarray:
        """生成 3D 轨迹喵~"""
        sx = start[0] if start is not None else None
        sy = start[1] if start is not None else None
        sz = start[2] if start is not None else None
        gx = goal[0] if goal is not None else None
        gy = goal[1] if goal is not None else None
        gz = goal[2] if goal is not None else None

        x = self.promp_x.generate(T=T, start=sx, goal=gx)
        y = self.promp_y.generate(T=T, start=sy, goal=gy)
        z = self.promp_z.generate(T=T, start=sz, goal=gz)

        return np.column_stack([x, y, z])

    @property
    def y0(self) -> np.ndarray:
        return np.array([self.promp_x._y0, self.promp_y._y0, self.promp_z._y0])

    @property
    def goal(self) -> np.ndarray:
        return np.array([self.promp_x._g, self.promp_y._g, self.promp_z._g])

    def save(self, path: str):
        """保存模型权重喵~"""
        np.savez_compressed(path,
                            wx=self.promp_x.weights, wy=self.promp_y.weights, wz=self.promp_z.weights,
                            n_basis=self.promp_x.n_basis, sigma=self.promp_x.sigma,
                            T=self.promp_x._T_learned,
                            y0_x=self.promp_x._y0, y0_y=self.promp_y._y0, y0_z=self.promp_z._y0,
                            g_x=self.promp_x._g, g_y=self.promp_y._g, g_z=self.promp_z._g,
                            model_type="ProMP3D")

    @classmethod
    def load(cls, path: str) -> "ProMP3D":
        """加载模型权重喵~"""
        data = np.load(path, allow_pickle=True)
        model_type = str(data.get("model_type", "DMP3D"))
        if model_type != "ProMP3D":
            raise ValueError(f"模型类型不匹配: 期望 ProMP3D, 实际 {model_type}")

        promp = cls(n_basis=int(data["n_basis"]), sigma=float(data["sigma"]))
        promp.promp_x.weights = data["wx"]
        promp.promp_y.weights = data["wy"]
        promp.promp_z.weights = data["wz"]
        promp.promp_x._T_learned = int(data["T"])
        promp.promp_y._T_learned = int(data["T"])
        promp.promp_z._T_learned = int(data["T"])
        promp.promp_x._y0 = float(data["y0_x"])
        promp.promp_y._y0 = float(data["y0_y"])
        promp.promp_z._y0 = float(data["y0_z"])
        promp.promp_x._g = float(data["g_x"])
        promp.promp_y._g = float(data["g_y"])
        promp.promp_z._g = float(data["g_z"])
        return promp
