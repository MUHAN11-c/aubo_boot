"""
DMP (Dynamic Movement Primitive) — 从示教轨迹学习可泛化形状喵~

=== 理论基础 ===

  DMP 将轨迹分解为:
    - 变换系统 (transformation system): 二阶线性弹簧-阻尼 + 非线性 forcing term
    - 规范系统 (canonical system): 一阶指数衰减, 驱动 forcing term 的时间相位

  公式:
    τ·ÿ = α_y·(β_y·(g - y) - τ·ẏ) + f(x)
    τ·ẋ = -α_x·x

  其中:
    f(x) = Σ w_i · ψ_i(x) · x / Σ ψ_i(x) · (g - y_0)
    ψ_i(x) = exp(-(x - c_i)² / (2·σ_i²))

  学习: 从示教轨迹 {y_demo, ẏ_demo, ÿ_demo} 反推 forcing term target:
    f_target = τ²·ÿ_demo - α_y·(β_y·(g - y_demo) - τ·ẏ_demo)

  然后通过 LWR (Locally Weighted Regression) 求解权重 w_i。

=== 参考文献 ===
  Ijspeert et al., "Dynamical Movement Primitives: Learning Attractor Models
  for Motor Behaviors", Neural Computation, 2013.
  Schaal et al., "Learning Movement Primitives", ISRR, 2003.

=== 适用性 ===
  5 条示教足够训练, one-shot 泛化到新起点/终点, 保留心形形状特征喵~
"""

import numpy as np
from scipy.interpolate import interp1d


class DMP:
    """一维 DMP — 多维度并联使用喵~

    Args:
        n_basis: 高斯基函数数量 (默认 25)
        dt: 时间步长 (秒)
        alpha_y: 变换系统弹性系数 (默认 25.0)
        beta_y: 变换系统阻尼系数 (默认 6.25, 临界阻尼 β=α/4)
        alpha_x: 规范系统衰减率 (默认 1.0)
    """

    def __init__(self, n_basis: int = 25, dt: float = 0.01,
                 alpha_y: float = 25.0, beta_y: float = 6.25,
                 alpha_x: float = 1.0):
        self.n_basis = n_basis
        self.dt = dt
        self.alpha_y = alpha_y
        self.beta_y = beta_y if beta_y > 0 else alpha_y / 4.0
        self.alpha_x = alpha_x

        # 基函数中心 (指数间距)
        self.c = np.exp(-self.alpha_x * np.linspace(0, 1, n_basis))
        # 基函数宽度
        self.h = np.ones(n_basis) * (self.n_basis ** 1.5) / self.c / self.alpha_x
        # 学习到的权重
        self.weights = None

    def _basis(self, x: np.ndarray) -> np.ndarray:
        """计算基函数激活值 ψ_i(x) 喵~

        Args:
            x: (T,) 规范系统状态

        Returns:
            (T, n_basis) 基函数矩阵
        """
        T = len(x)
        psi = np.zeros((T, self.n_basis))
        for i in range(self.n_basis):
            psi[:, i] = np.exp(-0.5 * self.h[i] * (x - self.c[i]) ** 2)
        return psi

    def _canonical_system(self, tau: float, T: int) -> np.ndarray:
        """模拟规范系统: τ·ẋ = -α_x·x 喵~

        初始 x(0)=1, 指数衰减到 0

        Args:
            tau: 时间缩放因子
            T: 帧数

        Returns:
            (T,) x 值, (T,) 时间数组
        """
        t = np.arange(T) * self.dt * tau
        x = np.exp(-self.alpha_x * t / tau)
        return t, x

    def learn(self, y_demo: np.ndarray, tau: float = 1.0):
        """从单条示教轨迹学习 forcing term 权重喵~

        Args:
            y_demo: (T,) 示教轨迹 (一维)
            tau: 时间缩放因子 (默认 1.0 = 原始速度)
        """
        T = len(y_demo)
        t, x = self._canonical_system(tau, T)

        # 计算示教轨迹的导数
        yd = np.gradient(y_demo, self.dt * tau)
        ydd = np.gradient(yd, self.dt * tau)

        # 目标值与起始值
        g = y_demo[-1]  # 终点 = 目标
        y0 = y_demo[0]  # 起点

        # Forcing term 目标值
        f_target = (tau ** 2) * ydd - self.alpha_y * (
            self.beta_y * (g - y_demo) - tau * yd
        )

        # 基函数
        psi = self._basis(x)  # (T, n_basis)

        # LWR (Locally Weighted Regression) 求解权重
        # w_i = (s^T·Ψ_i·f_target) / (s^T·Ψ_i·s)
        # 其中 s = x * (g - y0)
        s = x * (g - y0)  # (T,)

        self.weights = np.zeros(self.n_basis)
        for i in range(self.n_basis):
            num = np.sum(s * psi[:, i] * f_target)
            den = np.sum(s ** 2 * psi[:, i])
            if abs(den) > 1e-10:
                self.weights[i] = num / den
            else:
                self.weights[i] = 0.0

        self._tau_learned = tau
        self._y0_learned = y0
        self._g_learned = g

    def generate(self, start: float = None, goal: float = None,
                 tau: float = 1.0, T: int = None,
                 noise_std: float = 0.0) -> np.ndarray:
        """生成泛化轨迹喵~

        Args:
            start: 新起点 (None=使用学习时的起点)
            goal:  新终点 (None=使用学习时的终点)
            tau:   时间缩放因子 (>1 慢放, <1 加速)
            T:     输出帧数 (None=自动从 tau 和 dt 推算)
            noise_std: 探索噪声标准差 (贝叶斯优化用)

        Returns:
            (T,) 生成的轨迹
        """
        if self.weights is None:
            raise RuntimeError("请先调用 learn() 学习轨迹后再生成喵~")

        y0 = start if start is not None else self._y0_learned
        g = goal if goal is not None else self._g_learned

        if T is None:
            T = int(self._tau_learned / (self.dt * tau))
            T = max(T, 50)

        t, x = self._canonical_system(tau, T)

        # 计算 forcing term
        psi = self._basis(x)  # (T, n_basis)
        f = np.sum(self.weights * psi, axis=1)  # (T,)
        f *= x * (g - y0)

        # 积分变换系统 (Euler 积分)
        y = np.zeros(T)
        dy = np.zeros(T)
        y[0] = y0
        dy[0] = 0.0

        dt_tau = self.dt * tau
        for i in range(T - 1):
            # 二阶系统
            ddy = (self.alpha_y * (self.beta_y * (g - y[i]) - tau * dy[i]) + f[i]) / (tau ** 2)
            dy[i + 1] = dy[i] + ddy * dt_tau
            y[i + 1] = y[i] + dy[i] * dt_tau

            # 探索噪声
            if noise_std > 0:
                y[i + 1] += np.random.normal(0, noise_std)

        return y


class DMP3D:
    """3D DMP — 三个独立 DMP 并联, X/Y/Z 各自学习喵~

    Args:
        n_basis: 高斯基函数数量
        dt: 时间步长 (秒)
    """

    def __init__(self, n_basis: int = 25, dt: float = 0.01):
        self.dt = dt
        self.dmp_x = DMP(n_basis=n_basis, dt=dt)
        self.dmp_y = DMP(n_basis=n_basis, dt=dt)
        self.dmp_z = DMP(n_basis=n_basis, dt=dt)
        self._tau_learned = 1.0

    def learn(self, trajectory: np.ndarray, tau: float = 1.0):
        """从 3D 示教轨迹学习喵~

        Args:
            trajectory: (T, 3) XYZ 轨迹
            tau: 时间缩放因子
        """
        self._tau_learned = tau
        self.dmp_x.learn(trajectory[:, 0].copy(), tau)
        self.dmp_y.learn(trajectory[:, 1].copy(), tau)
        self.dmp_z.learn(trajectory[:, 2].copy(), tau)

    def generate(self, start: np.ndarray = None, goal: np.ndarray = None,
                 tau: float = 1.0, T: int = None,
                 noise_std: float = 0.0) -> np.ndarray:
        """生成泛化 3D 轨迹喵~

        Args:
            start: (3,) 新起点
            goal:  (3,) 新终点
            tau:   时间缩放因子
            T:     输出帧数
            noise_std: 探索噪声标准差

        Returns:
            (T, 3) XYZ 轨迹
        """
        sx = start[0] if start is not None else None
        sy = start[1] if start is not None else None
        sz = start[2] if start is not None else None
        gx = goal[0] if goal is not None else None
        gy = goal[1] if goal is not None else None
        gz = goal[2] if goal is not None else None

        x = self.dmp_x.generate(start=sx, goal=gx, tau=tau, T=T, noise_std=noise_std)
        y = self.dmp_y.generate(start=sy, goal=gy, tau=tau, T=T, noise_std=noise_std)
        z = self.dmp_z.generate(start=sz, goal=gz, tau=tau, T=T, noise_std=noise_std)

        return np.column_stack([x, y, z])

    def save(self, path: str):
        """保存模型权重为 npz 文件喵~"""
        np.savez_compressed(path,
                            wx=self.dmp_x.weights, wy=self.dmp_y.weights, wz=self.dmp_z.weights,
                            c=self.dmp_x.c, h=self.dmp_x.h,
                            n_basis=self.dmp_x.n_basis, dt=self.dt,
                            tau=self._tau_learned,
                            y0_x=self.dmp_x._y0_learned, y0_y=self.dmp_y._y0_learned, y0_z=self.dmp_z._y0_learned,
                            g_x=self.dmp_x._g_learned, g_y=self.dmp_y._g_learned, g_z=self.dmp_z._g_learned)

    @classmethod
    def load(cls, path: str) -> "DMP3D":
        """从 npz 文件加载模型权重喵~"""
        data = np.load(path)
        dmp = cls(n_basis=int(data["n_basis"]), dt=float(data["dt"]))
        dmp._tau_learned = float(data["tau"])

        for dim, d in [("x", dmp.dmp_x), ("y", dmp.dmp_y), ("z", dmp.dmp_z)]:
            d.weights = data[f"w{dim}"]
            d.c = data["c"]
            d.h = data["h"]
            d.n_basis = int(data["n_basis"])
            d._y0_learned = float(data[f"y0_{dim}"])
            d._g_learned = float(data[f"g_{dim}"])
            d._tau_learned = float(data["tau"])

        return dmp

    @property
    def tau(self) -> float:
        return self._tau_learned

    @property
    def y0(self) -> np.ndarray:
        return np.array([self.dmp_x._y0_learned,
                         self.dmp_y._y0_learned,
                         self.dmp_z._y0_learned])

    @property
    def goal(self) -> np.ndarray:
        return np.array([self.dmp_x._g_learned,
                         self.dmp_y._g_learned,
                         self.dmp_z._g_learned])
