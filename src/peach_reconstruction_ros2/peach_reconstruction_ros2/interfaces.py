"""
抽象接口层 — 纯核契约（abc.ABC，对标 nav2_core 形态）+ 实现注册表.

职责:
  设计文档 docs/superpowers/specs/2026-08-10-peach-layered-architecture.md
  §2.2 与协议 2.14（可替换架构）：ABC 只约束 workhorse 纯数据方法
  （numpy/纯 dataclass 进 → numpy/dict 出，无副作用、不碰 ROS）；本模块
  不 import 实现（防循环）。实现发现走 peach_core.registry.Registry
  按名注册/创建（yolo_ros 先例的正式化），注册语句以显式清单形式写在
  各实现模块末尾（导入期单线程注册，运行期只读）。

装配规则（2.14）:
  编排层（reconstruction_node）只依赖本模块的 ABC + 注册表，构造期按
  yaml ``*.impl`` 参数名 ``REGISTRY.create(name, **kwargs)`` 实例化注入；
  替换实现 = 新写一个 ABC 子类 + 一行注册 + 改 yaml 一个键。

注册表与默认实现清单（显式注册清单）:
  - FRAME_STORES:   'default'         → frame_collector.FrameCollector
  - CLOUD_BUILDERS: 'open3d_cloud'    → cloud_builder.Open3dCloudBuilder
  - REFINERS:       'bounded_icp'     → icp_refiner.BoundedIcp
  - VOLUMES:        'local_tsdf'      → tsdf_volume.LocalTsdf
  - REFITTERS:      'cylinder_refit'  → geometry_refiner.CylinderRefitter
                    'sphere_refit'    → geometry_refiner.SphereRefitter
  - MASK_GATES:     'strict_mask_gate' → mask_gate.StrictMaskGate

数据成员契约（start/reset 之外的状态面）写在各 ABC docstring，
由实现继承 docstring 语义，不以 abstractmethod 强制（实例属性无法
满足 ABC 抽象成员，强转 property 会无谓放大 diff）。

协议条款:
  2.14 可替换架构；纯核零 ROS import（test_pure_core.py AST 强制）。

线程模型:
  注册发生在模块导入期（单线程），运行期注册表只读（见
  peach_core.registry 模块 docstring）。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np

from peach_core.registry import Registry


class FrameStore(ABC):
    """
    批级帧栈数据持有者契约（对齐 frame_collector.FrameCollector）.

    数据成员契约（实现以实例属性提供）：``frames``（CapturedFrame 列表）、
    ``state``（IDLE/COLLECTING/READY/FAILED）、``target_id``、
    ``target_center``、``rejected_views``、``tf_failures``、
    ``last_rel_translation_m``、``last_rel_rotation_deg``。
    """

    @abstractmethod
    def start(self, target_id: str = '', target_center=None) -> str:
        """清空旧帧进入 COLLECTING 并绑定目标；返回状态说明字符串."""

    @abstractmethod
    def add_frame(self, frame) -> bool:
        """压栈一帧；满 max_views 拒收（False）."""

    @abstractmethod
    def remove_last(self):
        """弹出最后一帧；空栈返回 None."""

    @abstractmethod
    def reset(self) -> None:
        """清空全部帧/绑定/计数，状态回 IDLE."""

    @abstractmethod
    def finalize(self) -> Tuple[bool, str, Optional[np.ndarray]]:
        """结束采集：视角数达标拼接累加云并转 READY（ok, message, cloud）."""


class CloudBuilder(ABC):
    """单帧深度 → base 系点云契约（默认实现 Open3dCloudBuilder）."""

    @abstractmethod
    def build(self, depth_mm: np.ndarray, rgb_bgr: Optional[np.ndarray],
              camera_K: dict, T_base_camera: np.ndarray,
              stride: int = 1, target_mask=None
              ) -> Tuple[np.ndarray, Optional[np.ndarray], float]:
        """
        一帧深度反投影并变到 base 系.

        Args:
            depth_mm: (H, W) uint16 深度 [mm].
            rgb_bgr: (H, W, 3) uint8 BGR；None 只建几何.
            camera_K: 内参 dict {"fx","fy","cx","cy"}.
            T_base_camera: (4, 4) base←camera 位姿.
            stride: 降采样步长（像素）.

        Returns
        -------
            (xyz_base, colors_bgr|None, valid_depth_ratio)：
            (N, 3) [m] 点云、(N, 3) uint8 BGR 逐点颜色、有效深度占比.

        """


class Refiner(ABC):
    """
    FK 初值约束下的帧到模型配准契约（默认实现 BoundedIcp）.

    返回值鸭子类型对齐 icp_refiner.IcpResult：mode/correction/fitness/
    rmse/translation_m/rotation_deg/reason 字段与 accepted 属性。
    """

    @abstractmethod
    def refine(self, source_fk_base: np.ndarray,
               target_base: np.ndarray):
        """
        配准当前 FK 点云到既有模型，只做有界小修正.

        Args:
            source_fk_base: (N, 3) 当前帧点云（base 系，FK 已变换）[m].
            target_base: (M, 3) 模型点云（base 系）[m]；空表示模型未形成.

        Returns
        -------
            IcpResult（鸭子类型）：accepted=True 才允许积分.

        """


class Volume(ABC):
    """
    多帧 RGB-D 融合体积契约（默认实现 LocalTsdf）.

    显式轻量生命周期：``reset()`` 清空体积复用实例（对标 nav2 生命周期
    四件套的轻量版；现有实现也可直接弃例新建，语义一致）。
    """

    @abstractmethod
    def integrate_frame(self, rgb_bgr: np.ndarray, depth_mm: np.ndarray,
                        camera_K: dict, T_base_camera: np.ndarray) -> None:
        """积分一帧（BGR 彩图 + uint16 毫米深度 + base←camera 位姿）."""

    @abstractmethod
    def extract_cloud(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """提取融合点云：(xyz (N, 3) [m], colors_bgr|None)."""

    @abstractmethod
    def reset(self) -> None:
        """清空体积与累计耗时，实例可复用."""


class Refitter(ABC):
    """
    TSDF 云几何二次拟合契约（默认实现 CylinderRefitter/SphereRefitter）.

    圆柱（袋桃）与球（裸桃）为两条独立实现线，编排层按 target_kind
    各持一个实例（geometry_refiner.select_refitter 负责选线）。
    """

    @abstractmethod
    def refit(self, cloud_xyz: np.ndarray, target_kind: str,
              config=None, axis_hint=None) -> dict:
        """
        RANSAC 精化 + bottom→neck 消歧 + ACCEPT/REOBSERVE 门控.

        Args:
            cloud_xyz: (N, 3) 点 [m]（base_frame）.
            target_kind: 'bag'/'fruit'（实现可按自身拟合线忽略）.
            config: geometry_refiner.RefitConfig；None 用默认.
            axis_hint: 可选 bottom→neck 方向先验；球体无内禀轴时使用.

        Returns
        -------
            dict（RefitResult 形态，键集见
            geometry_refiner.refine_geometry）：ok/reason/kind/status/
            n_points/center/axis/axis_point/bottom/neck/entry/radius/
            diameter/span_m/rmse/inlier_ratio/flags；失败 ok=False 不抛异常.

        """


class MaskGate(ABC):
    """
    目标掩膜质量门契约（默认实现 StrictMaskGate）.

    入参鸭子类型对齐 mask_gate.MaskContext（stamp_ns/depth_mm/masks/
    bound_center/neighbor_centers）；返回鸭子类型对齐 mask_gate.GateResult
    （mask/reason，reason 为空串表示通过）。
    """

    @abstractmethod
    def check(self, mask_ctx):
        """
        评估一帧的目标掩膜门禁（同戳/像素数/有效深度占比/漂移/邻目标间距）.

        Args:
            mask_ctx: mask_gate.MaskContext（鸭子类型即可）.

        Returns
        -------
            mask_gate.GateResult（鸭子类型）：reason='' 时 mask 为
            可用掩膜（或未启用门禁时 None），否则为拒绝原因.

        """


# ── 实现注册表（2.14：ABC + Registry + yaml *.impl 装配）──────────────
# 注册语句在各实现模块末尾（显式注册清单，见模块 docstring 顶部表格）；
# 编排层只按名 create，不 import 实现类做条件分支。
FRAME_STORES: Registry[FrameStore] = Registry('帧栈存储')
CLOUD_BUILDERS: Registry[CloudBuilder] = Registry('点云构建器')
REFINERS: Registry[Refiner] = Registry('配准器')
VOLUMES: Registry[Volume] = Registry('融合体积')
REFITTERS: Registry[Refitter] = Registry('几何精化器')
MASK_GATES: Registry[MaskGate] = Registry('掩膜门')
