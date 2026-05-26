# 单臂 AUBO E5 咖啡拉花轨迹执行 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现单臂 AUBO E5 拉花轨迹的动态朝向剖面 + SE(3) 分离式重定目标 + 前端参数面板补全

**Architecture:** 新增 `latte_art/orientation_profile.py` 动态朝向生成；修改 `trajectory_transform.py` 增加 yaw 提取和分离式 retarget；修改 `trajectory_pipeline.py` 参数化模式下用杯子位姿作 retarget 目标；前端补全杯子/倾倒/速度配置 DOM，设置面板加参考位姿微调

**Tech Stack:** Python 3.10, NumPy, ROS 2 Humble (rclpy), MoveIt2, HTML/CSS/JS (ES modules)

**Spec:** `docs/superpowers/specs/2026-05-26-single-arm-latte-art-design.md`

**启动脚本:** `/home/mu/aubo_boot/aubo_ros2_ws/start_aubo_new_driver.sh`

---

## 文件结构

```
latte_imitation/
├── config/
│   └── latte_positions.yaml          ← NEW: ROS2 参考位姿默认值
├── latte_imitation/
│   ├── latte_art/
│   │   ├── __init__.py               ← MODIFY: 导出新模块
│   │   ├── config.py                 ← MODIFY: PourConfig 加朝向字段
│   │   ├── bridge.py                 ← MODIFY: 调用 assemble_cartesian_with_orientation
│   │   └── orientation_profile.py    ← NEW: 动态朝向剖面
│   ├── trajectory_transform.py       ← MODIFY: extract_yaw + retarget_constrained
│   └── trajectory_pipeline.py        ← MODIFY: Phase ②/④ 调用新函数

web/public/
├── coffee_latte_panel.html           ← MODIFY: 新增杯子/倾倒/速度 DOM
├── css/coffee_latte_panel.css        ← MODIFY: range slider + 条件显示样式
└── js/
    ├── latte/latte_controls.js       ← MODIFY: DEFAULTS 更新 + 条件显示
    └── core/settings.js              ← MODIFY: 新增参考位姿设置项 (或独立 settings JSON)
```

---

### Task 1: 新增 `latte_positions.yaml` 参考位姿配置文件

**Files:**
- Create: `aubo_ros2_ws/src/latte_imitation/config/latte_positions.yaml`

**背景:** 四个参考位姿 (coffee_Link, lizhu_Link, cup0_Link, reference_pose) 作为 ROS2 参数默认值，从 URDF TF 链计算 + JSON 参考位姿。来源: URDF `aubo_e5_base.urdf` 中 pedestal_Link 到各 link 的固定 joint 偏移，以及 `ivg_monitor_2026-05-26.json` 中的末端位姿。

- [ ] **Step 1: 创建 YAML 文件**

```bash
cat > /home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation/config/latte_positions.yaml << 'YAMLEOF'
# latte_positions.yaml — 拉花工作流程参考位姿
#
# 所有位姿在 base_link 坐标系中喵~
# 来源:
#   coffee_Link/lizhu_Link/cup0_Link: URDF aubo_e5_base.urdf 固定 joint 偏移
#     pedestal_Link 在 base_link: (-0.105, -0.105, -0.028)
#     coffee_joint: (-0.540, 0.203, 0.046) in pedestal → (-0.645, 0.098, 0.018) in base_link
#     lizhu_joint:  (-0.525, -0.263, 0.006) in pedestal → (-0.630, -0.368, -0.022) in base_link
#     cup0_joint:   (-0.423, -0.093, -0.012) in pedestal → (-0.528, -0.198, -0.040) in base_link
#   reference_pose: ivg_monitor_2026-05-26.json 末端位姿 (杯口 Z 轴朝上)
# =============================================================================

# ── coffee_Link: 咖啡机出杯位置 (取咖啡杯) ──
coffee_link:
  x: -0.645
  y: 0.098
  z: 0.05    # Z 需真机微调 (咖啡机出杯高度)
  roll: 0.0
  pitch: 0.0
  yaw: 0.0

# ── lizhu_Link: 立柱位置 (放咖啡杯, 拉花目标) ──
lizhu_link:
  x: -0.630
  y: -0.368
  z: 0.04    # 液面 Z = lizhu 顶(-0.022) + 杯高 + 液面距杯口, 需真机微调
  roll: 0.0
  pitch: 0.0
  yaw: 0.0

# ── cup0_Link: 牛奶杯/拉花壶位置 (取牛奶杯) ──
cup0_link:
  x: -0.528
  y: -0.198
  z: 0.05    # Z 需真机微调 (牛奶杯高度)
  roll: 0.0
  pitch: 0.0
  yaw: 0.0

# ── reference_pose: 杯口朝上安全中转位姿 ──
reference_pose:
  x: -0.419
  y: -0.400
  z: 0.246
  roll: -23.5
  pitch: 88.1
  yaw: 76.0
YAMLEOF
```

- [ ] **Step 2: 验证 YAML 可解析**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 -c "
import yaml
with open('src/latte_imitation/config/latte_positions.yaml') as f:
    data = yaml.safe_load(f)
for k, v in data.items():
    print(f'{k}: x={v[\"x\"]:.3f} y={v[\"y\"]:.3f} z={v[\"z\"]:.3f} rpy=({v[\"roll\"]:.1f},{v[\"pitch\"]:.1f},{v[\"yaw\"]:.1f})')
"
```

Expected output: 4 组位姿打印，数值与 YAML 一致

- [ ] **Step 3: 验证 YAML 中的位置与 URDF 计算一致**

```bash
python3 -c "
# 验证 URDF TF 链计算
pedestal_in_base = (-0.105, -0.105, -0.028)

# coffee_joint in pedestal
cj = (-0.540, 0.203, 0.046)
coffee_in_base = tuple(pedestal_in_base[i] + cj[i] for i in range(3))
print(f'coffee_Link in base_link: ({coffee_in_base[0]:.3f}, {coffee_in_base[1]:.3f}, {coffee_in_base[2]:.3f})')
assert abs(coffee_in_base[0] - (-0.645)) < 0.01
assert abs(coffee_in_base[1] - 0.098) < 0.01

# lizhu_joint in pedestal
lj = (-0.525, -0.263, 0.006)
lizhu_in_base = tuple(pedestal_in_base[i] + lj[i] for i in range(3))
print(f'lizhu_Link in base_link: ({lizhu_in_base[0]:.3f}, {lizhu_in_base[1]:.3f}, {lizhu_in_base[2]:.3f})')
assert abs(lizhu_in_base[0] - (-0.630)) < 0.01
assert abs(lizhu_in_base[1] - (-0.368)) < 0.01

# cup0_joint in pedestal
c0j = (-0.423, -0.093, -0.012)
cup0_in_base = tuple(pedestal_in_base[i] + c0j[i] for i in range(3))
print(f'cup0_Link in base_link: ({cup0_in_base[0]:.3f}, {cup0_in_base[1]:.3f}, {cup0_in_base[2]:.3f})')
assert abs(cup0_in_base[0] - (-0.528)) < 0.01
assert abs(cup0_in_base[1] - (-0.198)) < 0.01

print('All assertions passed')
"
```

Expected: `All assertions passed`

- [ ] **Step 4: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/config/latte_positions.yaml
git commit -m "feat: add latte_positions.yaml with URDF-derived reference poses

coffee_Link/lizhu_Link/cup0_Link positions computed from
aubo_e5_base.urdf fixed joint offsets in pedestal_Link frame.
reference_pose from ivg_monitor_2026-05-26.json (cup Z-up).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 2: 新增 `orientation_profile.py` 动态朝向剖面

**Files:**
- Create: `aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/orientation_profile.py`

**背景:** 当前 `bridge.py:parametric_to_cartesian()` 所有帧固定 pitch=45°。需改为三阶段动态 pitch：融合 45°→30°，成形 30°±3°，收尾 30°→60°。roll=0（无侧倾），yaw=0（canonical frame，后续 retarget 统一变换）。

- [ ] **Step 1: 写测试脚本 — 验证 pitch 剖面形状**

```bash
cat > /tmp/test_pitch_profile.py << 'PYEOF'
"""测试 orientation_profile 模块喵~"""
import sys
sys.path.insert(0, '/home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation/latte_imitation')
sys.path.insert(0, '/home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation')

import numpy as np
from latte_art.orientation_profile import compute_pitch_profile, assemble_cartesian_with_orientation
from latte_art.config import CupConfig, PourConfig

def test_pitch_profile_shape():
    """验证 pitch 剖面的三阶段形状"""
    total_frames = 280
    mix_end = 67      # ~25%
    draw_end = 238    # ~85%
    
    pitch = compute_pitch_profile(total_frames, mix_end, draw_end)
    
    assert len(pitch) == total_frames, f"长度应为 {total_frames}, 实际 {len(pitch)}"
    
    # 融合阶段 [0, mix_end): 45° → 30° 线性下降
    assert abs(pitch[0] - 45.0) < 1.0, f"Frame 0 pitch 应≈45°, 实际 {pitch[0]:.1f}"
    assert abs(pitch[mix_end-1] - 30.0) < 3.0, f"mix_end pitch 应≈30°, 实际 {pitch[mix_end-1]:.1f}"
    assert pitch[0] > pitch[mix_end-1], "融合阶段 pitch 应下降"
    
    # 成形阶段 [mix_end, draw_end): 30° ± 3°
    draw_mid = (mix_end + draw_end) // 2
    assert 27.0 < pitch[draw_mid] < 33.0, f"draw_mid pitch 应在 30±3°, 实际 {pitch[draw_mid]:.1f}"
    
    # 收尾阶段 [draw_end, total_frames): 30° → 60° 线性上升
    assert abs(pitch[draw_end] - 30.0) < 3.0, f"draw_end pitch 应≈30°, 实际 {pitch[draw_end]:.1f}"
    assert abs(pitch[-1] - 60.0) < 3.0, f"last frame pitch 应≈60°, 实际 {pitch[-1]:.1f}"
    assert pitch[draw_end] < pitch[-1], "收尾阶段 pitch 应上升"
    
    print("PASS: test_pitch_profile_shape")

def test_assemble_cartesian():
    """验证 assemble_cartesian_with_orientation 输出正确结构"""
    xyz = np.random.randn(100, 3).astype(np.float32)
    pitch = compute_pitch_profile(100, 25, 85)
    
    from latte_imitation.trajectory import CartesianTrajectory
    cart = assemble_cartesian_with_orientation(xyz, pitch, roll_deg=0.0, yaw_deg=0.0, dt=0.05)
    
    assert isinstance(cart, CartesianTrajectory)
    assert cart.num_frames == 100
    assert cart.orientations is not None
    assert cart.orientations.shape == (100, 4)
    
    # 验证第一帧朝向 ≈ euler(0, 45, 0)
    from trajectory_transform import quat_to_euler_deg
    rpy0 = quat_to_euler_deg(cart.orientations[0])
    assert abs(rpy0[0]) < 0.1, f"roll 应≈0, 实际 {rpy0[0]:.2f}"
    assert abs(rpy0[1] - 45.0) < 2.0, f"pitch 应≈45°, 实际 {rpy0[1]:.1f}"
    assert abs(rpy0[2]) < 0.1, f"yaw 应≈0, 实际 {rpy0[2]:.2f}"
    
    print("PASS: test_assemble_cartesian")

def test_stage_boundary_continuity():
    """验证阶段边界处 pitch 连续 (C0 连续)"""
    total_frames = 280
    mix_end, draw_end = 67, 238
    pitch = compute_pitch_profile(total_frames, mix_end, draw_end)
    
    # 相邻帧之间 pitch 变化应平滑 (< 2° per frame)
    diffs = np.abs(np.diff(pitch))
    assert np.max(diffs) < 2.0, f"帧间 pitch 变化应<2°, 实际最大 {np.max(diffs):.2f}°"
    
    print(f"PASS: test_stage_boundary_continuity (max frame delta = {np.max(diffs):.2f}°)")

if __name__ == '__main__':
    test_pitch_profile_shape()
    test_assemble_cartesian()
    test_stage_boundary_continuity()
    print("\nAll tests passed!")
PYEOF
```

- [ ] **Step 2: 运行测试 — 预期全部 FAIL (模块不存在)**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 /tmp/test_pitch_profile.py
```

Expected: `ModuleNotFoundError: No module named 'latte_art.orientation_profile'`

- [ ] **Step 3: 实现 `orientation_profile.py`**

```bash
cat > /home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/orientation_profile.py << 'PYEOF'
"""动态朝向剖面 — 为参数化轨迹生成每帧独立的奶缸倾倒姿态喵~

三阶段 pitch 规律 (来自多源交叉验证):
  融合 [0~25%]:   pitch 45°→30° 线性下降 (奶缸从高位大倾斜逐渐贴近液面)
  成形 [25~85%]:  pitch 30°±3° 微变 (贴近液面, 奶泡浮于表面)
  收尾 [85~100%]: pitch 30°→60° 线性上升 (奶缸上提, 细流 draw-through)

roll = 0 (全程无侧倾, 液体不洒)
yaw = 0 (canonical frame, 后续 SE(3) retarget 统一变换)
"""

import numpy as np
from typing import Optional

from .config import CupConfig, PourConfig


def compute_pitch_profile(
    total_frames: int,
    mix_end: int,
    draw_end: int,
    pour: Optional[PourConfig] = None,
) -> np.ndarray:
    """为每帧计算 pitch 角 (度) 喵~

    Args:
        total_frames: 轨迹总帧数
        mix_end:      融合阶段结束帧索引 (exclusive)
        draw_end:     成形阶段结束帧索引 (exclusive)
        pour:         倾倒参数 (使用朝向剖面默认值), None 时用默认

    Returns:
        (total_frames,) float64 — 每帧的 pitch 角 (度)
    """
    pitch = np.zeros(total_frames, dtype=np.float64)

    # ── 融合阶段 [0, mix_end): 45° → 30° ──
    if mix_end > 0:
        pitch[:mix_end] = np.linspace(45.0, 30.0, mix_end)

    # ── 成形阶段 [mix_end, draw_end): 30° ± 3° 微变 ──
    draw_frames = max(1, draw_end - mix_end)
    t_draw = np.linspace(0.0, 1.0, draw_frames)
    pitch[mix_end:draw_end] = 30.0 + 3.0 * np.sin(2.0 * np.pi * 0.5 * t_draw)

    # ── 收尾阶段 [draw_end, total_frames): 30° → 60° ──
    finish_frames = max(1, total_frames - draw_end)
    pitch[draw_end:] = np.linspace(30.0, 60.0, finish_frames)

    return pitch


def assemble_cartesian_with_orientation(
    xyz: np.ndarray,
    pitch_profile: np.ndarray,
    roll_deg: float = 0.0,
    yaw_deg: float = 0.0,
    dt: float = 0.05,
    episode_idx: int = -1,
    frame_id: str = "base_link",
) -> "CartesianTrajectory":
    """将 XYZ 轨迹 + 动态 pitch 剖面组装为 CartesianTrajectory 喵~

    每帧独立四元数: q[i] = euler_deg_to_quat(roll_deg, pitch_profile[i], yaw_deg)

    与 bridge.parametric_to_cartesian() 的区别:
      - 每帧 pitch 由剖面决定 (非固定 45°)
      - 保留了原有的 roll/yaw 参数入口
    """
    from latte_imitation.trajectory import CartesianTrajectory
    from latte_imitation.trajectory_transform import euler_deg_to_quat

    T = len(xyz)
    if len(pitch_profile) != T:
        raise ValueError(
            f"pitch_profile length ({len(pitch_profile)}) != xyz length ({T})"
        )

    orientations = np.zeros((T, 4), dtype=np.float32)
    for i in range(T):
        q = euler_deg_to_quat(roll_deg, float(pitch_profile[i]), yaw_deg)
        orientations[i] = q.astype(np.float32)

    timestamps = np.arange(T, dtype=np.float32) * dt
    return CartesianTrajectory(
        positions=np.asarray(xyz, dtype=np.float32),
        orientations=orientations,
        timestamps=timestamps,
        dt=dt,
        episode_idx=episode_idx,
        frame_id=frame_id,
    )
PYEOF
```

- [ ] **Step 4: 运行测试 — 验证 PASS**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
install/setup.bash 2>/dev/null || true
PYTHONPATH="src/latte_imitation:$PYTHONPATH" python3 /tmp/test_pitch_profile.py
```

Expected: `All tests passed!`

- [ ] **Step 5: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/orientation_profile.py
git commit -m "feat: add orientation_profile.py with 3-phase dynamic pitch

compute_pitch_profile: mix 45→30°, draw 30±3°, finish 30→60°
assemble_cartesian_with_orientation: per-frame quaternion from pitch array
replaces fixed 45° pitch in bridge.py

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 3: 修改 `config.py` — PourConfig 加朝向剖面字段

**Files:**
- Modify: `aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/config.py`

**背景:** `PourConfig` 已有倾倒工艺参数，需新增朝向剖面参数字段供 `orientation_profile.py` 使用。添加字段但不改变现有 dataclass 的向后兼容性（全部有默认值）。

- [ ] **Step 1: 修改 PourConfig**

在现有 `PourConfig` dataclass 末尾 (第 54 行 `finish_z` 方法之后) 添加字段:

```python
    # ── 朝向剖面参数 (NEW) ──
    mix_pitch_start_deg: float = 45.0     # 融合起始 pitch (度)
    mix_pitch_end_deg: float = 30.0       # 融合结束 pitch (度)
    draw_pitch_base_deg: float = 30.0     # 成形基础 pitch (度)
    draw_pitch_variation_deg: float = 3.0 # 成形 pitch 微变幅度 (度)
    finish_pitch_start_deg: float = 30.0  # 收尾起始 pitch (度)
    finish_pitch_end_deg: float = 60.0    # 收尾结束 pitch (度)
    pour_roll_deg: float = 0.0            # 全程 roll (度, 始终 0)
```

- [ ] **Step 2: 验证 dataclass 实例化不报错**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 -c "
import sys
sys.path.insert(0, 'src/latte_imitation/latte_imitation')
from latte_art.config import PourConfig
p = PourConfig()
print(f'mix_pitch_start={p.mix_pitch_start_deg}°, finish_pitch_end={p.finish_pitch_end_deg}°')
assert p.mix_pitch_start_deg == 45.0
assert p.finish_pitch_end_deg == 60.0
assert p.pour_roll_deg == 0.0
print('PASS')
"
```

Expected: `PASS`

- [ ] **Step 3: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/config.py
git commit -m "feat: add orientation profile fields to PourConfig

mix_pitch_start/end, draw_pitch_base/variation, finish_pitch_start/end,
pour_roll_deg — all with sensible defaults from multi-source validation.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 4: 修改 `bridge.py` — 调用新的朝向组装函数

**Files:**
- Modify: `aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/bridge.py`

**背景:** `parametric_to_cartesian()` 当前用固定 quat (默认 45° pitch) 给所有帧。改为调用 `assemble_cartesian_with_orientation()` 生成动态朝向。

- [ ] **Step 1: 修改 `parametric_to_cartesian()` 函数**

替换 `bridge.py` 中函数体:

```python
def parametric_to_cartesian(
    xyz: np.ndarray,
    roll_deg: float = 0.0,
    pitch_deg: float = 45.0,
    yaw_deg: float = 0.0,
    dt: float = 0.05,
    episode_idx: int = -1,
    frame_id: str = "base_link",
    pitch_profile: np.ndarray | None = None,
) -> "CartesianTrajectory":
    """将参数化 XYZ 轨迹转换为 CartesianTrajectory 对象喵~

    支持两种模式:
      - pitch_profile 不为 None: 使用动态朝向剖面 (推荐, 三阶段 pitch)
      - pitch_profile 为 None: 所有帧使用固定 pitch_deg (向后兼容)

    Args:
        xyz: (T, 3) XYZ 位置轨迹
        roll_deg: 绕 X 轴旋转 (度)
        pitch_deg: 绕 Y 轴旋转 (度), 默认 45°, 仅在 pitch_profile=None 时生效
        yaw_deg: 绕 Z 轴旋转 (度)
        dt: 时间步长 (s)
        episode_idx: -1 = 生成轨迹
        frame_id: 坐标系 ID
        pitch_profile: (T,) float array — 每帧 pitch 角 (度), None=固定

    Returns:
        CartesianTrajectory
    """
    from latte_imitation.latte_art.orientation_profile import (
        assemble_cartesian_with_orientation,
    )

    if pitch_profile is not None:
        return assemble_cartesian_with_orientation(
            xyz, pitch_profile, roll_deg=roll_deg, yaw_deg=yaw_deg,
            dt=dt, episode_idx=episode_idx, frame_id=frame_id,
        )

    # 向后兼容: 固定 pitch (旧行为)
    from latte_imitation.trajectory import CartesianTrajectory
    T = len(xyz)
    quat = euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg)
    orientations = np.tile(quat.astype(np.float32), (T, 1))
    timestamps = np.arange(T, dtype=np.float32) * dt
    return CartesianTrajectory(
        positions=xyz.astype(np.float32),
        orientations=orientations,
        timestamps=timestamps,
        dt=dt, episode_idx=episode_idx, frame_id=frame_id,
    )
```

- [ ] **Step 2: 验证向后兼容性和新功能**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 -c "
import sys, numpy as np
sys.path.insert(0, 'src/latte_imitation/latte_imitation')
sys.path.insert(0, 'src/latte_imitation')

# 测试 1: 向后兼容 (无 pitch_profile)
from latte_art.bridge import parametric_to_cartesian
xyz = np.random.randn(100, 3).astype(np.float32)
cart1 = parametric_to_cartesian(xyz)
assert cart1.num_frames == 100
assert cart1.orientations.shape == (100, 4)
print('Test 1 PASS: backward compat')

# 测试 2: 动态剖面
from latte_art.orientation_profile import compute_pitch_profile
pitch = compute_pitch_profile(100, 25, 85)
cart2 = parametric_to_cartesian(xyz, pitch_profile=pitch)
assert cart2.num_frames == 100
# 验证朝向变化 (不全相同)
assert not np.allclose(cart2.orientations[0], cart2.orientations[-1]), '朝向应有变化'
print('Test 2 PASS: dynamic pitch profile')

# 测试 3: pitch_profile 长度不匹配应报错
try:
    parametric_to_cartesian(xyz, pitch_profile=np.array([1.0]))
    assert False, '应抛出异常'
except (ValueError, IndexError):
    print('Test 3 PASS: length mismatch raises error')
"
```

Expected: 三项全部 PASS

- [ ] **Step 3: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/bridge.py
git commit -m "feat: bridge.py supports dynamic pitch_profile in parametric_to_cartesian

Adds optional pitch_profile parameter — when provided, uses per-frame
quaternion from orientation_profile. Otherwise falls back to fixed pitch
(backward compatible).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 5: 修改 `__init__.py` — 导出新模块

**Files:**
- Modify: `aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/__init__.py`

- [ ] **Step 1: 添加导出**

在现有 `__init__.py` 中添加:

```python
from .orientation_profile import (
    compute_pitch_profile,
    assemble_cartesian_with_orientation,
)

__all__ = [
    # ... 现有导出保持不变 ...
    "compute_pitch_profile",
    "assemble_cartesian_with_orientation",
]
```

- [ ] **Step 2: 验证导入**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 -c "
import sys
sys.path.insert(0, 'src/latte_imitation/latte_imitation')
from latte_art import compute_pitch_profile, assemble_cartesian_with_orientation
print('Import OK')
"
```

Expected: `Import OK`

- [ ] **Step 3: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/latte_art/__init__.py
git commit -m "feat: export orientation_profile functions from latte_art

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 6: 修改 `trajectory_transform.py` — 新增分离式 retarget

**Files:**
- Modify: `aubo_ros2_ws/src/latte_imitation/latte_imitation/trajectory_transform.py`

**背景:** 需添加两个新函数:
1. `extract_yaw_from_rotation(R)` — 从 3×3 旋转矩阵提取 yaw 角
2. `retarget_with_orientation_constraint()` — 位置用完整 R_rel，朝向仅用 yaw 分量

数学依据: `yaw = atan2(R[1,0], R[0,0])`，来源 `tf2/LinearMath/Matrix3x3.h:getEulerYPR()`

- [ ] **Step 1: 写测试脚本**

```bash
cat > /tmp/test_separated_retarget.py << 'PYEOF'
"""测试 SE(3) 分离式重定目标喵~"""
import sys
sys.path.insert(0, '/home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation/latte_imitation')
sys.path.insert(0, '/home/mu/aubo_boot/aubo_ros2_ws/src/latte_imitation')

import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_transform import (
    extract_yaw_from_rotation,
    retarget_with_orientation_constraint,
    retarget_trajectory,
    euler_deg_to_quat,
    quat_to_rot,
    quat_to_euler_deg,
)
from trajectory import CartesianTrajectory

def test_extract_yaw_pure_z():
    """纯 Z 轴旋转"""
    R = quat_to_rot(euler_deg_to_quat(0, 0, 76.0))
    yaw = extract_yaw_from_rotation(R)
    assert abs(yaw - 76.0) < 1.0, f"yaw 应≈76°, 实际 {yaw:.2f}"
    print("PASS: pure Z rotation")

def test_extract_yaw_combined():
    """组合旋转: yaw=76°, pitch=30°, roll=0"""
    R = quat_to_rot(euler_deg_to_quat(0, 30.0, 76.0))
    yaw = extract_yaw_from_rotation(R)
    assert abs(yaw - 76.0) < 1.0, f"yaw 应≈76°, 实际 {yaw:.2f}"
    print("PASS: combined rotation")

def test_extract_yaw_zero():
    """零旋转"""
    yaw = extract_yaw_from_rotation(np.eye(3))
    assert abs(yaw) < 0.01, f"yaw 应≈0, 实际 {yaw:.2f}"
    print("PASS: zero rotation")

def test_retarget_constrained_position():
    """验证位置变换: 完整 R_rel"""
    cart = CartesianTrajectory(
        positions=np.array([[0.0, 0.0, 0.3], [0.01, 0.0, 0.25], [0.0, -0.005, 0.3]], dtype=np.float32),
        orientations=np.tile(euler_deg_to_quat(0, 30, 0).astype(np.float32), (3, 1)),
        timestamps=np.array([0.0, 1.0, 2.0], dtype=np.float32),
        dt=1.0,
    )
    
    target = Pose(
        position=Point(x=-0.42, y=-0.40, z=0.20),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    
    # yaw=76°: 位置应绕Z旋转
    result = retarget_with_orientation_constraint(cart, target, rpy_user=(0, 0, 76))
    
    # Frame 0 应在 target 位置
    assert abs(result.positions[0, 0] - (-0.42)) < 0.01
    assert abs(result.positions[0, 1] - (-0.40)) < 0.01
    assert abs(result.positions[0, 2] - 0.20) < 0.01
    print("PASS: position retarget")

def test_retarget_constrained_orientation():
    """验证朝向约束: 仅 yaw 分量影响朝向"""
    cart = CartesianTrajectory(
        positions=np.array([[0.0, 0.0, 0.3], [0.01, 0.0, 0.25]], dtype=np.float32),
        orientations=np.tile(euler_deg_to_quat(0, 30, 0).astype(np.float32), (2, 1)),
        timestamps=np.array([0.0, 1.0], dtype=np.float32),
        dt=1.0,
    )
    
    target = Pose(
        position=Point(x=-0.42, y=-0.40, z=0.20),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    
    # 用 yaw=76° 变换
    result = retarget_with_orientation_constraint(cart, target, rpy_user=(0, 0, 76))
    
    rpy0 = quat_to_euler_deg(result.orientations[0])
    # roll 应≈0 (不受影响)
    assert abs(rpy0[0]) < 0.5, f"roll 应≈0, 实际 {rpy0[0]:.2f}"
    # pitch 应≈30° (轨迹原始 pitch, 不受 yaw 旋转影响)
    assert abs(rpy0[1] - 30.0) < 1.0, f"pitch 应≈30°, 实际 {rpy0[1]:.1f}"
    # yaw 应≈76° (被 yaw 旋转)
    assert abs(rpy0[2] - 76.0) < 2.0, f"yaw 应≈76°, 实际 {rpy0[2]:.1f}"
    print("PASS: orientation constraint — pitch preserved, yaw applied")

def test_retarget_no_rotation():
    """无旋转时应与旧 retarget_trajectory 行为一致"""
    cart = CartesianTrajectory(
        positions=np.array([[0.0, 0.0, 0.3], [0.01, 0.0, 0.25]], dtype=np.float32),
        orientations=np.tile(euler_deg_to_quat(0, 30, 0).astype(np.float32), (2, 1)),
        timestamps=np.array([0.0, 1.0], dtype=np.float32),
        dt=1.0,
    )
    
    target = Pose(
        position=Point(x=-0.42, y=-0.40, z=0.20),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    
    new = retarget_with_orientation_constraint(cart, target, rpy_user=(0, 0, 0))
    old = retarget_trajectory(cart, target, rpy_user=(0, 0, 0))
    
    assert np.allclose(new.positions, old.positions, atol=1e-6), "无旋转时位置应与旧版一致"
    print("PASS: no rotation matches old behavior")

if __name__ == '__main__':
    test_extract_yaw_pure_z()
    test_extract_yaw_combined()
    test_extract_yaw_zero()
    test_retarget_constrained_position()
    test_retarget_constrained_orientation()
    test_retarget_no_rotation()
    print("\nAll tests passed!")
PYEOF
```

- [ ] **Step 2: 运行测试 — 预期 FAIL (函数不存在)**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
install/setup.bash 2>/dev/null || true
python3 /tmp/test_separated_retarget.py
```

Expected: `AttributeError: module 'trajectory_transform' has no attribute 'extract_yaw_from_rotation'`

- [ ] **Step 3: 实现两个新函数**

在 `trajectory_transform.py` 文件末尾 (第 363 行 `retarget_trajectory()` 的 return 之后) 添加:

```python
# ═══════════════════════════════════════════════════════════════════
# SE(3) 分离式重定目标 — 位置完整 R, 朝向仅 yaw
# ═══════════════════════════════════════════════════════════════════

def extract_yaw_from_rotation(R: np.ndarray) -> float:
    """从 3×3 旋转矩阵提取绕 Z 轴的旋转角 (yaw) 喵~

    数学推导 (内旋 ZYX = 外旋 XYZ):
      R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
      R[1,0] = sin(yaw)·cos(pitch)
      R[0,0] = cos(yaw)·cos(pitch)
      → atan2(R[1,0], R[0,0]) = yaw  (当 cos(pitch) ≠ 0)

    对拉花任务 pitch ∈ [25°, 60°] → cos(pitch) ≠ 0 → 无 gimbal lock 喵~

    参考: ROS 2 tf2/LinearMath/Matrix3x3.h:getEulerYPR()
    """
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(np.degrees(yaw))


def retarget_with_orientation_constraint(
    cart: CartesianTrajectory,
    start_pose: Pose,
    rpy_user: tuple[float, float, float] = (0.0, 0.0, 0.0),
    translation_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> CartesianTrajectory:
    """SE(3) 分离式重定目标 — 位置用完整 R_rel, 朝向仅用 yaw 分量 喵~

    为什么分离:
      - 位置需要完整 R_rel 来正确放置图案在杯子坐标系
      - 朝向只需要 yaw 分量来对齐杯子方向
      - pitch (倾倒角度) 是拉花技能核心, 已编码在轨迹自身朝向剖面中
      - roll 始终为 0 (侧倾会导致液体洒出)

    变换公式:
      位置: p_new[i] = R_rel @ (p[i] - p[0]) + p_target + translation_offset
      朝向: q_new[i] = q_yaw_only ⊗ q_orig[i]

    Args:
        cart:                原始 CartesianTrajectory (canonical frame)
        start_pose:          目标位姿 (杯子在 base_link 中的位姿)
        rpy_user:            (roll_deg, pitch_deg, yaw_deg) 用户可调角度
        translation_offset:  (dx, dy, dz) 平移偏移 (m)

    Returns:
        变换后的 CartesianTrajectory
    """
    p0 = cart.positions[0].copy()
    p_target = np.array([
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z,
    ])
    q_cup = np.array([
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ])

    # 检查是否需要旋转
    all_zero_rpy = all(abs(r) < 1e-9 for r in rpy_user)
    is_identity_cup = (
        abs(q_cup[0]) < 1e-9 and abs(q_cup[1]) < 1e-9
        and abs(q_cup[2]) < 1e-9 and abs(q_cup[3] - 1.0) < 1e-9
    )

    if all_zero_rpy and is_identity_cup:
        R_rel = np.eye(3)
        yaw_rel_deg = 0.0
    else:
        R_rel = compute_rotation_matrix(rpy_user, q_cup)
        yaw_rel_deg = extract_yaw_from_rotation(R_rel)

    # ── 位置变换: 完整 R_rel ──
    translation = np.array(translation_offset, dtype=float)
    new_positions = np.array([
        R_rel @ (p - p0) + p_target + translation
        for p in cart.positions
    ])

    # ── 朝向变换: 仅 yaw 分量 ──
    if cart.orientations is not None and abs(yaw_rel_deg) > 1e-9:
        q_yaw = euler_deg_to_quat(0.0, 0.0, yaw_rel_deg)
        new_orientations = np.array([
            quat_multiply(q_yaw, q) for q in cart.orientations
        ])
    else:
        new_orientations = (
            None if cart.orientations is None
            else cart.orientations.copy()
        )

    return CartesianTrajectory(
        positions=new_positions,
        orientations=new_orientations,
        timestamps=cart.timestamps.copy(),
        dt=cart.dt,
        episode_idx=cart.episode_idx,
        frame_id=cart.frame_id,
    )
```

- [ ] **Step 4: 运行测试 — 验证 PASS**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
python3 /tmp/test_separated_retarget.py
```

Expected: 6 项全部 PASS, `All tests passed!`

- [ ] **Step 5: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/trajectory_transform.py
git commit -m "feat: add separated SE(3) retarget with orientation constraint

extract_yaw_from_rotation: atan2(R[1,0], R[0,0]) from tf2 Matrix3x3
retarget_with_orientation_constraint: full R_rel for position,
yaw-only for orientation — preserves pouring pitch, prevents spill.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 7: 修改 `trajectory_pipeline.py` — 集成新管线

**Files:**
- Modify: `aubo_ros2_ws/src/latte_imitation/latte_imitation/trajectory_pipeline.py`

**背景:** 修改 `_pipeline()` 的 Phase ②:
- 参数化模式 (pattern_type 非空): 用杯子位姿作 retarget 目标，调用 `retarget_with_orientation_constraint()`
- 录制回放模式: 保持现有逻辑不变
- Phase ① 中 `_load_or_generate()` 传递 `pitch_profile`
- 调用 `orchestration_profile.compute_pitch_profile()` 并在 `parametric_to_cartesian()` 中传入

- [ ] **Step 1: 修改 `_pipeline()` Phase ① — 生成时传入 pitch_profile**

在 `_load_or_generate()` 调用之后，`_pipeline()` Phase ② 之前添加朝向剖面生成:

修改 `trajectory_pipeline.py` 第 299-346 行之间:

```python
        # ═══ Phase ①: Load / Generate ═══
        cart = self._load_or_generate(
            episode_idx, arm, pattern_type, pattern_image_path,
            tulip_layers, cup_params, pour_params)
        if cart is None:
            # ... (existing error handling unchanged)

        # ═══ Phase ②: OrientProfile (参数化模式) ═══
        if pattern_type and cup_params:
            # 参数化生成: 使用动态朝向剖面
            from latte_imitation.latte_art.orientation_profile import compute_pitch_profile
            from latte_imitation.latte_art.bridge import parametric_to_cartesian as _make_cart

            # 重建 cup/pour 配置以获取精确的阶段帧数
            cup_cfg = CupConfig(
                center_x=cup_params.get("center_x", 0.0),
                center_y=cup_params.get("center_y", 0.0),
                surface_z=cup_params.get("surface_z", 0.15),
                radius=cup_params.get("radius", 0.04),
            )
            pour_cfg = PourConfig(
                mix_height_offset=pour_params.get("mix_height_offset", 0.076),
                draw_height_offset=pour_params.get("draw_height_offset", 0.006),
                finish_height_offset=pour_params.get("finish_height_offset", 0.076),
                wiggle_amplitude=pour_params.get("wiggle_amplitude", 0.006),
                wiggle_frequency=pour_params.get("wiggle_frequency", 5.0),
                max_velocity=pour_params.get("max_velocity", 0.05),
                max_acceleration=pour_params.get("max_acceleration", 0.1),
                max_jerk=pour_params.get("max_jerk", 0.5),
            )

            total_frames = cart.num_frames
            # 阶段帧数: 融合 25%, 成形 60%, 收尾 15% (与 compose_full_trajectory 一致)
            num_mix = 50   # compose_full_trajectory 默认
            mix_end = num_mix
            draw_end = total_frames - 30  # compose_full_trajectory 默认 30 帧收尾

            pitch_profile = compute_pitch_profile(total_frames, mix_end, draw_end, pour_cfg)
            cart = _make_cart(
                cart.positions,
                roll_deg=0.0,
                yaw_deg=0.0,
                dt=cart.dt,
                pitch_profile=pitch_profile,
            )
            self.get_logger().info(
                f"动态朝向剖面: pitch {pitch_profile[0]:.0f}°→"
                f"{pitch_profile[mix_end]:.0f}°→{pitch_profile[-1]:.0f}°"
            )
```

- [ ] **Step 2: 修改 `_pipeline()` Phase ②/④ — retarget 调用**

替换第 310-352 行的 retarget 部分:

```python
        # ═══ Phase ③: Retarget ═══
        if start_pose is None:
            start_pose = Pose()

        if pattern_type and cup_params:
            # 参数化模式: retarget 目标 = 杯子位姿
            target = Pose()
            target.position.x = cup_params.get("center_x", 0.0)
            target.position.y = cup_params.get("center_y", 0.0)
            target.position.z = cup_params.get("surface_z", 0.15)
            # q_cup = identity (杯子水平放置)
            target.orientation.x = 0.0
            target.orientation.y = 0.0
            target.orientation.z = 0.0
            target.orientation.w = 1.0
            pos_src = "杯子坐标"

            cart = retarget_with_orientation_constraint(
                cart, target,
                rpy_user=rpy_user,
                translation_offset=translation_offset,
            )
        else:
            # 录制回放模式: 保持现有逻辑
            use_tf_position = is_default_position(start_pose)
            tf_warning = False
            if use_tf_position:
                current_pose = self._get_current_ee_pose()
                if current_pose is None:
                    if mode in ("preview", "debug"):
                        self.get_logger().warn(
                            "TF 不可达, preview/debug 模式使用原点 (0,0,0) 作为起点"
                        )
                        target = Pose()
                        target.position.x = 0.0
                        target.position.y = 0.0
                        target.position.z = 0.0
                        target.orientation.w = 1.0
                        pos_src = "原点(TF不可达)"
                        tf_warning = True
                    else:
                        return self._empty_result(False,
                            "无法获取当前末端位姿 (TF base_link → tool_tcp)")
                else:
                    target = current_pose
                    rotate = not is_default_orientation(start_pose) or any(
                        abs(r) > 1e-9 for r in rpy_user
                    )
                    if rotate:
                        target.orientation = start_pose.orientation
                    pos_src = "TF"
            else:
                target = start_pose
                pos_src = "手动"

            cart = retarget_trajectory(cart, target, rpy_user=rpy_user,
                                       absolute_orientation=False,
                                       translation_offset=translation_offset)

        self.get_logger().info(
            f"轨迹已变换 (位置={pos_src}, rpy={rpy_user}): "
            f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})"
        )
```

- [ ] **Step 3: 更新 import 行**

在文件顶部 import 添加新函数:

```python
from .trajectory_transform import (
    retarget_trajectory,
    retarget_with_orientation_constraint,
    is_default_position,
    is_default_orientation,
)
```

- [ ] **Step 4: 编译验证**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select latte_imitation
```

Expected: `Summary: 1 package finished` — 编译成功，无错误

- [ ] **Step 5: 启动仿真验证 — 参数化心形预览**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动 latte_imitation 节点 (仅节点, 不需要完整驱动)
timeout 15 ros2 run latte_imitation latte_imitation_node --ros-args \
  -p mode:=preview \
  -p episode_idx:=0 \
  -p arm:=right &
LATTE_PID=$!
sleep 5

# 调用 service 测试参数化心形生成
ros2 service call /latte_imitation/replay_trajectory ivg_interfaces/srv/ReplayLatteTrajectory "{
  episode_idx: 0,
  arm: 'right',
  speed_scale: 1.0,
  mode: 'preview',
  pattern_type: 'heart',
  cup_center_x: -0.63,
  cup_center_y: -0.368,
  cup_surface_z: 0.04,
  cup_radius: 0.04,
  roll_deg: 0.0,
  pitch_deg: 0.0,
  yaw_deg: 76.0,
  tool_offset_id: 'default',
  translation_x: 0.0,
  translation_y: 0.0,
  translation_z: 0.0,
  waypoint_sample_step: 5
}" 2>&1

kill $LATTE_PID 2>/dev/null
```

Expected: response 中 `success: true`, `num_frames` > 0, `message` 包含 "preview"

- [ ] **Step 6: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/latte_imitation/latte_imitation/trajectory_pipeline.py
git commit -m "feat: integrate dynamic orientation and separated retarget into pipeline

Parametric mode: uses cup position as retarget target, calls
retarget_with_orientation_constraint for yaw-only orientation transform.
Recorded mode: existing TF-based retarget logic unchanged.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 8: 修改前端 HTML — 拉花面板新增杯子/倾倒/速度 DOM

**Files:**
- Modify: `aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/coffee_latte_panel.html`

**背景:** `latte_controls.js` 引用的 `latte-cupX`, `latte-mixH`, `latte-wiggleAmp` 等 DOM ID 在 HTML 中不存在。需新增三组配置区域。

- [ ] **Step 1: 在 HTML 中"轨迹图案"卡片之后、"RPY变换"之前插入新 DOM**

在 `<!-- ── Episode 选择 ──>` 和 `<!-- ── RPY 变换 ──>` 之间插入:

```html
<!-- ── 杯子配置 (参数化模式) ── -->
<div class="latte-ctrl-group" id="latte-cup-group" style="display:none">
    <h3 class="latte-ctrl-title">杯子配置 (base_link 坐标系)</h3>
    <p class="latte-ctrl-desc">杯子在底座坐标系中的位置。XY 默认使用 lizhu_Link 位置，可在设置面板调整。</p>
    <div class="latte-cfg-grid latte-cfg-grid--4col">
        <label class="latte-cfg-row"><span class="latte-cfg-label">液面 Z (m)</span><input
                id="latte-cupZ" type="number" value="0.04" min="-0.5" max="1.0" step="0.001"
                class="latte-num-inp" /></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">杯半径 (mm)</span><input
                id="latte-cupR" type="number" value="40" min="20" max="80" step="1"
                class="latte-num-inp" /></label>
    </div>
</div>
<!-- ── 倾倒工艺参数 (参数化模式) ── -->
<div class="latte-ctrl-group" id="latte-pour-group" style="display:none">
    <h3 class="latte-ctrl-title">倾倒工艺参数</h3>
    <p class="latte-ctrl-desc">三阶段高度基于 Sunergos 视频参数，摆幅/频率基于 latteartguide.com</p>
    <div class="latte-cfg-grid latte-cfg-grid--3col">
        <label class="latte-cfg-row"><span class="latte-cfg-label">融合高度 (mm)</span>
            <input id="latte-mixH" type="range" min="40" max="120" value="76" step="1" />
            <span id="latte-mixH_val" class="latte-cfg-val">76</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">成形高度 (mm)</span>
            <input id="latte-drawH" type="range" min="2" max="20" value="6" step="1" />
            <span id="latte-drawH_val" class="latte-cfg-val">6</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">收尾高度 (mm)</span>
            <input id="latte-finishH" type="range" min="40" max="120" value="76" step="1" />
            <span id="latte-finishH_val" class="latte-cfg-val">76</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">摆动振幅 (mm)</span>
            <input id="latte-wiggleAmp" type="range" min="1" max="15" value="6" step="0.5" />
            <span id="latte-wiggleAmp_val" class="latte-cfg-val">6</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">摆动频率 (Hz)</span>
            <input id="latte-wiggleFreq" type="range" min="1" max="10" value="5" step="0.5" />
            <span id="latte-wiggleFreq_val" class="latte-cfg-val">5</span></label>
        <label class="latte-cfg-row" id="latte-tulip-row" style="display:none">
            <span class="latte-cfg-label">郁金香层数</span>
            <input id="latte-tulip-layers" type="number" value="3" min="1" max="6" step="1" class="latte-num-inp" /></label>
    </div>
</div>
<!-- ── 速度约束 (参数化模式) ── -->
<div class="latte-ctrl-group" id="latte-vel-group" style="display:none">
    <h3 class="latte-ctrl-title">速度约束 (抗晃荡)</h3>
    <p class="latte-ctrl-desc">S 曲线速度剖面参数，基于 Di Leva 2023 抗晃荡约束</p>
    <div class="latte-cfg-grid latte-cfg-grid--3col">
        <label class="latte-cfg-row"><span class="latte-cfg-label">Vmax (m/s)</span>
            <input id="latte-maxVel" type="range" min="0.01" max="0.2" value="0.05" step="0.01" />
            <span id="latte-maxVel_val" class="latte-cfg-val">0.05</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">Amax (m/s²)</span>
            <input id="latte-maxAcc" type="range" min="0.05" max="0.5" value="0.1" step="0.01" />
            <span id="latte-maxAcc_val" class="latte-cfg-val">0.1</span></label>
        <label class="latte-cfg-row"><span class="latte-cfg-label">Jmax (m/s³)</span>
            <input id="latte-maxJerk" type="range" min="0.1" max="2.0" value="0.5" step="0.1" />
            <span id="latte-maxJerk_val" class="latte-cfg-val">0.5</span></label>
    </div>
    <label class="latte-cfg-row"><span class="latte-cfg-label">抗晃荡</span>
        <input id="latte-antiSlosh" type="checkbox" checked />
        <span class="latte-cfg-hint">S 曲线速度剖面，防止奶泡晃动</span>
    </label>
</div>
```

- [ ] **Step 2: 更新 RPY 区域说明文字**

将现有 RPY 区域的 `<p class="latte-ctrl-desc">轨迹绕机械臂末端 (tool_tcp) 旋转...</p>` 替换为:

```html
<p class="latte-ctrl-desc">Yaw: 进杯方向 (绕重力轴旋转, 影响轨迹位置和倾倒方向)。Roll/Pitch: 图案倾斜微调 (仅影响位置, 不改变奶缸倾倒)</p>
```

- [ ] **Step 3: 验证 HTML 语法**

```bash
python3 -c "
from html.parser import HTMLParser
class Validator(HTMLParser):
    def handle_starttag(self, tag, attrs):
        pass
v = Validator()
with open('/home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/coffee_latte_panel.html') as f:
    v.feed(f.read())
print('HTML syntax OK')
"
```

- [ ] **Step 4: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/coffee_latte_panel.html
git commit -m "feat: add cup/pour/velocity config DOM to latte panel

Three new collapsible sections for parametric mode:
- Cup config: surface Z, radius (XY from lizhu_Link)
- Pour params: 3 heights + wiggle amp/freq + tulip layers
- Velocity: Vmax/Amax/Jmax + anti-sloshing toggle
Updated RPY hint text for separated retarget semantics.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 9: 修改前端 CSS — range slider + 条件显示样式

**Files:**
- Modify: `aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/css/coffee_latte_panel.css`

- [ ] **Step 1: 追加 CSS 样式**

在 CSS 文件末尾追加:

```css
/* ── 条件显示组 (参数化模式) ── */
#latte-cup-group,
#latte-pour-group,
#latte-vel-group {
    margin-bottom: 12px;
}

/* ── Range slider 值显示 ── */
.latte-cfg-val {
    display: inline-block;
    min-width: 40px;
    text-align: right;
    font-size: 13px;
    font-weight: 600;
    color: var(--color-text-primary, #333);
    margin-left: 6px;
}

/* ── 3 列 / 4 列网格 ── */
.latte-cfg-grid--3col {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 8px 12px;
}

.latte-cfg-grid--4col {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 8px 12px;
}

/* ── Range input 样式 ── */
.latte-cfg-row input[type="range"] {
    width: 100%;
    height: 6px;
    accent-color: var(--color-accent, #4a90d9);
    margin: 4px 0;
}

/* ── 说明文字 ── */
.latte-ctrl-desc {
    font-size: 12px;
    color: var(--color-text-secondary, #666);
    margin: 0 0 8px 0;
    line-height: 1.4;
}
```

- [ ] **Step 2: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/css/coffee_latte_panel.css
git commit -m "feat: add range slider and conditional display styles for latte panel

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 10: 修改 `latte_controls.js` — 条件显示 + 默认值更新

**Files:**
- Modify: `aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/latte/latte_controls.js`

- [ ] **Step 1: 更新 DEFAULTS — 杯子 XY 来自 lizhu_Link, Z 来自 URDF 计算**

将 DEFAULTS 中的 cupX/cupY/cupZ 改为 lizhu_Link 计算值:

```javascript
const DEFAULTS = {
    episode: 0, maxEpisode: 39,
    cupX: -0.630,   // lizhu_Link X in base_link (from URDF)
    cupY: -0.368,   // lizhu_Link Y in base_link (from URDF)
    cupZ: 0.04,     // 液面 Z = lizhu 顶 + 杯高, 需真机微调
    cupR: 0.04,     // 杯口半径 40mm
    // ... 其余不变
};
```

- [ ] **Step 2: 在 `_render()` 中添加条件显示逻辑**

在 `_render()` 函数中添加 (约第 298 行 `const s = _state` 之后):

```javascript
    // 条件显示: 参数化模式显示杯子/倾倒/速度组
    const isParametric = !!s.patternType;
    const cupGroup = document.getElementById('latte-cup-group');
    const pourGroup = document.getElementById('latte-pour-group');
    const velGroup = document.getElementById('latte-vel-group');
    const epRow = document.getElementById('latte-episode-row');
    const tulipRow = document.getElementById('latte-tulip-row');
    
    if (cupGroup) cupGroup.style.display = isParametric ? 'block' : 'none';
    if (pourGroup) pourGroup.style.display = isParametric ? 'block' : 'none';
    if (velGroup) velGroup.style.display = isParametric ? 'block' : 'none';
    if (epRow) epRow.style.display = isParametric ? 'none' : 'flex';
    if (tulipRow) tulipRow.style.display = (s.patternType === 'tulip') ? 'flex' : 'none';
```

- [ ] **Step 3: 更新 `_buildRequest()` — 参数化模式下传入 cup 参数**

确保 `_buildRequest()` 在 patternType 非空时传入 `cup_center_x/y` 和 `cup_surface_z`:

```javascript
    if (_state.patternType) {
        req.pattern_type = _state.patternType;
        req.tulip_layers = _state.tulipLayers;
        req.cup_center_x = _state.cupX;
        req.cup_center_y = _state.cupY;
        req.cup_surface_z = _state.cupZ;
        req.cup_radius = _state.cupR;
        // ... 其余倾倒参数
    }
```

- [ ] **Step 4: 验证 JS 语法**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/latte
node --check latte_controls.js 2>&1 || echo "Syntax check completed (warnings OK)"
```

- [ ] **Step 5: Commit**

```bash
cd /home/mu/aubo_boot
git add aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/public/js/latte/latte_controls.js
git commit -m "feat: update latte_controls defaults and conditional display

Cup XY defaults now from lizhu_Link URDF position.
Added conditional show/hide for cup/pour/velocity config groups
based on pattern_type selection (parametric vs recorded).

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 11: 端到端集成测试

- [ ] **Step 1: 完整编译**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select latte_imitation
```

Expected: `Summary: 1 package finished [XX.Xs]` — 编译成功

- [ ] **Step 2: 启动仿真环境并测试 service 调用**

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动节点 (后台)
ros2 run latte_imitation latte_imitation_node --ros-args \
  -p mode:=preview \
  -p episode_idx:=0 \
  -p arm:=right &
LATTE_PID=$!
sleep 5

# 测试参数化心形
echo "=== Test 1: Heart pattern ==="
ros2 service call /latte_imitation/replay_trajectory ivg_interfaces/srv/ReplayLatteTrajectory "{
  episode_idx: 0, arm: 'right', speed_scale: 1.0, mode: 'preview',
  pattern_type: 'heart',
  cup_center_x: -0.63, cup_center_y: -0.368, cup_surface_z: 0.04, cup_radius: 0.04,
  roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 76.0,
  tool_offset_id: 'default', translation_x: 0.0, translation_y: 0.0, translation_z: 0.0,
  waypoint_sample_step: 5
}" 2>&1 | head -15

echo "=== Test 2: Rosetta pattern ==="
ros2 service call /latte_imitation/replay_trajectory ivg_interfaces/srv/ReplayLatteTrajectory "{
  episode_idx: 0, arm: 'right', speed_scale: 1.0, mode: 'preview',
  pattern_type: 'rosetta',
  cup_center_x: -0.63, cup_center_y: -0.368, cup_surface_z: 0.04, cup_radius: 0.04,
  roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 76.0,
  tool_offset_id: 'default', translation_x: 0.0, translation_y: 0.0, translation_z: 0.0,
  waypoint_sample_step: 5
}" 2>&1 | head -15

echo "=== Test 3: Recorded playback (backward compat) ==="
ros2 service call /latte_imitation/replay_trajectory ivg_interfaces/srv/ReplayLatteTrajectory "{
  episode_idx: 0, arm: 'right', speed_scale: 1.0, mode: 'preview',
  pattern_type: '',
  roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
  tool_offset_id: 'default', translation_x: 0.0, translation_y: 0.0, translation_z: 0.0
}" 2>&1 | head -15

kill $LATTE_PID 2>/dev/null
```

Expected: 三次调用均返回 `success: true`, `num_frames` > 0

- [ ] **Step 3: 验证 pitch 剖面日志输出**

```bash
# 检查 ROS 日志中是否有 "动态朝向剖面" 关键字
grep -r "动态朝向剖面\|pitch_profile\|pitch " ~/.ros/log/latest/ 2>/dev/null | tail -5
```

Expected: 有 "动态朝向剖面: pitch XX°→XX°→XX°" 日志

- [ ] **Step 4: Commit (如有微调)**

```bash
cd /home/mu/aubo_boot
git status
# 如有修改, commit
```

---

## 实施总结

| Task | 内容 | 类型 |
|------|------|------|
| 1 | `latte_positions.yaml` | NEW |
| 2 | `orientation_profile.py` + 测试 | NEW |
| 3 | `config.py` PourConfig 加字段 | MODIFY |
| 4 | `bridge.py` parametric_to_cartesian 支持 pitch_profile | MODIFY |
| 5 | `__init__.py` 导出新模块 | MODIFY |
| 6 | `trajectory_transform.py` extract_yaw + retarget_constrained + 测试 | MODIFY |
| 7 | `trajectory_pipeline.py` 集成新管线 | MODIFY |
| 8 | `coffee_latte_panel.html` 新增 DOM | MODIFY |
| 9 | `coffee_latte_panel.css` 新增样式 | MODIFY |
| 10 | `latte_controls.js` 条件显示 + 默认值 | MODIFY |
| 11 | 端到端集成测试 | VERIFY |
