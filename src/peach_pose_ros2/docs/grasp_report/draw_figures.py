"""
生成 peach_pose_ros2 项目汇报配图（figures/ 下 6 张 PNG）.

用法（工作区根目录）:
    aubo_py3.12/bin/python src/peach_pose_ros2/docs/grasp_report/draw_figures.py

图清单:
    fig1_pipeline_overview.png  系统数据流与模块拓扑
    fig2_bag_geometry.png       袋装桃抓取几何（轴/底颈/入口/行程/误差锥）
    fig3_fruit_geometry.png     裸果桃抓取几何（球拟合 + 梗洼定向）
    fig4_gating_decision.png    三态安全门控决策流程
    fig5_error_budget.png       误差预算 vs 径向净空（门控边界数值图）
    fig6_param_influence.png    yaml 参数 → 管线环节 → 影响图谱

图中数值与 src/peach_pose_ros2/config/peach_pose.yaml 及
peach_pose/pipeline.py / contracts.py / fitting.py 的实现一一对应。
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC', 'Noto Sans CJK JP', 'Droid Sans Fallback',
    'AR PL UKai CN']
plt.rcParams['axes.unicode_minus'] = False

OUT = Path(__file__).resolve().parent / 'figures'
OUT.mkdir(parents=True, exist_ok=True)

# 与 config/peach_pose.yaml 对齐的工具参数（米）
D_INNER = 0.104
L_INSERT = 0.200
L_BLADE = 0.025
ENTRY_D_TOOL = 0.030
ENTRY_D_S = 0.040
STANDOFF = ENTRY_D_TOOL + ENTRY_D_S   # 0.070
CLEARANCE_MIN = 0.005
MARGIN_NECK = 0.015

C_BOX = '#eef4fb'
C_EDGE = '#33608a'
C_OK = '#2e9e4f'
C_WARN = '#d9a414'
C_BAD = '#c0392b'


def box(ax, x, y, w, h, text, fc=C_BOX, ec=C_EDGE, fs=10, lw=1.4, weight='normal'):
    """画圆角文本框，返回 (cx, cy)."""
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle='round,pad=0.008,rounding_size=0.012',
        fc=fc, ec=ec, lw=lw, mutation_aspect=0.6))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center',
            fontsize=fs, weight=weight, linespacing=1.35)
    return x + w / 2, y + h / 2


def arrow(ax, p0, p1, color='#444444', lw=1.6, style='-|>', shrinkA=2,
          shrinkB=2, ls='-', mutation=14):
    """两点间箭头."""
    ax.add_patch(FancyArrowPatch(
        p0, p1, arrowstyle=style, color=color, lw=lw, linestyle=ls,
        shrinkA=shrinkA, shrinkB=shrinkB, mutation_scale=mutation))


# ════════════════════════════════════════════════════════════════
# 图 1：系统数据流与模块拓扑
# ════════════════════════════════════════════════════════════════
def fig1():
    fig, ax = plt.subplots(figsize=(14.5, 9.2))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    ax.set_title('图 1  PeachPose 感知数据流：RGB-D → 检测 → 分割 → 几何 → 三态门控 → 输出',
                 fontsize=14, weight='bold', pad=14)

    # ── 第一列：输入 ──
    box(ax, 0.01, 0.86, 0.16, 0.09, 'RGB 彩图\n/camera/color/image_raw\n(bgr8)', fs=9)
    box(ax, 0.01, 0.73, 0.16, 0.09, '深度图\n/camera/depth/image_raw\n(uint16 / 32FC1)', fs=9)
    box(ax, 0.01, 0.60, 0.16, 0.09, '相机内参\n/camera/color/camera_info\n(K 矩阵)', fs=9)
    box(ax, 0.20, 0.68, 0.15, 0.14,
        '近似时间同步\nApproximateTimeSync\nsync_slop_s=0.05s\n（超 80% 允差 WARN）', fs=9)
    box(ax, 0.20, 0.50, 0.15, 0.12,
        '深度归一化\nnormalize_depth\nraw×depth_scale_unit\n=0.25 → uint16 毫米', fs=9)

    # ── 第二列：检测与分割 ──
    box(ax, 0.40, 0.80, 0.16, 0.11,
        'YOLO 检测\nyolo_conf=0.3（宽进）\nclass 0 袋 / class 1 裸果', fs=9)
    box(ax, 0.40, 0.64, 0.16, 0.10,
        '第二级过滤\nmin_detection_conf=0.5\n（严出，低于不入管线）', fs=9)
    box(ax, 0.40, 0.47, 0.16, 0.12,
        'MobileSAM 分割\n逐检测框出实例掩膜\n（缺失→mask_unavailable）', fs=9)

    # ── 第三列：前景与分流 ──
    box(ax, 0.61, 0.55, 0.17, 0.15,
        '前景掩膜 hybrid_dilated\n(SAM ∩ 有效深度) ∩\n膨胀(深度带连通域)\n<50px → REOBSERVE', fs=9)
    box(ax, 0.61, 0.30, 0.17, 0.17,
        '几何管线（只信实测深度）\nclass 0 → 袋线：圆柱 RANSAC 定轴\n'
        'class 1 → 果线：球拟合+梗洼定向\n降级：重力先验（置信封顶 0.4）', fs=9)

    # ── 第四列：门控与输出 ──
    c_gate = box(ax, 0.82, 0.42, 0.17, 0.16,
                 '三态安全门控\n净空 / 行程 / 轴来源 /\n误差预算 / 触边 等 12 类 flag\n'
                 '→ ACCEPT / REOBSERVE / REJECT', fs=9,
                 fc='#fdf3e0', ec=C_WARN)
    box(ax, 0.82, 0.72, 0.17, 0.13,
        'TF 坐标变换\noutput_frame=base_link\ntf_timeout_sec=0.5s\n失败打 tf_stale/tf_unavailable', fs=9)
    box(ax, 0.82, 0.14, 0.17, 0.20,
        '发布话题\n~/grasp_candidates（主输出）\n~/fitting（诊断指标）\n~/markers ~/debug_image ~/masks\n'
        '~/detection_cloud\n/peach/perception/*（规范化）', fs=9)

    # ── 参数块 ──
    box(ax, 0.20, 0.14, 0.36, 0.22,
        '参数源 config/peach_pose.yaml（28 个参数）\n'
        '重力: gravity_hint_xyz / gravity_mode(fixed|tf)\n'
        '工具: tool.D_inner / L_insert / L_blade /\n'
        '  entry_d_tool+entry_d_s / clearance_min / margin_neck\n'
        '追溯: model_version / calibration_version / tool.version',
        fc='#f2f2f2', ec='#888888', fs=9)

    # ── 连线 ──
    for y in (0.905, 0.775, 0.645):
        arrow(ax, (0.17, y), (0.20, 0.75))
    arrow(ax, (0.275, 0.68), (0.275, 0.62))                     # sync → 归一化
    arrow(ax, (0.35, 0.72), (0.40, 0.84))                       # sync → YOLO
    arrow(ax, (0.48, 0.80), (0.48, 0.74))                       # YOLO → 过滤
    arrow(ax, (0.48, 0.64), (0.48, 0.59))                       # 过滤 → SAM
    arrow(ax, (0.35, 0.56), (0.40, 0.53), ls='--')              # 深度 → SAM 同路
    arrow(ax, (0.56, 0.53), (0.61, 0.62))                       # SAM → 掩膜
    arrow(ax, (0.35, 0.54), (0.61, 0.60), ls='--')              # 深度 → 掩膜
    arrow(ax, (0.695, 0.55), (0.695, 0.47))                     # 掩膜 → 几何
    arrow(ax, (0.78, 0.42), (0.82, 0.46))                       # 几何 → 门控
    arrow(ax, (0.905, 0.58), (0.905, 0.72))                     # 门控 → TF（几何先出门控再变换）
    arrow(ax, (0.905, 0.72), (0.905, 0.58), ls='--', color='#999999')
    arrow(ax, (c_gate[0], 0.42), (c_gate[0], 0.34))             # 门控 → 发布
    arrow(ax, (0.38, 0.25), (0.61, 0.36), ls=':', color='#888888')  # 参数 → 管线
    ax.text(0.02, 0.03,
            '要点：几何只消费实测深度（禁学习补全）；节点只发参考位姿，不发送运动指令。',
            fontsize=10, color='#555555')
    fig.savefig(OUT / 'fig1_pipeline_overview.png', dpi=160,
                bbox_inches='tight')
    plt.close(fig)


# ════════════════════════════════════════════════════════════════
# 图 2：袋装桃抓取几何
# ════════════════════════════════════════════════════════════════
def fig2():
    fig, ax = plt.subplots(figsize=(11.5, 9.5))
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('图 2  袋装桃抓取几何：圆柱 RANSAC 定轴 → 底/颈/袋径 → 入口与行程',
                 fontsize=13.5, weight='bold', pad=12)

    # ── 袋体（侧视轮廓，轴竖直） ──
    ys = np.linspace(0.0, 0.12, 60)
    prof = 0.047 - 0.010 * (ys / 0.12) + 0.006 * np.sin(np.pi * ys / 0.12)
    bag_x = np.concatenate([-prof, prof[::-1]])
    bag_y = np.concatenate([ys, ys[::-1]])
    ax.add_patch(Polygon(np.column_stack([bag_x, bag_y]), closed=True,
                         fc='#fdebd0', ec='#b9770e', lw=1.6, alpha=0.85,
                         zorder=2))
    rng = np.random.default_rng(3)
    # 实测点云：只画“可见一侧”的表面点，呼应单目 RGB-D 只见正面
    pt_y = rng.uniform(0.01, 0.115, 260)
    pt_r = 0.047 - 0.010 * (pt_y / 0.12) + 0.006 * np.sin(np.pi * pt_y / 0.12)
    pt_x = pt_r + rng.normal(0, 0.0012, len(pt_y))
    ax.scatter(pt_x, pt_y, s=7, c='#33608a', alpha=0.7, zorder=3,
               label='实测深度点云（可见面）')

    # ── 拟合轴（底→颈 = 逆重力） ──
    ax.annotate('', xy=(0, 0.155), xytext=(0, -0.035),
                arrowprops={'arrowstyle': '-|>', 'color': C_BAD, 'lw': 2.2})
    ax.plot([0, 0], [-0.10, 0.19], '--', color=C_BAD, lw=1.0, alpha=0.5)
    ax.text(0.006, 0.160, '袋轴 axis = 底→颈（逆重力 −g）\n圆柱 RANSAC 主估 / 重力先验降级',
            fontsize=9.5, color=C_BAD, ha='left')

    # ── 底 / 颈 / 入口 ──
    ax.plot(0, 0.0, 'o', ms=9, color=C_OK, zorder=4)
    ax.annotate('P_bottom 袋底（轴投影 P10 带中位）', xy=(0, 0.0),
                xytext=(-0.205, 0.012), fontsize=9.5, color='#1d6b33',
                arrowprops={'arrowstyle': '-', 'color': '#1d6b33', 'lw': 0.8})
    ax.plot(0, 0.12, 'o', ms=9, color=C_OK, zorder=4)
    ax.annotate('P_neck 袋颈（P90 带中位，观测下界）', xy=(0, 0.12),
                xytext=(-0.205, 0.126), fontsize=9.5, color='#1d6b33',
                arrowprops={'arrowstyle': '-', 'color': '#1d6b33', 'lw': 0.8})
    entry_y = -STANDOFF
    ax.plot(0, entry_y, 's', ms=10, color='#8e44ad', zorder=4)
    ax.annotate('entry_start = P_bottom − standoff·axis\n（= 工具顶面圆心 = 末端 TCP 目标位）',
                xy=(0, entry_y), xytext=(-0.155, entry_y - 0.017),
                fontsize=9.5, color='#8e44ad',
                arrowprops={'arrowstyle': '-', 'color': '#8e44ad', 'lw': 0.8})

    # ── standoff 分解标注 ──
    ax.annotate('', xy=(-0.062, 0.0), xytext=(-0.062, entry_y),
                arrowprops={'arrowstyle': '<->', 'color': '#555555', 'lw': 1.2})
    ax.text(-0.115, entry_y / 2,
            'standoff = entry_d_tool(30mm)\n+ entry_d_s(40mm) = 70mm',
            fontsize=9, ha='center', color='#333333')

    # ── 工具圆柱（空心，待套入） ──
    half = D_INNER / 2
    tool_top, tool_bot = entry_y, entry_y - L_INSERT
    wall = 0.006
    ax.add_patch(Polygon([(-half - wall, tool_top), (-half, tool_top),
                          (-half, tool_bot), (-half - wall, tool_bot)],
                         closed=True, fc='#aeb6bf', ec='#566573', zorder=1))
    ax.add_patch(Polygon([(half, tool_top), (half + wall, tool_top),
                          (half + wall, tool_bot), (half, tool_bot)],
                         closed=True, fc='#aeb6bf', ec='#566573', zorder=1))
    ax.plot([-half - wall, half + wall], [tool_bot, tool_bot],
            color='#566573', lw=1.4)
    ax.text(half + 0.012, tool_top - 0.09,
            '套袋工具（空心圆柱）\nD_inner=104mm\nL_insert=200mm（行程上限）',
            fontsize=9.5, va='center')
    ax.annotate('', xy=(-half, tool_bot - 0.012), xytext=(half, tool_bot - 0.012),
                arrowprops={'arrowstyle': '<->', 'color': '#566573', 'lw': 1.1})
    ax.text(0, tool_bot - 0.024, 'D_inner', fontsize=9, ha='center',
            color='#566573')

    # ── 建议行程 ──
    s_neck = 0.12 - entry_y - L_BLADE
    s_max = min(s_neck - MARGIN_NECK, L_INSERT)
    travel_end = entry_y + s_max
    ax.annotate('', xy=(0.062, travel_end), xytext=(0.062, entry_y),
                arrowprops={'arrowstyle': '<->', 'color': C_OK, 'lw': 2.0})
    ax.plot(0, travel_end, 'D', ms=7, color=C_OK, zorder=4)
    ax.text(0.068, (entry_y + travel_end) / 2,
            'suggested_travel\n= (P_neck − entry)·axis − L_blade\n'
            '  − margin_neck，再被 L_insert 封顶\n= 190−25−15 = 150mm',
            fontsize=9, va='center', color='#1d6b33')
    ax.annotate('刀刃停于袋颈前\nL_blade=25mm + margin_neck=15mm',
                xy=(0.012, 0.12 - L_BLADE - MARGIN_NECK), xytext=(0.10, 0.065),
                fontsize=8.5, color='#7d3c98',
                arrowprops={'arrowstyle': '->', 'color': '#7d3c98', 'lw': 0.9})

    # ── 误差锥（误差预算） ──
    theta = np.radians(10)
    for sgn in (-1, 1):
        ax.plot([0, sgn * (STANDOFF + s_max) * np.sin(theta)],
                [entry_y, travel_end], ':', color=C_WARN, lw=1.6)
    ax.annotate('误差锥 ±θ_err\nδ=(standoff+travel)·sinθ_err\n须 ≤ 径向净空',
                xy=(-0.035, 0.045), xytext=(-0.16, 0.075), fontsize=9.5,
                color='#9a7d0a',
                arrowprops={'arrowstyle': '->', 'color': C_WARN, 'lw': 1.0})

    # ── 径向净空剖面 ──
    ax.annotate('', xy=(-0.047, 0.135), xytext=(-half, 0.135),
                arrowprops={'arrowstyle': '-', 'color': C_BAD, 'lw': 3.0})
    ax.annotate('', xy=(0.047, 0.135), xytext=(half, 0.135),
                arrowprops={'arrowstyle': '-', 'color': C_BAD, 'lw': 3.0})
    ax.text(0.062, 0.135,
            '径向净空 = (D_inner − D_bag)/2 − clearance_min\n'
            'D_bag = 横向半径 P95 × 2（保守上界）',
            fontsize=9, va='center', color=C_BAD)

    # ── 重力 ──
    ax.annotate('', xy=(-0.135, -0.13), xytext=(-0.135, -0.075),
                arrowprops={'arrowstyle': '-|>', 'color': '#333333', 'lw': 1.6})
    ax.text(-0.135, -0.145, '重力 g（gravity_hint / tf 反推）',
            fontsize=9, ha='center')

    # ── 抓取系 ──
    ax.text(-0.185, 0.165,
            '抓取系 R=[Xg,Yg,Zg]\nZg=axis（套入轴）\nXg=⊥Zg 最大方差方向',
            fontsize=9, color='#33608a',
            bbox={'fc': '#eef4fb', 'ec': '#33608a', 'boxstyle': 'round,pad=0.35'})

    ax.set_xlim(-0.21, 0.21)
    ax.set_ylim(-0.30, 0.20)
    ax.legend(loc='lower right', fontsize=9)
    fig.savefig(OUT / 'fig2_bag_geometry.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


# ════════════════════════════════════════════════════════════════
# 图 3：裸果桃抓取几何
# ════════════════════════════════════════════════════════════════
def fig3():
    fig, ax = plt.subplots(figsize=(11.5, 8.8))
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('图 3  裸果桃抓取几何：球拟合定心定径 + 梗洼定向（重力极性校正）',
                 fontsize=13.5, weight='bold', pad=12)

    r = 0.040
    # 拟合球
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(r * np.cos(th), r * np.sin(th), color='#b9770e', lw=2.0,
            label='拟合球（半径夹紧 [25,45]mm）')
    ax.add_patch(Polygon(np.column_stack([r * np.cos(th), r * np.sin(th)]),
                         closed=True, fc='#fdebd0', ec='none', alpha=0.55,
                         zorder=0))
    # 可见面点云（相机在左侧 → 只画左半球）
    rng = np.random.default_rng(7)
    ang = rng.uniform(np.pi / 2 - 1.05, np.pi / 2 + 1.05, 300) + np.pi / 2
    rad = r + rng.normal(0, 0.0011, len(ang))
    # 梗洼：顶部局部下陷
    dip_mask = np.abs(ang - np.pi / 2) < 0.22
    rad = rad.copy()
    rad[dip_mask] -= 0.004 * (1 - np.abs(ang[dip_mask] - np.pi / 2) / 0.22)
    ax.scatter(rad * np.cos(ang), rad * np.sin(ang), s=8, c='#33608a',
               alpha=0.75, zorder=3, label='实测深度点云（面向相机半球）')

    # 相机
    ax.annotate('', xy=(-0.085, 0), xytext=(-0.135, 0),
                arrowprops={'arrowstyle': '-|>', 'color': '#555555', 'lw': 1.5})
    ax.text(-0.14, 0.006, 'RGB-D 相机', fontsize=9.5, ha='center')

    # 梗洼帽（20° 扫描帽 + 下陷标注）
    cap_th = np.linspace(np.pi / 2 - np.radians(20), np.pi / 2 + np.radians(20), 30)
    ax.plot(1.18 * r * np.cos(cap_th), 1.18 * r * np.sin(cap_th),
            color=C_BAD, lw=2.4)
    ax.annotate('梗洼：实测面相对拟合球面\n径向残差 P30 < −1.5mm 的方向帽\n'
                '（Fibonacci 球面 ~200 向扫描）',
                xy=(r * 1.18 * np.cos(np.pi / 2 + 0.3), r * 1.18 * np.sin(np.pi / 2 + 0.3)),
                xytext=(0.062, 0.062), fontsize=9.5, color=C_BAD,
                arrowprops={'arrowstyle': '->', 'color': C_BAD, 'lw': 1.0})

    # 套入轴
    ax.annotate('', xy=(0, 0.082), xytext=(0, -0.062),
                arrowprops={'arrowstyle': '-|>', 'color': C_BAD, 'lw': 2.2})
    ax.plot([0, 0], [-0.085, 0.095], '--', color=C_BAD, lw=1.0, alpha=0.5)
    ax.text(0.006, -0.092, '套入轴 = 底→梗端（颈）\nstem_cavity 主估 / 重力先验降级',
            fontsize=9.5, color=C_BAD, ha='left')

    # 底 / 颈 / 球心
    ax.plot(0, -r, 'o', ms=9, color=C_OK, zorder=4)
    ax.plot(0, r, 'o', ms=9, color=C_OK, zorder=4)
    ax.plot(0, 0, '+', ms=13, mew=2.2, color='#1d6b33', zorder=4)
    ax.annotate('P_bottom = 球心 − r·axis', xy=(0, -r), xytext=(-0.155, -0.052),
                fontsize=9.5, color='#1d6b33',
                arrowprops={'arrowstyle': '-', 'color': '#1d6b33', 'lw': 0.8})
    ax.annotate('P_neck = 球心 + r·axis（梗端）', xy=(0, r), xytext=(-0.16, 0.048),
                fontsize=9.5, color='#1d6b33',
                arrowprops={'arrowstyle': '-', 'color': '#1d6b33', 'lw': 0.8})
    ax.text(0.004, -0.006, '球心（点+法线 RANSAC\n+ 几何 LM 抛光）', fontsize=9,
            color='#1d6b33')

    # entry 与工具
    entry_y = -r - STANDOFF
    ax.plot(0, entry_y, 's', ms=10, color='#8e44ad', zorder=4)
    ax.annotate('entry_start = P_bottom − 70mm·axis\n（公式与袋线相同）',
                xy=(0, entry_y), xytext=(0.066, entry_y - 0.010), fontsize=9.5,
                color='#8e44ad',
                arrowprops={'arrowstyle': '-', 'color': '#8e44ad', 'lw': 0.8})
    half = D_INNER / 2
    ax.plot([-half, -half], [entry_y, entry_y - 0.05], color='#566573', lw=5,
            solid_capstyle='butt')
    ax.plot([half, half], [entry_y, entry_y - 0.05], color='#566573', lw=5,
            solid_capstyle='butt')
    ax.plot([-half, half], [entry_y - 0.05, entry_y - 0.05], color='#566573',
            lw=1.4)
    ax.text(half + 0.008, entry_y - 0.025, '同一圆柱套袋工具', fontsize=9,
            va='center')

    # 重力极性校正说明
    ax.annotate('', xy=(-0.105, -0.10), xytext=(-0.105, -0.045),
                arrowprops={'arrowstyle': '-|>', 'color': '#333333', 'lw': 1.6})
    ax.text(-0.105, -0.116, '重力 g', fontsize=9.5, ha='center')
    ax.text(-0.165, 0.10,
            '极性校正（Kok 2024 对极点教训）\n'
            '检测到洼朝下 ⇒ 是萼洼非梗洼\n'
            '⇒ 翻轴 + 置信度×0.7 + θ_err+5°\n'
            '（不直接放弃，显式降级）',
            fontsize=9, color='#7d3c98',
            bbox={'fc': '#f5eef8', 'ec': '#7d3c98', 'boxstyle': 'round,pad=0.4'})

    # 袋径取保守大者
    ax.annotate('D_bag = max(2r, 横向 P95 直径)\n→ 与袋线同一净空门控',
                xy=(r * 0.72, -r * 0.72), xytext=(0.06, -0.075), fontsize=9.5,
                color=C_BAD,
                arrowprops={'arrowstyle': '->', 'color': C_BAD, 'lw': 0.9})

    ax.set_xlim(-0.175, 0.175)
    ax.set_ylim(-0.175, 0.125)
    ax.legend(loc='lower right', fontsize=9)
    fig.savefig(OUT / 'fig3_fruit_geometry.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


# ════════════════════════════════════════════════════════════════
# 图 4：三态安全门控决策流程
# ════════════════════════════════════════════════════════════════
def fig4():
    fig, ax = plt.subplots(figsize=(14.5, 9.6))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    ax.set_title('图 4  三态安全门控：硬性失败 → REJECT，软标记 → REOBSERVE，全净 → ACCEPT',
                 fontsize=13.5, weight='bold', pad=14)

    # ── 左列：硬性失败链（任一命中直接 REJECT，不进门控表） ──
    hard = [
        ('bbox < 8×8 px', 'invalid_bbox'),
        ('有效前景点 < 100', 'insufficient_measured_points'),
        ('重力向量无效', 'invalid_gravity'),
        ('袋轴长度 ≤ 30mm', 'bag_axis_too_short'),
        ('SAM 掩膜不可用\n（缺失/交后 <50px）', 'mask_unavailable → REOBSERVE'),
    ]
    ax.text(0.045, 0.955, '前置硬性检查（管线入口，逐项短路）', fontsize=11,
            weight='bold', color=C_BAD)
    y = 0.895
    for name, flag in hard:
        box(ax, 0.02, y - 0.062, 0.185, 0.062,
            f'{name}\n[{flag}]', fs=8.6, fc='#fdecea', ec=C_BAD)
        if y < 0.895:
            arrow(ax, (0.1125, y + 0.005), (0.1125, y - 0.0), color=C_BAD)
        y -= 0.082
    arrow(ax, (0.1125, y + 0.077), (0.30, 0.46), color=C_BAD, ls='--')
    ax.text(0.19, 0.47, '命中\n→', fontsize=9, color=C_BAD, ha='center')

    # ── 中列：门控 flag 判定表 ──
    box(ax, 0.30, 0.80, 0.32, 0.075, '几何量齐备 → 汇总门控标记 flags',
        fs=10.5, weight='bold')
    flags_doc = [
        ('tool_clearance_failed',
         'D_bag + 2·clearance_min ≥ D_inner\n（袋径上界 + 余量 ≥ 工具内径）→ 唯一硬性 REJECT', C_BAD),
        ('error_budget_exceeded',
         '误差预算 δ=(standoff+travel)·sinθ_err\n> 径向净空（图 5）', C_WARN),
        ('travel_too_short', '建议行程 < 50mm', C_WARN),
        ('low_valid_depth', 'ROI 有效深度占比 < 40%', C_WARN),
        ('small_foreground', '前景覆盖率 < 1%', C_WARN),
        ('axis_from_gravity_prior', '轴来自重力先验（拟合失败降级）', C_WARN),
        ('axis_orientation_uncertain', '轴近水平：|axis·g| < 0.3（袋）/ 0.2（果）', C_WARN),
        ('axis_2d_mismatch', '3D 轴投影与掩膜 2D 主轴夹角 > 45°', C_WARN),
        ('foreground_truncated', '掩膜触边 >15% 或 ≥3 边（截断/遮挡）', C_WARN),
        ('sphere_fit_unstable', '（果线）球拟合内点率 < 35%', C_WARN),
        ('tf_stale / tf_unavailable', '（节点层）TF 回退/失败追加标记', C_WARN),
    ]
    ax.text(0.30, 0.775, '门控标记（flags，可叠加多个）', fontsize=11,
            weight='bold', color=C_EDGE)
    y = 0.745
    for name, desc, color in flags_doc:
        box(ax, 0.30, y - 0.052, 0.115, 0.052, name, fs=8.2,
            fc='#fdecea' if color is C_BAD else '#fdf6e3',
            ec=color)
        ax.text(0.425, y - 0.026, desc, fontsize=8.4, va='center',
                color='#333333')
        y -= 0.062

    # ── 右列：三态输出 ──
    arrow(ax, (0.62, 0.84), (0.70, 0.60))
    box(ax, 0.68, 0.53, 0.13, 0.10,
        '状态判定\n（pipeline.py 末尾）', fs=10, weight='bold')
    box(ax, 0.855, 0.74, 0.135, 0.09, 'ACCEPT (0)\nflags 为空\n可据此位姿执行',
        fs=9.5, fc='#e8f6ec', ec=C_OK, weight='bold')
    box(ax, 0.855, 0.50, 0.135, 0.10, 'REOBSERVE (1)\n仅有软标记\n换视角重采再判',
        fs=9.5, fc='#fdf6e3', ec=C_WARN, weight='bold')
    box(ax, 0.855, 0.24, 0.135, 0.10, 'REJECT (2)\n含 tool_clearance_failed\n或前置硬性失败 → 禁止动作',
        fs=9.5, fc='#fdecea', ec=C_BAD, weight='bold')
    arrow(ax, (0.81, 0.60), (0.855, 0.76), color=C_OK)
    ax.text(0.815, 0.70, '无 flag', fontsize=9, color=C_OK)
    arrow(ax, (0.81, 0.575), (0.855, 0.555), color=C_WARN)
    ax.text(0.815, 0.555, '软标记', fontsize=9, color=C_WARN)
    arrow(ax, (0.81, 0.545), (0.855, 0.30), color=C_BAD)
    ax.text(0.815, 0.40, '硬性失败', fontsize=9, color=C_BAD)

    ax.text(0.02, 0.035,
            '设计原则：宁缺毋滥 —— 任何信息不足都显式降级为 REOBSERVE/REJECT 并带诊断标记，'
            '绝不静默放行；错误 ACCEPT 是离线评估的首要安全指标（Beta 95% 上界须 <1%）。',
            fontsize=10, color='#555555')
    fig.savefig(OUT / 'fig4_gating_decision.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


# ════════════════════════════════════════════════════════════════
# 图 5：误差预算 vs 径向净空（数值图）
# ════════════════════════════════════════════════════════════════
def fig5():
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.6))
    fig.suptitle('图 5  误差预算门控：δ = (standoff + travel)·sin(θ_err) ≤ 径向净空',
                 fontsize=13.5, weight='bold')

    # ── (a) 预算曲线 vs 净空水平线 ──
    ax = axes[0]
    th = np.linspace(0, 30, 200)
    for travel, ls in ((0.10, '-'), (0.15, '--'), (0.20, ':')):
        ax.plot(th, (STANDOFF + travel) * np.sin(np.radians(th)) * 1000, ls,
                lw=2.0, label=f'误差预算 δ（travel={travel * 1000:.0f}mm）')
    for d_bag, c in ((0.080, C_OK), (0.090, C_WARN), (0.094, C_BAD)):
        clearance = (D_INNER - d_bag) / 2 - CLEARANCE_MIN
        ax.axhline(clearance * 1000, color=c, lw=1.6, alpha=0.85)
        ax.text(29.6, clearance * 1000 + 0.25,
                f'净空（D_bag={d_bag * 1000:.0f}mm）={clearance * 1000:.0f}mm',
                fontsize=8.6, ha='right', color=c)
    ax.set_xlabel('θ_err 轴角误差（°）')
    ax.set_ylabel('毫米 (mm)')
    ax.set_title('(a) 预算曲线高于净空线 ⇒ error_budget_exceeded', fontsize=11)
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 30)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc='upper left')

    # ── (b) 可容忍最大轴角误差 vs 袋径 ──
    ax = axes[1]
    d_bag_mm = np.linspace(60, 103, 200)
    clearance_mm = (D_INNER * 1000 - d_bag_mm) / 2 - CLEARANCE_MIN * 1000
    for travel, ls in ((0.10, '-'), (0.15, '--'), (0.20, ':')):
        arm_mm = (STANDOFF + travel) * 1000
        ratio = np.clip(clearance_mm / arm_mm, 0, 1)
        th_max = np.degrees(np.arcsin(ratio))
        ax.plot(d_bag_mm, th_max, ls, lw=2.0,
                label=f'travel={travel * 1000:.0f}mm（臂长 {arm_mm:.0f}mm）')
    # 工作点标注：D_bag=90mm, travel=150mm
    wp_d, wp_t = 90.0, 0.15
    wp_cl = (D_INNER * 1000 - wp_d) / 2 - CLEARANCE_MIN * 1000
    wp_th = np.degrees(np.arcsin(np.clip(wp_cl / ((STANDOFF + wp_t) * 1000), 0, 1)))
    ax.plot(wp_d, wp_th, 'o', ms=9, color=C_BAD, zorder=5)
    ax.annotate(f'例：D_bag=90mm, travel=150mm\nθ_err 须 ≤ {wp_th:.1f}°',
                xy=(wp_d, wp_th), xytext=(70, 8), fontsize=9.5, color=C_BAD,
                arrowprops={'arrowstyle': '->', 'color': C_BAD, 'lw': 1.0})
    ax.axvline(D_INNER * 1000 - 2 * CLEARANCE_MIN * 1000, color=C_BAD,
               ls='--', lw=1.2, alpha=0.6)
    ax.text(94.5, 27, 'tool_clearance_failed 边界\nD_bag ≥ D_inner − 2·clearance_min',
            fontsize=8.4, color=C_BAD)
    ax.set_xlabel('袋径上界 D_bag（mm）')
    ax.set_ylabel('可容忍 θ_err 上限（°）')
    ax.set_title('(b) 袋越粗 / 行程越长 ⇒ 对轴向精度要求越苛刻', fontsize=11)
    ax.set_xlim(60, 103)
    ax.set_ylim(0, 32)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(OUT / 'fig5_error_budget.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


# ════════════════════════════════════════════════════════════════
# 图 6：yaml 参数 → 管线环节 → 影响图谱（表格式）
# ════════════════════════════════════════════════════════════════
def fig6():
    fig, ax = plt.subplots(figsize=(14.0, 10.6))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    ax.set_title('图 6  config/peach_pose.yaml 参数 → 作用环节 → 对抓取判定/位姿的影响',
                 fontsize=13.5, weight='bold', pad=14)

    groups = [
        ('输入与同步', '#33608a', [
            ('sync_slop_s = 0.05', '三路近似同步允差',
             '过大→错帧配对；过小→丢帧不回调'),
            ('depth_scale_unit = 0.25', '深度归一化',
             'Percipio 原始值→毫米；设错 = 全部几何尺度错误'),
            ('tf_timeout_sec = 0.5', 'TF 查询',
             '超时→退回相机系，并打 tf_unavailable 标记'),
        ]),
        ('检测与分割', '#7d3c98', [
            ('yolo_conf = 0.3', 'YOLO 推理阈值（宽进）',
             '调低→召回↑误检↑；调高→漏检'),
            ('min_detection_conf = 0.5', '入管线置信下限（严出）',
             '两级过滤主闸：直接决定进入几何的目标集'),
        ]),
        ('重力先验', '#1d6b33', [
            ('gravity_hint_xyz = ""', '相机系重力方向',
             '空 = 默认 +Y（正装朝下）；相机装歪必须改，否则降级轴向错误'),
            ('gravity_mode = fixed', '重力来源 fixed | tf',
             'tf = 按本帧 TF 反推，随机械臂姿态自适应'),
        ]),
        ('工具几何（换工具必改）', '#c0392b', [
            ('tool.D_inner = 0.104', '工具内径',
             '净空门控基准：直接决定 tool_clearance_failed 边界'),
            ('tool.clearance_min = 0.005', '最小径向余量',
             '调大→ACCEPT 变严（更安全但更挑果）'),
            ('tool.entry_d_tool/_s = 0.030/0.040', '入口 standoff',
             '决定 entry_start 位置；同时加长误差臂 (standoff+travel)'),
            ('tool.L_insert = 0.200', '最大插入深度',
             'suggested_travel 上限'),
            ('tool.L_blade = 0.025', '刃口长度',
             '行程在袋颈前扣掉；改错→切不断或撞颈'),
            ('tool.margin_neck = 0.015', '颈前安全余量',
             '调大→行程变短→可能 travel_too_short'),
        ]),
        ('输出与追溯', '#555555', [
            ('output_frame = base_link', '输出坐标系',
             '几何经 TF 变到机械臂基座系供下游使用'),
            ('publish_* / detection_cloud_stride', '可视化开关',
             '只影响调试输出与 RViz 负载，不影响判定'),
            ('model/calibration/tool version', '版本标识',
             '仅随结果发布用于追溯，不参与计算'),
        ]),
    ]

    X_GROUP, X_PARAM, X_STAGE, X_EFFECT = 0.015, 0.135, 0.395, 0.62
    y = 0.945
    ROW = 0.036
    # 表头
    for x, t in ((X_GROUP, '分组'), (X_PARAM, '参数（yaml 默认值）'),
                 (X_STAGE, '作用环节'), (X_EFFECT, '调参影响')):
        ax.text(x, y, t, fontsize=10.5, weight='bold', color='#222222')
    y -= ROW * 0.7
    ax.plot([0.01, 0.99], [y, y], color='#333333', lw=1.2)
    y -= ROW * 0.55
    for gtitle, color, params in groups:
        # 分组行
        ax.add_patch(plt.Rectangle((0.01, y - ROW * 0.72), 0.98, ROW * 0.9,
                                   fc=color, alpha=0.10, ec='none'))
        ax.text(X_GROUP, y - ROW * 0.38, gtitle, fontsize=10.5,
                weight='bold', color=color, va='center')
        y -= ROW
        for name, stage, effect in params:
            ax.text(X_PARAM, y - ROW * 0.38, name, fontsize=9.3,
                    family='monospace', color='#111111', va='center')
            ax.text(X_STAGE, y - ROW * 0.38, stage, fontsize=9.3,
                    color=color, va='center', weight='bold')
            ax.text(X_EFFECT, y - ROW * 0.38, effect, fontsize=9.3,
                    color='#333333', va='center')
            ax.plot([0.01, 0.99], [y - ROW * 0.82, y - ROW * 0.82],
                    color='#dddddd', lw=0.7)
            y -= ROW
        y -= ROW * 0.30

    # 底部结论条
    ax.add_patch(plt.Rectangle((0.01, 0.008), 0.98, 0.062,
                               fc='#fdf6e3', ec=C_WARN, lw=1.2))
    ax.text(0.03, 0.052, '对可抓取判定影响最大：', fontsize=9.8,
            weight='bold', color='#9a7d0a')
    ax.text(0.03, 0.026,
            'tool.D_inner / clearance_min → 净空门控（硬性 REJECT）；'
            'entry_d_tool + entry_d_s → 误差预算臂长（软标记）；'
            'min_detection_conf → 进入几何的目标集；gravity_* → 降级轴向正确性',
            fontsize=9.3, color='#333333')
    ax.text(0.03, 0.078,
            '只改位姿不改判定：L_insert / L_blade / margin_neck → entry 位置与 suggested_travel；'
            'output_frame → 输出坐标系。参数仅启动时读取，改后须重启节点。',
            fontsize=9.3, color='#777777')
    fig.savefig(OUT / 'fig6_param_influence.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


fig1()
fig2()
fig3()
fig4()
fig5()
fig6()
print('all figures ->', OUT)
