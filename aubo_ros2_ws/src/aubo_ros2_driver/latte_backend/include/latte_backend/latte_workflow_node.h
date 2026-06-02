#ifndef LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_
#define LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "demo_driver/robot_controller.h"
#include <ivg_interfaces/srv/run_latte_workflow.hpp>

// ── 调试辅助: 四元数 → RPY(度) → 格式化字符串 ──
namespace latte_backend {
std::string quatToRPYStr(const geometry_msgs::msg::Quaternion& q);
std::string poseToStr(const geometry_msgs::msg::Pose& p);
}

namespace latte_backend
{

using demo_driver::RobotController;

/// ═══════════════════════════════════════════════════════════════════════════
/// 心形轨迹生成参数 (基于 Barista Hustle MSLA / SCA WLAC 权威资料) 喵~
///
/// 图案结构 (MSLA Ch.5 — Heart = 双元素设计):
///
///       融合画圈 (僧帽白圆)              划穿收尾 (心形尖部)
///       ╭──────────────────╮          ╭──────────────────╮
///       │   r=1cm, 2圈      │    +     │  沿Y轴穿过圆心     │  =  ♥
///       │   z=80mm 高位注入  │          │  15mm直线推进     │
///       ╰──────────────────╯          ╰──────────────────╯
///
/// 坐标系约定: 代码中所有 "roll" 指 **绕世界坐标系 X 轴的倾角** (非 TCP body-fixed roll)
///
///     世界 X 轴 (前) → 倾倒倾角轴: 绕此轴旋转使奶缸前倾(+)/后仰(-)
///     世界 Y 轴 (左) → 划穿方向轴: 奶缸嘴沿此方向倒奶, cut-through 沿此方向
///     世界 Z 轴 (上) → 高度轴: 液面上方距离
///
///     step4 已注释 (仅测试用), roll 从水平 (0°) 直接起算, 无需叠加基准
///
/// 倾角-流量映射 (MSLA 4.04: 倾角越大→重力沿流出方向分量越大→流速越高):
///
///     倾角          流速         线宽       阶段          物理效果
///     ────────────────────────────────────────────────────────────
///     45°          ~10 ml/s      ~3mm       融合 mix       细流穿透 crema 沉底
///     50°          ~15 ml/s      ~4mm       收尾 finish    中速精准切割
///     60°          ~20 ml/s      ~5mm       成形 draw      高流量泡沫浮面扩散
///
///   MSLA 4.04 公式: 线宽 ∝ √流量, 流量 +100% → 线宽 +42%
///
/// 参数权威来源对照:
///
///     参数                 MSLA 章节         值            依据
///     ──────────────────────────────────────────────────────────────
///     mix_height           MSLA 5.02         80mm          高位倾倒 7-10cm
///     draw_height          MSLA 4.02          5mm          低位作画 <1cm
///     mix_circle_r         MSLA 5.02         10mm          硬币大小 ~1cm
///     mix_circles          MSLA 5.02          2            足够建立清晰白圆
///     push_y               MSLA 5.02-5.05    15mm          穿过圆心产生尖部
///     sway_offset_y        MSLA 5.02         10mm          白圆偏向前方留划穿空间
///     roll_mix             MSLA 4.04         45° (~10ml/s) Pin-Drop 细流穿透
///     roll_draw_start      MSLA 4.04         60° (~20ml/s) 高流量建立白圆基底
///     roll_draw_end        MSLA 4.04         45° (~10ml/s) 降低流量防止溢出
///     roll_finish          MSLA 4.04         50° (~15ml/s) 中速精准划穿
///     total_points         MSLA 5.02         200 (25/55/20) 类人节奏 ~4-5s
/// ═══════════════════════════════════════════════════════════════════════════
struct HeartParams {
    // ── 高度 (MSLA 5.02: 融合 7-10cm, MSLA 4.02: 成形 <1cm) ──
    // 注: 收尾阶段复用 mix_height=80mm (简化设计, MSLA 推荐收尾 5-8cm)
    double mix_height     = 0.08;   // 融合高度 (m), 液面上方 8cm
    double draw_height    = 0.005;  // 成形高度 (m), 液面上方 5mm, 奶缸嘴紧贴液面

    // ── 融合画圈 (心形 = 僧帽白圆 + 中轴划穿, 无需摆动) ──
    // ⚠️ 心形不摆动! wiggle 仅用于 Rosetta 叶形 (MSLA 5.06)
    double mix_circle_r   = 0.010;  // 画圈半径 (m), MSLA 推荐 ~1cm 硬币大小
    double mix_circles    = 2.0;    // 画圈圈数, 2 圈建立清晰白圆, 总路径 ~12.6cm

    // ── XY 运动 (世界坐标系) ──
    // sway_offset_y: 将白圆中心前移 1cm (远离杯沿侧), MSLA 中对应"注入点偏向前方"
    //   为后续划穿留出推进空间 (划穿从 sway_y → sway_y+push_y)
    double push_y         = 0.015;  // 划穿收尾 Y 轴推进距离 (m), 穿过圆心产生心形尖部
    double sway_offset_y  = 0.01;   // Y 轴杯前偏移 (m), 留空间给划穿收尾

    // ── 时序 (阶段分配: 融合 25% + 成形 55% + 收尾 20% = 100%, MSLA 类人节奏) ──
    int    total_points   = 200;    // 总路点数, 三段按比例分配
    double velocity       = 0.5;    // 拉花专用速度缩放 ~4-5s (独立于 lwf_velocity)

    // ── 日志 ──
    bool   verbose        = true;   // 打印阶段详情 (路点数/轨迹长度/倾角)

    // ── Roll 剖面 (绕世界 X 轴绝对倾角, 从水平 0° 直接起算)
    //   MSLA 4.04: 倾角↑ → 重力沿流出方向分量↑ → 流速↑ → 线宽↑
    double roll_mix       = 45.0;   // 融合绝对倾角 (°), ~10ml/s 细流穿透 crema
    double roll_draw      = 60.0;   // 成形绝对倾角 (°) — ⚠️ 兼容别名, 仅 roll_draw_dynamic=false 时使用
    double roll_finish    = 50.0;   // 收尾绝对倾角 (°), ~15ml/s 中速精准划穿

    // ── 成形阶段动态 Roll 渐变 (模拟人类手腕从 60° 逐渐回正至 45° 的自然动作)
    //   梯度方向: 高流量建立白圆基底 → 降低流量防止溢出
    //   roll_draw 仅为兼容别名, 动态模式下实际使用 roll_draw_start/end 这对参数
    //   修改 roll_draw 不会自动同步到 roll_draw_start, 需分别设置喵~
    double roll_draw_start  = 60.0;   // 成形起始倾角 (°), 高流量 ~20ml/s 建立白圆底
    double roll_draw_end    = 45.0;   // 成形结束倾角 (°), 降低流量 ~10ml/s 防止溢出
    bool   roll_draw_dynamic = true;  // true=60°→45° 渐变, false=固定 roll_draw 值
};

class LatteWorkflowNode : public rclcpp::Node
{
public:
    explicit LatteWorkflowNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    bool init();

private:
    // ── 成员 ──
    std::shared_ptr<RobotController> robot_;
    rclcpp::Service<ivg_interfaces::srv::RunLatteWorkflow>::SharedPtr srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recorder_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recorder_stop_cli_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mtx_;

    // ── 工作流参数 (每次 service 回调刷新) ──
    struct Params {
        double approach_h, retract_h;
        double vel, acc;
        int    gripper_pin;
        std::string pattern_type;
        bool execute_latte;
        std::array<double, 6> coffee_joints;
        std::array<double, 6> place_coffee_joints;
        std::array<double, 6> pick_milk_joints;
        std::array<double, 6> nozzle_joints;
        std::array<double, 6> rotate_up_joints;
        HeartParams heart;  // 心形轨迹参数
        double spout_offset_x, spout_offset_y, spout_offset_z;  // TCP→奶缸嘴 (TCP局部坐标)
        double cup_x, cup_y, cup_z;  // 纸杯杯口世界坐标 (来自 URDF paper_cup_Link)
        bool debug_verbose;  // 分段调试详细位姿日志
    };

    void readParameters();
    Params params_;

    // ── Service ──
    void handleRunWorkflow(
        const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
        std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res);

    // ── 工作流步骤 ──
    bool step0_pickCoffee();
    bool step0_placeCoffee();
    bool step1_pickMilk();
    bool step2_approachNozzle();
    bool step3_reorient();
    bool step4_pour();           // 绕世界 X 轴前倾45° — 为 step5 轨迹提供倾倒基准 (MSLA 4.04)
    bool step5_executeLatte();   // 心形三段式笛卡尔轨迹: 委托 LatteTrajectoryGenerator
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_
