#ifndef LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_
#define LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "demo_driver/robot_controller.h"
#include <ivg_interfaces/srv/run_latte_workflow.hpp>

namespace latte_backend
{

using demo_driver::RobotController;

/// ═══════════════════════════════════════════════════════════════════════════
/// 心形轨迹生成参数 (基于 Barista Hustle MSLA / SCA WLAC 权威资料) 喵~
///
/// 坐标系约定: 代码中所有 "roll" 指 **绕世界坐标系 X 轴的倾角** (非 TCP body-fixed roll)
///
///     世界 X 轴 (前) → 倾倒倾角轴: 绕此轴旋转使奶缸前倾(+)/后仰(-)
///     世界 Y 轴 (左) → 划穿方向轴: 奶缸嘴沿此方向倒奶, cut-through 沿此方向
///     世界 Z 轴 (上) → 高度轴: 液面上方距离
///
///     step4 提供 45° 基准倾角 (绕 X 轴), step5 各阶段在此基准上叠加增量
///     实际绝对倾角 = step4基准(45°) + rel_roll(roll_deg - 45°) = roll_deg
///
///   倾角-流量关系: 倾角越大→重力沿流出方向分量越大→流速越高
///     融合 45° → ~10ml/s 细流, 高位穿透 crema 沉底
///     成形 60° → ~20ml/s 加速, 低位泡沫浮面扩散白圆
///     收尾 50° → 中速细流, 抬高划穿产生心形尖部
/// ═══════════════════════════════════════════════════════════════════════════
struct HeartParams {
    // ── 高度 (Barista Hustle MSLA 5.02: mix ~10cm, draw <1cm, finish 5-8cm) ──
    double mix_height     = 0.08;   // 融合: 液面上方 8cm (权威推荐 7-10cm)
    double draw_height    = 0.005;  // 成形: 液面上方 5mm (权威推荐 <1cm)

    // ── 融合画圈 (心形 = 僧帽白圆 + 中轴划穿, NO WIGGLE) ──
    double mix_circle_r   = 0.010;  // 融合画圈半径 10mm (权威推荐 ~1cm 硬币大小)
    double mix_circles    = 2.0;    // 融合画圈圈数 (半径翻倍后减圈补偿, 总路径 ~12.6cm)
    // ⚠️ 心形无需摆动! 摆动仅用于 Rosetta 叶形

    // ── XY 运动 (世界坐标系) ──
    double push_y         = 0.015;  // 划穿收尾沿世界Y轴推进距离 1.5cm
    double sway_offset_y  = 0.01;   // 世界Y轴杯前偏移 1cm (留空间给划穿收尾)

    // ── 时序 (阶段分配: 融合25% + 成形55% + 收尾20% = 100%) ──
    int    total_points   = 200;    // 总路点数
    double velocity       = 0.5;    // 拉花专用速度缩放 ~4-5s 类人节奏 (独立于 lwf_velocity)

    // ── 日志 ──
    bool   verbose        = true;   // 打印阶段详情

    // ── Roll 动态剖面 (绕世界X轴绝对倾角, step4已提供45°基准, 代码自动减45°算增量)
    //   Barista Hustle MSLA 4.04: 流量+100%→线宽+42%
    //   融合 45° (10ml/s 细流, 高位穿透crema沉底)
    //   成形 60°→45° 渐变 (20ml/s 加速扩散白圆, 逐渐回正)
    //   收尾 50° (中速细流, 抬高划穿)
    double roll_mix       = 45.0;   // 融合绝对倾角 (°)
    double roll_draw      = 60.0;   // 成形绝对倾角 (°) — 兼容别名, 等于 roll_draw_start
    double roll_finish    = 50.0;   // 收尾绝对倾角 (°)

    // ── 成形阶段动态 Roll 渐变 (模拟人类"45°逐渐减小"的杯子回正) ──
    double roll_draw_start  = 60.0;   // 成形起始倾角 (°), 高流量建立白圆底
    double roll_draw_end    = 45.0;   // 成形结束倾角 (°), 降低流量防止溢出
    bool   roll_draw_dynamic = true;  // true=渐变, false=固定 roll_draw
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
    bool step4_pour();
    bool step5_executeLatte();

    // ── 心形轨迹分段生成 (每段固定 Z, XY 运动; 成形阶段可选动态 Roll)
    //   stage: 1=融合画圈(固定Roll)  2=成形注入(支持动态Roll渐变)  4=划穿收尾(固定Roll)
    //   坐标: origin=tool_tcp 在 base_link 下的位姿, 所有运动在世界坐标系中
    std::vector<geometry_msgs::msg::Pose> generateHeartStageWaypoints(
        const geometry_msgs::msg::Pose& origin,
        const HeartParams& hp,
        double fixed_z,          // 世界Z偏移 (液面上方高度)
        double fixed_roll_deg,   // 绕世界X轴绝对倾角 (stage2动态时忽略)
        int stage);

    // 构建目标姿态 — 在 origin 位姿基础上叠加 Z 偏移 + 绕世界 X 轴旋转 + Y 偏移
    geometry_msgs::msg::Pose makeStagePose(
        const geometry_msgs::msg::Pose& origin,
        double z_offset,
        double roll_deg,
        double sway_y_offset);
};

}  // namespace latte_backend

#endif  // LATTE_BACKEND_LATTE_WORKFLOW_NODE_H_
