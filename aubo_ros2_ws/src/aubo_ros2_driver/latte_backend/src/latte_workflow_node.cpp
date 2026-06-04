#include "latte_backend/latte_workflow_node.h"
#include "latte_backend/latte_trajectory.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <sstream>

namespace latte_backend
{

// ═════════════════════════════════════════════════════════════════════════════
// 构造 & 参数声明
// ═════════════════════════════════════════════════════════════════════════════

LatteWorkflowNode::LatteWorkflowNode(const rclcpp::NodeOptions& options)
    : Node("latte_workflow_node", options)
{
    // ── 取咖啡杯 预教关节角 ──
    if (!has_parameter("lwf_coffee_joints"))
        declare_parameter("lwf_coffee_joints", std::vector<double>{
            0.2719387759609349,    // shoulder_joint
            0.8270516803614146,    // upperArm_joint
            -1.3754182925631984,   // foreArm_joint
            -1.0105609246379685,   // wrist1_joint
            -2.8890522224315975,   // wrist2_joint
            1.2586091933562182     // wrist3_joint
        });

    // ── 放咖啡杯 预教关节角 ──
    if (!has_parameter("lwf_place_coffee_joints"))
        declare_parameter("lwf_place_coffee_joints", std::vector<double>{
            0.27419108480660237,    // shoulder_joint    (15.71°)
            0.6506088935005561,     // upperArm_joint    (37.28°)
            -1.9337206413917687,    // foreArm_joint     (-110.79°)
            -2.5646322590311774,    // wrist1_joint      (-146.94°)
            -1.7729794494848519,    // wrist2_joint      (-101.58°)
            -1.5520265101973545     // wrist3_joint      (-88.92°)
        });

    // ── 取牛奶杯 预教关节角 ──
    if (!has_parameter("lwf_pick_milk_joints"))
        declare_parameter("lwf_pick_milk_joints", std::vector<double>{
            0.5114496604395626,    // shoulder_joint
            0.5560157552322338,    // upperArm_joint
            -1.3872343091070718,   // foreArm_joint
            0.24830893922891084,   // wrist1_joint
            -2.248289855414927,    // wrist2_joint
            0.7857612363757142     // wrist3_joint
        });

    // ── 打奶泡喷嘴终点 预教关节角 (空间直线来回的终点) ──
    if (!has_parameter("lwf_nozzle_joints"))
        declare_parameter("lwf_nozzle_joints", std::vector<double>{
            0.4204691895403951,    // shoulder_joint
            0.3955009389365049,    // upperArm_joint
            -1.428256578941237,    // foreArm_joint
            0.3036545419375402,    // wrist1_joint
            -2.2988592687020133,   // wrist2_joint
            0.6865933110630452     // wrist3_joint
        });

    // ── 转腕朝上中间位姿 (取牛奶后杯口朝上, 不经过大角度反转) ──
    if (!has_parameter("lwf_rotate_up_joints"))
        declare_parameter("lwf_rotate_up_joints", std::vector<double>{
            0.5914579558153719,    // shoulder_joint
            0.5766217599758321,    // upperArm_joint
            -2.079998564794082,    // foreArm_joint
            -2.6342428897545616,   // wrist1_joint
            -2.090051028614271,    // wrist2_joint
            -1.5449341576992022    // wrist3_joint
        });

    // ── 控制参数 ──
    if (!has_parameter("lwf_approach_height")) declare_parameter("lwf_approach_height", 0.15);
    if (!has_parameter("lwf_retract_height"))  declare_parameter("lwf_retract_height", 0.10);
    if (!has_parameter("lwf_velocity"))        declare_parameter("lwf_velocity", 0.5);
    if (!has_parameter("lwf_acceleration"))    declare_parameter("lwf_acceleration", 0.5);
    if (!has_parameter("lwf_gripper_pin"))     declare_parameter("lwf_gripper_pin", 6);
    if (!has_parameter("lwf_pattern_type"))    declare_parameter("lwf_pattern_type", std::string("heart"));
    if (!has_parameter("lwf_execute_latte"))   declare_parameter("lwf_execute_latte", true);

    // ── 心形轨迹参数 (坐标系: 世界X=倾角轴 Y=划穿方向 Z=高度) ──
    if (!has_parameter("lwf_heart_mix_height"))     declare_parameter("lwf_heart_mix_height", 0.08);
    if (!has_parameter("lwf_heart_draw_height"))    declare_parameter("lwf_heart_draw_height", 0.005);
    if (!has_parameter("lwf_heart_mix_circle_r"))   declare_parameter("lwf_heart_mix_circle_r", 0.010);
    if (!has_parameter("lwf_heart_mix_circles"))    declare_parameter("lwf_heart_mix_circles", 2.0);
    // wiggle_amp / wiggle_cycles 已移除: 心形无需摆动, 仅 Rosetta 叶形使用
    if (!has_parameter("lwf_heart_push_y"))         declare_parameter("lwf_heart_push_y", 0.015);
    if (!has_parameter("lwf_heart_total_points"))   declare_parameter("lwf_heart_total_points", 200);
    if (!has_parameter("lwf_heart_sway_offset_y"))  declare_parameter("lwf_heart_sway_offset_y", 0.01);
    if (!has_parameter("lwf_heart_velocity"))       declare_parameter("lwf_heart_velocity", 0.5);
    if (!has_parameter("lwf_heart_verbose"))        declare_parameter("lwf_heart_verbose", true);
    if (!has_parameter("lwf_heart_roll_mix"))       declare_parameter("lwf_heart_roll_mix", 45.0);
    if (!has_parameter("lwf_heart_roll_draw"))      declare_parameter("lwf_heart_roll_draw", 60.0);
    if (!has_parameter("lwf_heart_roll_finish"))    declare_parameter("lwf_heart_roll_finish", 50.0);
    // ── 成形阶段动态 Roll 渐变 (roll_draw_start→roll_draw_end, 模拟杯子回正)
    if (!has_parameter("lwf_heart_roll_draw_start"))   declare_parameter("lwf_heart_roll_draw_start", 60.0);
    if (!has_parameter("lwf_heart_roll_draw_end"))     declare_parameter("lwf_heart_roll_draw_end", 45.0);
    if (!has_parameter("lwf_heart_roll_draw_dynamic")) declare_parameter("lwf_heart_roll_draw_dynamic", true);

    // ── 纸杯位置 + 工具偏移 (2026-06-02 标定) ──
    if (!has_parameter("lwf_cup_x"))              declare_parameter("lwf_cup_x", -0.630);
    if (!has_parameter("lwf_cup_y"))              declare_parameter("lwf_cup_y", -0.308);
    if (!has_parameter("lwf_cup_z"))              declare_parameter("lwf_cup_z", 0.198);
    if (!has_parameter("lwf_spout_offset_x"))     declare_parameter("lwf_spout_offset_x", -0.038);
    if (!has_parameter("lwf_spout_offset_y"))     declare_parameter("lwf_spout_offset_y", 0.095);
    if (!has_parameter("lwf_spout_offset_z"))     declare_parameter("lwf_spout_offset_z", 0.212);
    if (!has_parameter("lwf_debug_verbose"))      declare_parameter("lwf_debug_verbose", true);

    // ── Service 创建 ──
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = create_service<ivg_interfaces::srv::RunLatteWorkflow>(
        "/latte/run_workflow",
        [this](const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
               std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res) {
            handleRunWorkflow(req, res);
        },
        rmw_qos_profile_services_default, cb_group_);

    RCLCPP_INFO(get_logger(), "LatteWorkflowNode 构造完成");
}

bool LatteWorkflowNode::init()
{
    robot_ = std::make_shared<RobotController>(this);
    if (!robot_->init()) {
        RCLCPP_ERROR(get_logger(), "RobotController 初始化失败");
        return false;
    }

    RCLCPP_INFO(get_logger(), "LatteWorkflowNode 就绪");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 参数刷新
// ═════════════════════════════════════════════════════════════════════════════

void LatteWorkflowNode::readParameters()
{
    // 控制参数
    params_.approach_h    = get_parameter("lwf_approach_height").as_double();
    params_.retract_h     = get_parameter("lwf_retract_height").as_double();
    params_.vel           = get_parameter("lwf_velocity").as_double();
    params_.acc           = get_parameter("lwf_acceleration").as_double();
    params_.gripper_pin   = get_parameter("lwf_gripper_pin").as_int();
    params_.pattern_type  = get_parameter("lwf_pattern_type").as_string();
    if (params_.pattern_type != "heart") {
        RCLCPP_WARN(get_logger(),
            "lwf_pattern_type='%s' 暂不支持, 当前仅实现心形(heart)轨迹, 将使用 heart 执行喵~",
            params_.pattern_type.c_str());
    }
    params_.execute_latte = get_parameter("lwf_execute_latte").as_bool();
    // 预教关节角
    auto jv = get_parameter("lwf_coffee_joints").as_double_array();
    if (jv.size() == 6) std::copy_n(jv.begin(), 6, params_.coffee_joints.begin());
    jv = get_parameter("lwf_place_coffee_joints").as_double_array();
    if (jv.size() == 6) std::copy_n(jv.begin(), 6, params_.place_coffee_joints.begin());
    jv = get_parameter("lwf_pick_milk_joints").as_double_array();
    if (jv.size() == 6) std::copy_n(jv.begin(), 6, params_.pick_milk_joints.begin());
    jv = get_parameter("lwf_nozzle_joints").as_double_array();
    if (jv.size() == 6) std::copy_n(jv.begin(), 6, params_.nozzle_joints.begin());
    jv = get_parameter("lwf_rotate_up_joints").as_double_array();
    if (jv.size() == 6) std::copy_n(jv.begin(), 6, params_.rotate_up_joints.begin());

    // ── 心形轨迹参数 ──
    params_.heart.mix_height     = get_parameter("lwf_heart_mix_height").as_double();
    params_.heart.draw_height    = get_parameter("lwf_heart_draw_height").as_double();
    params_.heart.mix_circle_r   = get_parameter("lwf_heart_mix_circle_r").as_double();
    params_.heart.mix_circles    = get_parameter("lwf_heart_mix_circles").as_double();
    params_.heart.push_y         = get_parameter("lwf_heart_push_y").as_double();
    params_.heart.total_points   = get_parameter("lwf_heart_total_points").as_int();
    params_.heart.sway_offset_y  = get_parameter("lwf_heart_sway_offset_y").as_double();
    params_.heart.velocity       = get_parameter("lwf_heart_velocity").as_double();
    params_.heart.verbose        = get_parameter("lwf_heart_verbose").as_bool();
    params_.heart.roll_mix       = get_parameter("lwf_heart_roll_mix").as_double();
    params_.heart.roll_draw      = get_parameter("lwf_heart_roll_draw").as_double();
    params_.heart.roll_finish    = get_parameter("lwf_heart_roll_finish").as_double();
    params_.heart.roll_draw_start  = get_parameter("lwf_heart_roll_draw_start").as_double();
    params_.heart.roll_draw_end    = get_parameter("lwf_heart_roll_draw_end").as_double();
    params_.heart.roll_draw_dynamic = get_parameter("lwf_heart_roll_draw_dynamic").as_bool();

    // 纸杯位置 + 工具偏移
    params_.spout_offset_x = get_parameter("lwf_spout_offset_x").as_double();
    params_.spout_offset_y = get_parameter("lwf_spout_offset_y").as_double();
    params_.spout_offset_z = get_parameter("lwf_spout_offset_z").as_double();
    params_.cup_x = get_parameter("lwf_cup_x").as_double();
    params_.cup_y = get_parameter("lwf_cup_y").as_double();
    params_.cup_z = get_parameter("lwf_cup_z").as_double();
    params_.debug_verbose = get_parameter("lwf_debug_verbose").as_bool();
}

// ═════════════════════════════════════════════════════════════════════════════
// Service 回调 — 5 步咖啡拉花工作流 (step0 保留未启用)
//   [1/5] 取牛奶杯 → [2/5] 打奶泡 → [3/5] 转腕朝上→放置咖啡杯
//   → [4/5] 嘴口绕世界X轴前倾45° (倾倒基准) → [5/5] 心形拉花执行
//   坐标系: 世界X=倾角轴  世界Y=划穿方向  世界Z=高度
//   step4 提供 45° 基准倾角, step5 各阶段 Roll 绝对倾角: 融合45° 成形60°→45° 收尾50°
// ═════════════════════════════════════════════════════════════════════════════

void LatteWorkflowNode::handleRunWorkflow(
    const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> /*req*/,
    std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res)
{
    std::lock_guard<std::mutex> lock(mtx_);
    readParameters();

    RCLCPP_INFO(get_logger(), "========== 咖啡拉花工作流 开始 (共5步) ==========");

    // ── step0 保留未启用 (多杯方案时取消注释): step0_pickCoffee / step0_placeCoffee ──
    if (!step1_pickMilk())       { res->success = false; res->message = "[步骤1/5] 失败: 取牛奶杯";      return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS 已关闭"; return; }
    if (!step2_approachNozzle()) { res->success = false; res->message = "[步骤2/5] 失败: 打奶泡";        return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS 已关闭"; return; }
    if (!step3_reorient())       { res->success = false; res->message = "[步骤3/5] 失败: 转腕朝上";      return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS 已关闭"; return; }
    // step4: X轴前倾45° — 从水平状态倾斜到倒奶起始角, 为 step5 轨迹 roll 剖面提供正确基准
    // if (!step4_pour())           { res->success = false; res->message = "[步骤4/5] 失败: 嘴口倾倒";      return; }
    // if (!rclcpp::ok())           { res->success = false; res->message = "ROS 已关闭"; return; }
    if (!step5_executeLatte())   { res->success = false; res->message = "[步骤5/5] 失败: 拉花执行";      return; }

    res->success = true;
    res->message = "全部步骤完成";
    RCLCPP_INFO(get_logger(), "========== 咖啡拉花工作流 完成 ==========");
}

// ═════════════════════════════════════════════════════════════════════════════
// step0: 取咖啡杯 — 关节运动到预教位姿 → 抓取 → Z 退避
// ⚠️ 保留未启用: 多杯方案时取消注释
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step0_pickCoffee()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    RCLCPP_INFO(get_logger(), "[保留-取咖啡杯] 关节=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                params_.coffee_joints[0], params_.coffee_joints[1], params_.coffee_joints[2],
                params_.coffee_joints[3], params_.coffee_joints[4], params_.coffee_joints[5]);

    if (!robot_->moveToJoints(params_.coffee_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[保留-取咖啡杯] 关节运动失败"); return false;
    }
    if (!robot_->setGripper(params_.gripper_pin, false)) {
        RCLCPP_WARN(get_logger(), "[保留-取咖啡杯] 夹爪失败(仿真?), 继续");
    }
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[保留-取咖啡杯] Z轴退避失败"); return false;
    }
    RCLCPP_INFO(get_logger(), "[保留-取咖啡杯] 完成");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// step0: 放咖啡杯 — 关节运动到预教位姿 → 释放 → Z 退避
// ⚠️ 保留未启用: 多杯方案时取消注释
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step0_placeCoffee()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    RCLCPP_INFO(get_logger(), "[保留-放咖啡杯] 关节=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                params_.place_coffee_joints[0], params_.place_coffee_joints[1], params_.place_coffee_joints[2],
                params_.place_coffee_joints[3], params_.place_coffee_joints[4], params_.place_coffee_joints[5]);

    if (!robot_->moveToJoints(params_.place_coffee_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[保留-放咖啡杯] 关节运动失败"); return false;
    }
    if (!robot_->setGripper(params_.gripper_pin, true)) {
        RCLCPP_WARN(get_logger(), "[保留-放咖啡杯] 释放失败(仿真?), 继续");
    }
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[保留-放咖啡杯] Z轴退避失败"); return false;
    }
    RCLCPP_INFO(get_logger(), "[保留-放咖啡杯] 完成");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 步骤1/5: 取牛奶杯 — 关节运动到预教位姿 → 抓取 (不退避)
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step1_pickMilk()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    RCLCPP_INFO(get_logger(), "[步骤1/5] 取牛奶杯  关节=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                params_.pick_milk_joints[0], params_.pick_milk_joints[1], params_.pick_milk_joints[2],
                params_.pick_milk_joints[3], params_.pick_milk_joints[4], params_.pick_milk_joints[5]);

    if (!robot_->moveToJoints(params_.pick_milk_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[步骤1/5] 关节运动失败"); return false;
    }
    if (!robot_->setGripper(params_.gripper_pin, false)) {
        RCLCPP_WARN(get_logger(), "[步骤1/5] 夹爪失败(仿真?), 继续");
    }
    RCLCPP_INFO(get_logger(), "[步骤1/5] 完成");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 步骤2/5: 打奶泡 — 从取牛奶位姿出发, 笛卡尔直线去喷嘴 → 打奶泡 → 笛卡尔直线回 → Z退避
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step2_approachNozzle()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    auto start_pose = robot_->getCurrentPose();
    auto nozzle_pose = robot_->jointsToPose(params_.nozzle_joints);
    RCLCPP_INFO(get_logger(), "[步骤2/5] 打奶泡  起点(%.3f %.3f %.3f) -> 喷嘴(%.3f %.3f %.3f)",
                start_pose.position.x, start_pose.position.y, start_pose.position.z,
                nozzle_pose.position.x, nozzle_pose.position.y, nozzle_pose.position.z);

    // 笛卡尔直线去 (步级重试 3 次)
    bool ok = false;
    for (int r = 0; r < 3; ++r) {
        if (robot_->moveCartesianStraight(nozzle_pose, vel, acc)) { ok = true; break; }
        RCLCPP_WARN(get_logger(), "[步骤2/5] 去喷嘴 第%d次失败, 重试...", r + 1);
    }
    if (!ok) { RCLCPP_ERROR(get_logger(), "[步骤2/5] 去喷嘴直线失败(已重试3次)"); return false; }
    // 打奶泡等待
    RCLCPP_INFO(get_logger(), "[步骤2/5] 打奶泡中 2s...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 笛卡尔直线回取牛奶位姿 (步级重试 3 次)
    ok = false;
    for (int r = 0; r < 3; ++r) {
        if (robot_->moveCartesianStraight(start_pose, vel, acc)) { ok = true; break; }
        RCLCPP_WARN(get_logger(), "[步骤2/5] 返回起点 第%d次失败, 重试...", r + 1);
    }
    if (!ok) { RCLCPP_ERROR(get_logger(), "[步骤2/5] 返回起点直线失败(已重试3次)"); return false; }
    // Z 退避 — 为后续大跨度运动提供安全高度
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "[步骤2/5] Z轴退避失败"); return false;
    }
    RCLCPP_INFO(get_logger(), "[步骤2/5] 完成");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 步骤3/5: 转腕朝上 → 笛卡尔平移至放置咖啡杯位置
//   3a: 笛卡尔 slerp 到转腕朝上位姿 (防止杯子倾翻)
//   3b: 笛卡尔平移至放置咖啡杯位置 (FK place_coffee_joints)
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step3_reorient()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 3a — 笛卡尔 slerp: 从当前位姿到转腕朝上位姿, 保证最短旋转路径
    auto target_pose = robot_->jointsToPose(params_.rotate_up_joints);
    RCLCPP_INFO(get_logger(), "[步骤3/5] 3a 转腕朝上 笛卡尔 -> (%.3f %.3f %.3f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    bool ok = false;
    for (int r = 0; r < 3; ++r) {
        if (robot_->moveCartesianStraight(target_pose, vel, acc)) { ok = true; break; }
        RCLCPP_WARN(get_logger(), "[步骤3/5] 3a 第%d次失败, 重试...", r + 1);
    }
    if (!ok) { RCLCPP_ERROR(get_logger(), "[步骤3/5] 3a 笛卡尔直线失败(已重试3次)"); return false; }

    // 3b — 笛卡尔平移: 到放置咖啡杯位置, slerp 保证最短旋转路径
    auto place_pose = robot_->jointsToPose(params_.place_coffee_joints);
    RCLCPP_INFO(get_logger(), "[步骤3/5] 3b 放置咖啡杯  笛卡尔 -> (%.3f %.3f %.3f)",
                place_pose.position.x, place_pose.position.y, place_pose.position.z);
    ok = false;
    for (int r = 0; r < 3; ++r) {
        if (robot_->moveCartesianStraight(place_pose, vel, acc)) { ok = true; break; }
        RCLCPP_WARN(get_logger(), "[步骤3/5] 3b 第%d次失败, 重试...", r + 1);
    }
    if (!ok) { RCLCPP_ERROR(get_logger(), "[步骤3/5] 3b 笛卡尔直线失败(已重试3次)"); return false; }

    RCLCPP_INFO(get_logger(), "[步骤3/5] 完成");
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 心形轨迹辅助函数: 四元数运算 — 必须在 step4/step5 之前定义
// ═════════════════════════════════════════════════════════════════════════════

namespace {

/// 绕 X 轴旋转 angle_deg 度的四元数
geometry_msgs::msg::Quaternion quat_from_x_rotation(double angle_deg)
{
    double half = angle_deg * M_PI / 360.0;
    geometry_msgs::msg::Quaternion q;
    q.x = std::sin(half);
    q.y = 0.0;
    q.z = 0.0;
    q.w = std::cos(half);
    return q;
}

/// 四元数 Hamilton 乘法: q1 * q0 (先 q0 后 q1)
geometry_msgs::msg::Quaternion quat_mul(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q0)
{
    geometry_msgs::msg::Quaternion r;
    r.w = q1.w * q0.w - q1.x * q0.x - q1.y * q0.y - q1.z * q0.z;
    r.x = q1.w * q0.x + q1.x * q0.w + q1.y * q0.z - q1.z * q0.y;
    r.y = q1.w * q0.y - q1.x * q0.z + q1.y * q0.w + q1.z * q0.x;
    r.z = q1.w * q0.z + q1.x * q0.y - q1.y * q0.x + q1.z * q0.w;
    return r;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// 调试辅助: 四元数 → RPY(度) + 位姿格式化
// ═════════════════════════════════════════════════════════════════════════════

std::string quatToRPYStr(const geometry_msgs::msg::Quaternion& q)
{
    double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr, cosr);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch;
    if (std::fabs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny, cosy);

    char buf[64];
    snprintf(buf, sizeof(buf), "RPY(%.1f° %.1f° %.1f°)",
             roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    return std::string(buf);
}

std::string poseToStr(const geometry_msgs::msg::Pose& p)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "xyz(%.4f %.4f %.4f) %s",
             p.position.x, p.position.y, p.position.z,
             quatToRPYStr(p.orientation).c_str());
    return std::string(buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// step4: 嘴口倾倒 — 绕世界 X 轴前倾 45°, 为 step5 轨迹 roll 剖面提供倾倒基准喵~
//
// MSLA 依据: MSLA 4.04 (倾角-流量关系) — 45° 对应 ~10ml/s 细流
//            MSLA 4.02 (Pin-Drop Technique) — 细流穿透 crema 沉底, 不扰动表面
//
// 为何选 45° (而非 30° 或 60°):
//   45° 是"最小有效倾角" — 恰好足够让牛奶克服表面张力流出奶缸嘴,
//   同时保持低流量 (~10ml/s), 适合高位 Pin-Drop 融合阶段。
//   30°: 流量太低, 牛奶可能滴落而非连续流束
//   60°: 流量太高 (~20ml/s), 高位注入会飞溅破坏 crema
//
// 数学操作:
//   quat_mul(q_rot, current_ori) = 全局坐标系左乘
//   即: 绕 base_link 的 X 轴旋转 TCP, 非 TCP body-fixed 旋转
//   效果: pitch 从 ~89° (水平) → ~44° (前倾 45°), 奶缸嘴指向下方
//
// step4 之后: TCP 位姿中 orientation 已包含 45° X 轴旋转
//   step5 中 makeStagePose/generateHeartStageWaypoints 通过
//   rel_roll = roll_deg - 45° 计算增量, 防止双叠旋转
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step4_pour()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 绕全局 X 轴前倾 45° (左乘 = 全局旋转, pitch从89°→46°即奶缸前倾)
    auto pour_pose = robot_->getCurrentPose();
    auto q_rot = quat_from_x_rotation(45.0);
    pour_pose.orientation = quat_mul(q_rot, pour_pose.orientation);

    RCLCPP_INFO(get_logger(), "[步骤4/5] 嘴口倾倒  X轴旋转45° (%.3f %.3f %.3f)",
                pour_pose.position.x, pour_pose.position.y, pour_pose.position.z);
    bool ok = false;
    for (int r = 0; r < 3; ++r) {
        if (robot_->moveCartesianStraight(pour_pose, vel, acc)) { ok = true; break; }
        RCLCPP_WARN(get_logger(), "[步骤4/5] 第%d次失败, 重试...", r + 1);
    }
    if (!ok) { RCLCPP_ERROR(get_logger(), "[步骤4/5] 笛卡尔直线失败(已重试3次)"); return false; }
    RCLCPP_INFO(get_logger(), "[步骤4/5] 完成");
    return true;
}


// ═════════════════════════════════════════════════════════════════════════════
// step5: 心形拉花 (Heart Latte Art) — 三段式笛卡尔轨迹执行
//
// 架构总览:
//
//   step4 (45° 基准) ─────┐
//                         ▼
//   spout_origin = cup_mouth (纸杯杯口)  ◄── 奶缸嘴坐标系
//   tool_world  = rotate(spout_offset, tcp_ori)
//   tcp_pose    = spout_pose - tool_world  (发送 MoveIt 前转换)
//       │
//       ├── [5a] spout@Z=cup_z+80mm roll=45° → 画 r=10mm 圆 ×2 圈 (25%)
//       │       高位 Pin-Drop, ~10ml/s 细流穿透 crema 建立白圆基底
//       │
//       ├── [5b] spout@Z=cup_z+5mm roll=60°→45° → 壶嘴定点注入 (55%)
//       │       低位, ~20→10ml/s, 奶泡浮面自然扩散白圆
//       │
//       └── [5c] spout@Z=cup_z+80mm roll=50° → Y轴推进 15mm (20%)
//               ~15ml/s 中速细流划穿圆心产生心形尖部
//
// 关键设计决策:
//   1. 阶段间过渡用 moveCartesianStraight (slerp 保证姿态平滑过渡),
//      阶段内部连续运动用 executeCartesianPath (waypoints 离散点)
//   2. 所有 roll 为绝对倾角, step4 已禁用, 从水平(0°)直接起算
//   3. 阶段编号跳过 3: 预留用于 Heart-in-Heart / Tulip 第二层花瓣
//
// MSLA 权威依据:
//   [5a] 融合: MSLA 5.02 + 4.02 (Pin-Drop) + 4.04 (倾角-流量)
//   [5b] 成形: MSLA 5.04 + 4.04 (线宽∝√流量) + B1 5.04-5.05 (壶嘴定点)
//   [5c] 收尾: MSLA 5.02-5.05 (划穿) + "The Off Switch"
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step5_executeLatte()
{
    if (!params_.execute_latte) {
        RCLCPP_INFO(get_logger(), "[步骤5/5] 跳过 (lwf_execute_latte=false)");
        return true;
    }

    const float vel = static_cast<float>(params_.heart.velocity);
    const float acc = static_cast<float>(params_.acc);

    // tool_tcp 末端坐标系 — 不切换 spout_marker (MoveIt IK 可能不处理固定偏移)
    // 轨迹在 spout 坐标生成, 通过 gen.spoutToTcp() 转为 tool_tcp 坐标后执行

    LatteTrajectoryParams tp;
    tp.cup_x = params_.cup_x; tp.cup_y = params_.cup_y; tp.cup_z = params_.cup_z;
    tp.spout_offset_x = params_.spout_offset_x;
    tp.spout_offset_y = params_.spout_offset_y;
    tp.spout_offset_z = params_.spout_offset_z;
    tp.heart = params_.heart;
    tp.tcp_orientation = robot_->getCurrentPose().orientation;

    LatteTrajectoryGenerator gen(tp);

    RCLCPP_INFO(get_logger(),
        "[步骤5] ┌─ 杯口(%.4f,%.4f,%.4f)  spout偏移(%.3f,%.3f,%.3f)  eef=tool_tcp",
        tp.cup_x, tp.cup_y, tp.cup_z,
        tp.spout_offset_x, tp.spout_offset_y, tp.spout_offset_z);
    auto cur = robot_->getCurrentPose();
    RCLCPP_INFO(get_logger(),
        "[步骤5] └─ 当前TCP位姿: %s", poseToStr(cur).c_str());

    auto exec_stage = [&](const StagePlan& sp) -> bool {
        // spout → TCP 坐标转换 (MoveIt 用 tool_tcp EE 执行)
        StagePlan tcp = gen.spoutToTcp(sp);
        char label = "0abcd"[std::min(tcp.stage_id, 4)];
        auto t0 = std::chrono::steady_clock::now();

        if (params_.debug_verbose) {
            auto cp = robot_->getCurrentPose();
            RCLCPP_INFO(get_logger(), "[5%c] ┌────────────────────────────────────────", label);
            RCLCPP_INFO(get_logger(), "[5%c] │ %s START", label, tcp.name);
            RCLCPP_INFO(get_logger(), "[5%c] │   当前TCP: %s", label, poseToStr(cp).c_str());
            RCLCPP_INFO(get_logger(), "[5%c] │   过渡TCP: %s", label, poseToStr(tcp.transition_target).c_str());
        }

        if (params_.debug_verbose && !tcp.waypoints.empty()) {
            size_t n = tcp.waypoints.size();
            RCLCPP_INFO(get_logger(), "[5%c] │   路点[0/%zu]: %s", label, n-1, poseToStr(tcp.waypoints.front()).c_str());
            if (n > 2)
                RCLCPP_INFO(get_logger(), "[5%c] │   路点[%zu/%zu]: %s", label, n/2, n-1, poseToStr(tcp.waypoints[n/2]).c_str());
            RCLCPP_INFO(get_logger(), "[5%c] │   路点[%zu/%zu]: %s", label, n-1, n-1, poseToStr(tcp.waypoints.back()).c_str());
        }

        RCLCPP_INFO(get_logger(), "[5%c] │   moveCartesianStraight → 过渡...", label);
        if (!robot_->moveCartesianStraight(tcp.transition_target, vel, acc)) {
            RCLCPP_ERROR(get_logger(), "[5%c] └─ ✗ 过渡失败", label);
            return false;
        }
        if (!tcp.waypoints.empty()) {
            RCLCPP_INFO(get_logger(), "[5%c] │   executeCartesianPath → %zu点...", label, tcp.waypoints.size());
            if (!robot_->executeCartesianPath(tcp.waypoints, vel, acc)) {
                RCLCPP_ERROR(get_logger(), "[5%c] └─ ✗ 轨迹执行失败", label);
                return false;
            }
        }

        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        if (params_.debug_verbose) {
            auto ap = robot_->getCurrentPose();
            RCLCPP_INFO(get_logger(), "[5%c] │   到达TCP: %s", label, poseToStr(ap).c_str());
        }
        RCLCPP_INFO(get_logger(), "[5%c] └─ ✓ %s 完成 (%ldms)", label, tcp.name, ms);
        return true;
    };

    bool ok = exec_stage(gen.stageApproach())
           && rclcpp::ok()
           && exec_stage(gen.stageMix())
           && rclcpp::ok()
           && exec_stage(gen.stageDraw())
           && rclcpp::ok()
           && exec_stage(gen.stageFinish())
           && rclcpp::ok()
           && exec_stage(gen.stageHome());


    if (ok) RCLCPP_INFO(get_logger(), "[步骤5/5] ✓ 全部完成 (3段)");
    else    RCLCPP_ERROR(get_logger(), "[步骤5/5] ✗ 失败");
    return ok;
}

}  // namespace latte_backend

// ═════════════════════════════════════════════════════════════════════════════
// main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<latte_backend::LatteWorkflowNode>(options);
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "LatteWorkflowNode 初始化失败");
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
