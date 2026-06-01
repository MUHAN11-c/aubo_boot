#include "latte_backend/latte_workflow_node.h"
#include <algorithm>
#include <array>
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

    recorder_start_cli_ = create_client<std_srvs::srv::Trigger>(
        "/trajectory_recorder/start_latte_record");
    recorder_stop_cli_ = create_client<std_srvs::srv::Trigger>(
        "/trajectory_recorder/stop_latte_record");

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
    if (!step4_pour())           { res->success = false; res->message = "[步骤4/5] 失败: 嘴口倾倒";      return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS 已关闭"; return; }
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
// step4: 嘴口倾倒 — 绕世界 X 轴前倾 45°, 为 step5 轨迹 roll 剖面提供倾倒基准
//   世界 X 轴 = 倾倒倾角轴, 左乘 q_rot_x * q_current (全局旋转)
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
// makeStagePose: 构建目标姿态 (moveCartesianStraight 过渡用)
//   在 origin 基础上叠加: Z 偏移 (世界Z) + 绕世界X轴旋转 + Y 偏移 (世界Y)
//   roll_deg 为绝对倾角, 自动减去 step4 已提供的 45° 基准避免双叠
//   rel_roll = roll_deg - 45°  (融合0° 成形起始+15° 成形结束0° 收尾+5°)
// ═════════════════════════════════════════════════════════════════════════════

geometry_msgs::msg::Pose LatteWorkflowNode::makeStagePose(
    const geometry_msgs::msg::Pose& origin,
    double z_offset, double roll_deg, double sway_y_offset)
{
    geometry_msgs::msg::Pose p;
    p.position.x = origin.position.x;
    p.position.y = origin.position.y + sway_y_offset;
    p.position.z = origin.position.z + z_offset;

    double rel_roll = roll_deg - 45.0;  // step4 X轴45° = baseline
    auto q_roll = quat_from_x_rotation(rel_roll);
    p.orientation = quat_mul(q_roll, origin.orientation);
    return p;
}

// ═════════════════════════════════════════════════════════════════════════════
// generateHeartStageWaypoints: 生成单阶段 waypoints (世界坐标系)
//   stage=1 融合画圈 (固定 Roll)  stage=2 成形定点注入 (支持动态 Roll 渐变)
//   stage=4 划穿收尾 (固定 Roll)
//   阶段分配: 25% + 55% + 20% = 100% (基于 Barista Hustle MSLA 推荐)
// ═════════════════════════════════════════════════════════════════════════════

std::vector<geometry_msgs::msg::Pose> LatteWorkflowNode::generateHeartStageWaypoints(
    const geometry_msgs::msg::Pose& origin,
    const HeartParams& hp,
    double fixed_z,
    double fixed_roll_deg,
    int stage)
{
    const double origin_x = origin.position.x;
    const double sway_y   = origin.position.y + hp.sway_offset_y;

    // 统一朝向: 减去 step4 已提供的 45° 基准
    // 成形阶段若启用动态 Roll, 逐点在循环内计算 (非预计算)
    bool dynamic = (stage == 2 && hp.roll_draw_dynamic);
    geometry_msgs::msg::Quaternion fixed_ori;
    if (!dynamic) {
        if (stage == 2 && hp.verbose) {
            RCLCPP_WARN(rclcpp::get_logger("latte_workflow_node"),
                "阶段2-成形注入: roll_draw_dynamic=false, 所有 waypoints 位置+朝向完全相同, "
                "executeCartesianPath 将产生零长度轨迹, 成形阶段不会产生任何运动喵~");
        }
        double rel_roll = fixed_roll_deg - 45.0;
        auto q_roll = quat_from_x_rotation(rel_roll);
        fixed_ori = quat_mul(q_roll, origin.orientation);
    }

    int N = 0;
    if (stage == 1)      N = static_cast<int>(hp.total_points * 0.25);  // 融合 25%
    else if (stage == 2) N = static_cast<int>(hp.total_points * 0.55);  // 成形 55%
    else                 N = static_cast<int>(hp.total_points * 0.20);  // 收尾 20%
    if (N < 1) N = 1;

    auto calc_progress = [](int i, int n) -> double {
        return n > 1 ? static_cast<double>(i) / (n - 1) : 0.0;
    };

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(N);

    double total_len = 0.0;
    double prev_x = origin_x, prev_y = sway_y, prev_z = fixed_z;

    for (int i = 0; i < N; ++i) {
        geometry_msgs::msg::Pose p;
        double progress = calc_progress(i, N);

        if (stage == 1) {
            // 融合画圈 — r=10mm, 2圈 (Barista Hustle MSLA: ~1cm 硬币大小)
            double angle = 2.0 * M_PI * hp.mix_circles * progress;
            p.position.x = origin_x + hp.mix_circle_r * std::cos(angle);
            p.position.y = sway_y + hp.mix_circle_r * std::sin(angle);
        } else if (stage == 2) {
            // 成形定点注入 — 心形 = 僧帽白圆 + 中轴划穿, NO WIGGLE
            // Barista Hustle B1 5.04-5.05: 壶嘴杯中心固定不动
            // 白色圆形由流速加速 (倾角60°→45°渐变) 自然扩散, 非奶缸移动
            p.position.x = origin_x;   // X 中心固定
            p.position.y = sway_y;     // Y 固定不动, 圆形靠流速扩散
        } else {
            // 划穿收尾 — 中轴穿透 (<1s thin stream cut-through)
            // Barista Hustle: 沿世界Y轴穿过圆心, 产生心形尖部
            p.position.x = origin_x;
            p.position.y = sway_y + progress * hp.push_y;  // sway_y → sway_y+push_y
        }

        p.position.z = fixed_z;

        if (!dynamic) {
            p.orientation = fixed_ori;
        } else {
            // 动态 Roll 渐变: roll_draw_start → roll_draw_end 线性插值
            double dy_roll = hp.roll_draw_start + progress * (hp.roll_draw_end - hp.roll_draw_start);
            double rel_dyn = dy_roll - 45.0;
            auto q_dyn = quat_from_x_rotation(rel_dyn);
            p.orientation = quat_mul(q_dyn, origin.orientation);
        }

        double dx = p.position.x - prev_x;
        double dy = p.position.y - prev_y;
        double dz = p.position.z - prev_z;
        total_len += std::sqrt(dx*dx + dy*dy + dz*dz);
        prev_x = p.position.x; prev_y = p.position.y; prev_z = p.position.z;

        waypoints.push_back(p);
    }

    if (hp.verbose) {
        const char* name = (stage == 1) ? "阶段1-融合画圈" :
                           (stage == 2) ? "阶段2-成形注入" : "阶段4-划穿收尾";
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Z=%.0fmm roll=%.0f° N=%d 总长=%.1fmm",
            fixed_z * 1000, fixed_roll_deg, N, total_len * 1000);
        if (stage == 1) {
            std::snprintf(buf, sizeof(buf),
                "%s  r=%.0fmm ×%.1f圈  总长=%.1fmm",
                name, hp.mix_circle_r * 1000, hp.mix_circles, total_len * 1000);
        } else if (stage == 2) {
            if (dynamic) {
                std::snprintf(buf, sizeof(buf),
                    "%s  定点注入 roll=%.0f°→%.0f°  总长=%.1fmm",
                    name, hp.roll_draw_start, hp.roll_draw_end, total_len * 1000);
            } else {
                std::snprintf(buf, sizeof(buf),
                    "%s  定点注入 roll=%.0f°  总长=%.1fmm",
                    name, fixed_roll_deg, total_len * 1000);
            }
        } else {
            std::snprintf(buf, sizeof(buf),
                "%s  Y_push=%.1fmm  总长=%.1fmm",
                name, hp.push_y * 1000, total_len * 1000);
        }
        RCLCPP_INFO(get_logger(), "%s", buf);
    }

    return waypoints;
}

// ═════════════════════════════════════════════════════════════════════════════
// step5: 心形拉花 — 分段执行 (moveCartesianStraight 过渡 + executeCartesianPath 执行)
//   [5a] 移动到融合位姿 (Z=mix, roll=45°)          → 画圈 r=10mm ×2圈
//   [5b] 移动到成形位姿 (Z=draw, roll=60°→45°渐变)  → 定点注入
//   [5c] 移动到收尾位姿 (Z=mix, roll=50°)            → 中轴划穿 15mm
//   坐标系: 世界X=倾角轴 Y=划穿方向 Z=高度
//   姿态过渡统一用 moveCartesianStraight (slerp保证姿态精度),
//   XY 运动用 executeCartesianPath (waypoints 内姿态恒定或渐变, IK 稳定)
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step5_executeLatte()
{
    if (!params_.execute_latte) {
        RCLCPP_INFO(get_logger(), "[步骤5/5] 跳过 (lwf_execute_latte=false)");
        return true;
    }

    const float vel = static_cast<float>(params_.heart.velocity);
    const float acc = static_cast<float>(params_.acc);
    const auto& hp = params_.heart;

    // 通知 trajectory_recorder 开始拉花录制
    if (recorder_start_cli_->wait_for_service(std::chrono::milliseconds(50))) {
        recorder_start_cli_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        RCLCPP_INFO(get_logger(), "[步骤5/5] 已通知 trajectory_recorder 开始录制");
    }

    auto origin = robot_->getCurrentPose();
    RCLCPP_INFO(get_logger(),
        "[步骤5/5] 心形轨迹分段执行 原点=(%.3f, %.3f, %.3f) vel=%.3f",
        origin.position.x, origin.position.y, origin.position.z, vel);

    bool ok = true;

    // ── [5a] 阶段1: 融合画圈 (Z=mix=80mm, roll=mix=45°) ──
    {
        auto pose_1 = makeStagePose(origin, hp.mix_height, hp.roll_mix, hp.sway_offset_y);
        RCLCPP_INFO(get_logger(), "[5a] 移动到融合位姿 Z=%.0fmm roll=%.0f°",
                    hp.mix_height * 1000, hp.roll_mix);
        if (!robot_->moveCartesianStraight(pose_1, vel, acc)) {
            RCLCPP_ERROR(get_logger(), "[5a] moveCartesianStraight 失败");
            ok = false;
        }
        if (ok) {
            auto wps = generateHeartStageWaypoints(origin, hp,
                origin.position.z + hp.mix_height, hp.roll_mix, 1);
            RCLCPP_INFO(get_logger(), "[5a] 画圈 %zu 点", wps.size());
            if (!robot_->executeCartesianPath(wps, vel, acc)) {
                RCLCPP_ERROR(get_logger(), "[5a] executeCartesianPath 失败");
                ok = false;
            }
        }
    }

    // ── [5b] 阶段2: 成形注入 (Z=draw=5mm, roll=60°→45° 渐变) ──
    if (ok && rclcpp::ok()) {
        auto pose_2 = makeStagePose(origin, hp.draw_height, hp.roll_draw_start, hp.sway_offset_y);
        RCLCPP_INFO(get_logger(), "[5b] 移动到成形位姿 Z=%.0fmm roll=%.0f°",
                    hp.draw_height * 1000, hp.roll_draw_start);
        // 从 mix 降到 draw 高度 + 同时改变 roll (slerp 保证正确)
        if (!robot_->moveCartesianStraight(pose_2, vel, acc)) {
            RCLCPP_ERROR(get_logger(), "[5b] moveCartesianStraight 失败");
            ok = false;
        }
        if (ok) {
            auto wps = generateHeartStageWaypoints(origin, hp,
                origin.position.z + hp.draw_height, hp.roll_draw_start, 2);
            RCLCPP_INFO(get_logger(), "[5b] 定点注入 %zu 点", wps.size());
            if (!robot_->executeCartesianPath(wps, vel, acc)) {
                RCLCPP_ERROR(get_logger(), "[5b] executeCartesianPath 失败");
                ok = false;
            }
        }
    }

    // ── [5c] 阶段4: 划穿收尾 (Z=mix=80mm, roll=finish=50°) ──
    if (ok && rclcpp::ok()) {
        auto pose_4 = makeStagePose(origin, hp.mix_height, hp.roll_finish, hp.sway_offset_y);
        RCLCPP_INFO(get_logger(), "[5c] 移动到收尾位姿 Z=%.0fmm roll=%.0f°",
                    hp.mix_height * 1000, hp.roll_finish);
        // 从 draw 升回 mix 高度 + 同时改变 roll (slerp 保证正确)
        if (!robot_->moveCartesianStraight(pose_4, vel, acc)) {
            RCLCPP_ERROR(get_logger(), "[5c] moveCartesianStraight 失败");
            ok = false;
        }
        if (ok) {
            auto wps = generateHeartStageWaypoints(origin, hp,
                origin.position.z + hp.mix_height, hp.roll_finish, 4);
            RCLCPP_INFO(get_logger(), "[5c] 划穿 %zu 点", wps.size());
            if (!robot_->executeCartesianPath(wps, vel, acc)) {
                RCLCPP_ERROR(get_logger(), "[5c] executeCartesianPath 失败");
                ok = false;
            }
        }
    }

    // 通知 trajectory_recorder 停止拉花录制
    if (recorder_stop_cli_->wait_for_service(std::chrono::milliseconds(50))) {
        recorder_stop_cli_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        RCLCPP_INFO(get_logger(), "[步骤5/5] 已通知 trajectory_recorder 停止录制");
    }

    if (ok) {
        RCLCPP_INFO(get_logger(), "[步骤5/5] 完成 (3段共 %s)",
                    hp.verbose ? "融合+成形+收尾" : "");
    } else {
        RCLCPP_ERROR(get_logger(), "[步骤5/5] 失败");
    }
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
