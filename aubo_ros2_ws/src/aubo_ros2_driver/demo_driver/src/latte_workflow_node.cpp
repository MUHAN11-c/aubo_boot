#include "demo_driver/latte_workflow_node.h"
#include <array>

namespace demo_driver
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

    // ── 放咖啡杯 预教关节角 (来源: coffee_approach_pose.json 2026-05-29) ──
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
    if (!has_parameter("lwf_velocity"))        declare_parameter("lwf_velocity", 0.3);
    if (!has_parameter("lwf_acceleration"))    declare_parameter("lwf_acceleration", 0.2);
    if (!has_parameter("lwf_gripper_pin"))     declare_parameter("lwf_gripper_pin", 6);
    if (!has_parameter("lwf_pattern_type"))    declare_parameter("lwf_pattern_type", std::string("heart"));
    if (!has_parameter("lwf_execute_latte"))   declare_parameter("lwf_execute_latte", true);
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
    latte_client_ = create_client<ivg_interfaces::srv::ReplayLatteTrajectory>(
        "/latte/replay_trajectory", rmw_qos_profile_services_default, cb_group_);
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
}

// ═════════════════════════════════════════════════════════════════════════════
// Service 回调 — 5 步咖啡拉花工作流 (step0 保留未启用)
//   [1/5] 取牛奶杯 → [2/5] 打奶泡 → [3/5] 转腕朝上→放置咖啡杯 → [4/5] 嘴口倾倒 → [5/5] 拉花执行
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
    RCLCPP_INFO(get_logger(), "[步骤3/5] 3a 转腕朝上  笛卡尔 -> (%.3f %.3f %.3f)",
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
// 步骤4/5: 嘴口倾倒 — 位置不变, 笛卡尔 X 轴旋转 45° 倾倒牛奶
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step4_pour()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 从当前位置, 绕 X 轴旋转 45° (slerp 保证最短旋转路径)
    auto pour_pose = robot_->getCurrentPose();
    double half_angle = 45.0 * M_PI / 360.0;  // 22.5° half-angle
    double qx = std::sin(half_angle);          // X 轴旋转四元数
    double qw = std::cos(half_angle);
    // q_new = q_rot_x * q_current
    double cx = pour_pose.orientation.x, cy = pour_pose.orientation.y;
    double cz = pour_pose.orientation.z, cw = pour_pose.orientation.w;
    pour_pose.orientation.x = qx * cw + qw * cx;
    pour_pose.orientation.y = qw * cy - qx * cz;
    pour_pose.orientation.z = qx * cy + qw * cz;
    pour_pose.orientation.w = qw * cw - qx * cx;

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
// 步骤5/5: 拉花执行 — 调用 /latte/replay_trajectory 轨迹服务
// ═════════════════════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step5_executeLatte()
{
    if (!params_.execute_latte) {
        RCLCPP_INFO(get_logger(), "[步骤5/5] 跳过 (lwf_execute_latte=false)");
        return true;
    }

    RCLCPP_INFO(get_logger(), "[步骤5/5] 调用拉花服务 pattern=%s", params_.pattern_type.c_str());
    if (!latte_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "[步骤5/5] 拉花服务不可用"); return false;
    }

    auto req = std::make_shared<ivg_interfaces::srv::ReplayLatteTrajectory::Request>();
    req->mode         = "action";
    req->pattern_type = params_.pattern_type;
    req->arm          = "right";
    req->speed_scale  = 1.0;

    auto future = latte_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "[步骤5/5] 超时"); return false;
    }
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(get_logger(), "[步骤5/5] 服务返回失败: %s", response->message.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "[步骤5/5] 完成 — %s", response->message.c_str());
    return true;
}

}  // namespace demo_driver

// ═════════════════════════════════════════════════════════════════════════════
// main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<demo_driver::LatteWorkflowNode>(options);
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
