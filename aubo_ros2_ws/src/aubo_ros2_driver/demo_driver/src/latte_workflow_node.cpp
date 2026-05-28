#include "demo_driver/latte_workflow_node.h"
#include <array>

namespace demo_driver
{

LatteWorkflowNode::LatteWorkflowNode(const rclcpp::NodeOptions& options)
    : Node("latte_workflow_node", options)
{
    // ── 位置参数 ──
    if (!has_parameter("lwf_coffee_link.x"))
        declare_parameter("lwf_coffee_link.x", -0.645);
    if (!has_parameter("lwf_coffee_link.y"))
        declare_parameter("lwf_coffee_link.y", 0.098);
    if (!has_parameter("lwf_coffee_link.z"))
        declare_parameter("lwf_coffee_link.z", 0.05);

    // 拿咖啡杯预教关节角 (来自 ivg_monitor 录制的位姿 #1)
    if (!has_parameter("lwf_coffee_joints"))
        declare_parameter("lwf_coffee_joints", std::vector<double>{
            0.2719387759609349,    // shoulder_joint
            0.8270516803614146,    // upperArm_joint
            -1.3754182925631984,   // foreArm_joint
            -1.0105609246379685,   // wrist1_joint
            -2.8890522224315975,   // wrist2_joint
            1.2586091933562182     // wrist3_joint
        });

    // 放咖啡杯预教关节角 (来自 ivg_monitor 录制的位姿)
    if (!has_parameter("lwf_place_coffee_joints"))
        declare_parameter("lwf_place_coffee_joints", std::vector<double>{
            -2.1125299631189396,   // shoulder_joint
            -1.011760818090542,    // upperArm_joint
            1.4045700292642762,    // foreArm_joint
            2.4942647612447955,    // wrist1_joint
            0.5701059870462559,    // wrist2_joint
            0.01832051537251852    // wrist3_joint
        });

    // 牛奶杯开口朝上 (防倾倒姿态)
    if (!has_parameter("lwf_milk_up_joints"))
        declare_parameter("lwf_milk_up_joints", std::vector<double>{
            -2.1126422363857578,   // shoulder_joint
            -1.0118528235197004,   // upperArm_joint
            1.4045342847643725,    // foreArm_joint
            2.4942737641257944,    // wrist1_joint
            0.570028596954554,     // wrist2_joint
            -1.6537710055541166    // wrist3_joint
        });

    // 牛奶杯嘴口倾倒 (拉花姿态)
    if (!has_parameter("lwf_milk_pour_joints"))
        declare_parameter("lwf_milk_pour_joints", std::vector<double>{
            -2.1127079215348608,   // shoulder_joint
            -1.011765102818058,    // upperArm_joint
            1.4045242122563133,    // foreArm_joint
            2.4944045148414533,    // wrist1_joint
            0.5701142052074466,    // wrist2_joint
            -2.2334196808229465    // wrist3_joint
        });

    // 牛奶杯口朝上位姿 (防倾倒, 笛卡尔直线目标)
    if (!has_parameter("lwf_milk_up_pose.x"))   declare_parameter("lwf_milk_up_pose.x", -0.524);
    if (!has_parameter("lwf_milk_up_pose.y"))   declare_parameter("lwf_milk_up_pose.y", -0.444);
    if (!has_parameter("lwf_milk_up_pose.z"))   declare_parameter("lwf_milk_up_pose.z", 0.164);
    if (!has_parameter("lwf_milk_up_pose.qx"))  declare_parameter("lwf_milk_up_pose.qx", -0.692);
    if (!has_parameter("lwf_milk_up_pose.qy"))  declare_parameter("lwf_milk_up_pose.qy", -0.015);
    if (!has_parameter("lwf_milk_up_pose.qz"))  declare_parameter("lwf_milk_up_pose.qz", 0.722);
    if (!has_parameter("lwf_milk_up_pose.qw"))  declare_parameter("lwf_milk_up_pose.qw", -0.003);

    // 牛奶杯倾倒位姿 (拉花准备, 笛卡尔直线目标)
    if (!has_parameter("lwf_milk_pour_pose.x"))  declare_parameter("lwf_milk_pour_pose.x", -0.524);
    if (!has_parameter("lwf_milk_pour_pose.y"))  declare_parameter("lwf_milk_pour_pose.y", -0.444);
    if (!has_parameter("lwf_milk_pour_pose.z"))  declare_parameter("lwf_milk_pour_pose.z", 0.164);
    if (!has_parameter("lwf_milk_pour_pose.qx")) declare_parameter("lwf_milk_pour_pose.qx", -0.659);
    if (!has_parameter("lwf_milk_pour_pose.qy")) declare_parameter("lwf_milk_pour_pose.qy", -0.212);
    if (!has_parameter("lwf_milk_pour_pose.qz")) declare_parameter("lwf_milk_pour_pose.qz", 0.693);
    if (!has_parameter("lwf_milk_pour_pose.qw")) declare_parameter("lwf_milk_pour_pose.qw", 0.203);

    // 拿取牛奶杯预教关节角
    if (!has_parameter("lwf_pick_milk_joints"))
        declare_parameter("lwf_pick_milk_joints", std::vector<double>{
            0.5114496604395626,    // shoulder_joint
            0.5560157552322338,    // upperArm_joint
            -1.3872343091070718,   // foreArm_joint
            0.24830893922891084,   // wrist1_joint
            -2.248289855414927,    // wrist2_joint
            0.7857612363757142     // wrist3_joint
        });

    // 打花喷嘴终点预教关节角
    if (!has_parameter("lwf_nozzle_joints"))
        declare_parameter("lwf_nozzle_joints", std::vector<double>{
            0.4204691895403951,    // shoulder_joint
            0.3955009389365049,    // upperArm_joint
            -1.428256578941237,    // foreArm_joint
            0.3036545419375402,    // wrist1_joint
            -2.2988592687020133,   // wrist2_joint
            0.6865933110630452     // wrist3_joint
        });

    // 转腕朝上中间位姿 (取牛奶后 → 杯口朝上, 不经过大角度反转)
    if (!has_parameter("lwf_rotate_up_joints"))
        declare_parameter("lwf_rotate_up_joints", std::vector<double>{
            0.5914579558153719,    // shoulder_joint
            0.5766217599758321,    // upperArm_joint
            -2.079998564794082,    // foreArm_joint
            -2.6342428897545616,   // wrist1_joint
            -2.090051028614271,    // wrist2_joint
            -1.5449341576992022    // wrist3_joint
        });

    if (!has_parameter("lwf_lizhu_link.x"))
        declare_parameter("lwf_lizhu_link.x", -0.630);
    if (!has_parameter("lwf_lizhu_link.y"))
        declare_parameter("lwf_lizhu_link.y", -0.368);
    if (!has_parameter("lwf_lizhu_link.z"))
        declare_parameter("lwf_lizhu_link.z", 0.04);

    if (!has_parameter("lwf_cup0_link.x"))
        declare_parameter("lwf_cup0_link.x", -0.528);
    if (!has_parameter("lwf_cup0_link.y"))
        declare_parameter("lwf_cup0_link.y", -0.198);
    if (!has_parameter("lwf_cup0_link.z"))
        declare_parameter("lwf_cup0_link.z", 0.05);

    if (!has_parameter("lwf_nozzle_link.x"))
        declare_parameter("lwf_nozzle_link.x", -0.495);
    if (!has_parameter("lwf_nozzle_link.y"))
        declare_parameter("lwf_nozzle_link.y", -0.269);
    if (!has_parameter("lwf_nozzle_link.z"))
        declare_parameter("lwf_nozzle_link.z", 0.276);

    if (!has_parameter("lwf_reference_pose.x"))
        declare_parameter("lwf_reference_pose.x", -0.419);
    if (!has_parameter("lwf_reference_pose.y"))
        declare_parameter("lwf_reference_pose.y", -0.400);
    if (!has_parameter("lwf_reference_pose.z"))
        declare_parameter("lwf_reference_pose.z", 0.246);
    if (!has_parameter("lwf_reference_pose.roll"))
        declare_parameter("lwf_reference_pose.roll", -23.5);
    if (!has_parameter("lwf_reference_pose.pitch"))
        declare_parameter("lwf_reference_pose.pitch", 88.1);
    if (!has_parameter("lwf_reference_pose.yaw"))
        declare_parameter("lwf_reference_pose.yaw", 76.0);

    // ── 控制参数 ──
    if (!has_parameter("lwf_approach_height"))
        declare_parameter("lwf_approach_height", 0.15);
    if (!has_parameter("lwf_retract_height"))
        declare_parameter("lwf_retract_height", 0.10);
    if (!has_parameter("lwf_velocity"))
        declare_parameter("lwf_velocity", 0.3);
    if (!has_parameter("lwf_acceleration"))
        declare_parameter("lwf_acceleration", 0.2);
    if (!has_parameter("lwf_gripper_pin"))
        declare_parameter("lwf_gripper_pin", 6);
    if (!has_parameter("lwf_pattern_type"))
        declare_parameter("lwf_pattern_type", std::string("heart"));
    if (!has_parameter("lwf_execute_latte"))
        declare_parameter("lwf_execute_latte", true);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_ = create_service<ivg_interfaces::srv::RunLatteWorkflow>(
        "/latte/run_workflow",
        [this](const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> req,
               std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res) {
            handleRunWorkflow(req, res);
        },
        rmw_qos_profile_services_default, cb_group_);

    RCLCPP_INFO(get_logger(), "LatteWorkflowNode constructed, awaiting init()");
}

bool LatteWorkflowNode::init()
{
    robot_ = std::make_shared<RobotController>(this);
    if (!robot_->init()) {
        RCLCPP_ERROR(get_logger(), "RobotController init failed");
        return false;
    }

    latte_client_ = create_client<ivg_interfaces::srv::ReplayLatteTrajectory>(
        "/latte/replay_trajectory", rmw_qos_profile_services_default, cb_group_);

    RCLCPP_INFO(get_logger(), "LatteWorkflowNode ready");
    return true;
}

void LatteWorkflowNode::readParameters()
{
    params_.coffee_x  = get_parameter("lwf_coffee_link.x").as_double();
    params_.coffee_y  = get_parameter("lwf_coffee_link.y").as_double();
    params_.coffee_z  = get_parameter("lwf_coffee_link.z").as_double();
    params_.lizhu_x   = get_parameter("lwf_lizhu_link.x").as_double();
    params_.lizhu_y   = get_parameter("lwf_lizhu_link.y").as_double();
    params_.lizhu_z   = get_parameter("lwf_lizhu_link.z").as_double();
    params_.cup0_x    = get_parameter("lwf_cup0_link.x").as_double();
    params_.cup0_y    = get_parameter("lwf_cup0_link.y").as_double();
    params_.cup0_z    = get_parameter("lwf_cup0_link.z").as_double();
    params_.nozzle_x  = get_parameter("lwf_nozzle_link.x").as_double();
    params_.nozzle_y  = get_parameter("lwf_nozzle_link.y").as_double();
    params_.nozzle_z  = get_parameter("lwf_nozzle_link.z").as_double();
    params_.ref_x     = get_parameter("lwf_reference_pose.x").as_double();
    params_.ref_y     = get_parameter("lwf_reference_pose.y").as_double();
    params_.ref_z     = get_parameter("lwf_reference_pose.z").as_double();
    params_.ref_roll  = get_parameter("lwf_reference_pose.roll").as_double();
    params_.ref_pitch = get_parameter("lwf_reference_pose.pitch").as_double();
    params_.ref_yaw   = get_parameter("lwf_reference_pose.yaw").as_double();
    params_.approach_h = get_parameter("lwf_approach_height").as_double();
    params_.retract_h  = get_parameter("lwf_retract_height").as_double();
    params_.vel        = get_parameter("lwf_velocity").as_double();
    params_.acc        = get_parameter("lwf_acceleration").as_double();
    params_.gripper_pin   = get_parameter("lwf_gripper_pin").as_int();
    params_.pattern_type  = get_parameter("lwf_pattern_type").as_string();
    params_.execute_latte = get_parameter("lwf_execute_latte").as_bool();

    auto joints_vec = get_parameter("lwf_coffee_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.coffee_joints.begin());
    }
    joints_vec = get_parameter("lwf_place_coffee_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.place_coffee_joints.begin());
    }
    joints_vec = get_parameter("lwf_milk_up_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.milk_up_joints.begin());
    }
    joints_vec = get_parameter("lwf_milk_pour_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.milk_pour_joints.begin());
    }
    joints_vec = get_parameter("lwf_pick_milk_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.pick_milk_joints.begin());
    }
    joints_vec = get_parameter("lwf_nozzle_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.nozzle_joints.begin());
    }
    joints_vec = get_parameter("lwf_rotate_up_joints").as_double_array();
    if (joints_vec.size() == 6) {
        std::copy_n(joints_vec.begin(), 6, params_.rotate_up_joints.begin());
    }
}

// ═══════════════════════════════════════════════════════════════
// Service 回调
// ═══════════════════════════════════════════════════════════════

void LatteWorkflowNode::handleRunWorkflow(
    const std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Request> /*req*/,
    std::shared_ptr<ivg_interfaces::srv::RunLatteWorkflow::Response> res)
{
    std::lock_guard<std::mutex> lock(mtx_);
    readParameters();

    RCLCPP_INFO(get_logger(), "══════ Latte Workflow Start (7 steps) ══════");

    // if (!step1_pickCoffee())     { res->success = false; res->message = "Step 1 failed: pick coffee";   return; }
    // if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    // if (!step2_placeCoffee())    { res->success = false; res->message = "Step 2 failed: place coffee";  return; }
    // if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    if (!step3_pickMilk())       { res->success = false; res->message = "Step 3 failed: pick milk";     return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    if (!step4_approachNozzle()) { res->success = false; res->message = "Step 4 failed: approach nozzle"; return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    if (!step5_reorient())       { res->success = false; res->message = "Step 5 failed: reorient";      return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    if (!step6_approachLizhu())  { res->success = false; res->message = "Step 6 failed: tilt pour";     return; }
    if (!rclcpp::ok())           { res->success = false; res->message = "ROS shutdown"; return; }
    if (!step7_callLatte())      { res->success = false; res->message = "Step 7 failed: latte service"; return; }

    res->success = true;
    res->message = "All 7 steps completed";
    RCLCPP_INFO(get_logger(), "══════ Latte Workflow Done ══════");
}

// ═══════════════════════════════════════════════════════════════
// Step 1: 取咖啡杯
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step1_pickCoffee()
{
    RCLCPP_INFO(get_logger(), "Step 1: pick coffee, joints=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                params_.coffee_joints[0], params_.coffee_joints[1], params_.coffee_joints[2],
                params_.coffee_joints[3], params_.coffee_joints[4], params_.coffee_joints[5]);
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 关节空间直达标预教位姿 (已知可行的 IK 解)
    if (!robot_->moveToJoints(params_.coffee_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 1: moveToJoints failed");
        return false;
    }

    // 抓取 (仿真下 IO 不可用，警告但继续)
    if (!robot_->setGripper(params_.gripper_pin, false)) {
        RCLCPP_WARN(get_logger(), "Step 1 grip failed (simulation?), continuing");
    }

    // Cartesian Z 退避
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 1 retract failed");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 2: 放咖啡杯 (保持朝向)
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step2_placeCoffee()
{
    RCLCPP_INFO(get_logger(), "Step 2: place coffee, joints=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                params_.place_coffee_joints[0], params_.place_coffee_joints[1], params_.place_coffee_joints[2],
                params_.place_coffee_joints[3], params_.place_coffee_joints[4], params_.place_coffee_joints[5]);
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 关节空间直达标预教位姿
    if (!robot_->moveToJoints(params_.place_coffee_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 2: moveToJoints failed");
        return false;
    }

    // 放置 (仿真下 IO 不可用，警告但继续)
    if (!robot_->setGripper(params_.gripper_pin, true)) {
        RCLCPP_WARN(get_logger(), "Step 2 release failed (simulation?), continuing");
    }

    // Cartesian Z 退避
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 2 retract failed");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 3: 取牛奶杯
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step3_pickMilk()
{
    RCLCPP_INFO(get_logger(), "Step 3: pick milk, joints=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                params_.pick_milk_joints[0], params_.pick_milk_joints[1], params_.pick_milk_joints[2],
                params_.pick_milk_joints[3], params_.pick_milk_joints[4], params_.pick_milk_joints[5]);
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 关节空间直达取牛奶预教位姿
    if (!robot_->moveToJoints(params_.pick_milk_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 3: moveToJoints failed");
        return false;
    }

    // 抓取 (仿真下 IO 不可用，警告但继续)
    if (!robot_->setGripper(params_.gripper_pin, false)) {
        RCLCPP_WARN(get_logger(), "Step 3 grip failed (simulation?), continuing");
    }

    // 不退避 — Step 4 从取牛奶精准位姿起点做笛卡尔来回
    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 4: 靠近咖啡机喷嘴打奶泡 (空间斜线)
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step4_approachNozzle()
{
    RCLCPP_INFO(get_logger(), "Step 4: approach nozzle at (%.3f, %.3f, %.3f)",
                params_.nozzle_x, params_.nozzle_y, params_.nozzle_z);
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    auto start_pose = robot_->getCurrentPose();

    // 目标位姿: 朝向不变, 位置 = 喷嘴终点
    geometry_msgs::msg::Pose nozzle_pose = start_pose;
    nozzle_pose.position.x = params_.nozzle_x;
    nozzle_pose.position.y = params_.nozzle_y;
    nozzle_pose.position.z = params_.nozzle_z;

    // 单 waypoint 空间直线靠近喷嘴
    if (!robot_->moveCartesianStraight(nozzle_pose, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 4 approach nozzle failed");
        return false;
    }

    // 打奶泡 — 机械放置, 短暂等待
    RCLCPP_INFO(get_logger(), "Step 4: frothing (sleep 2s)");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 空间直线原路退回取牛奶精准位姿
    if (!robot_->moveCartesianStraight(start_pose, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 4 retract from nozzle failed");
        return false;
    }

    // Z 退避, 为 Step 5 大跨度关节运动提供安全距离
    if (!robot_->moveCartesianZ(+params_.retract_h, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 4 retract Z failed");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 5: 转腕对轴 — 牛奶杯开口朝上 (防倾倒)
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step5_reorient()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 读取 milk_up 目标位姿
    geometry_msgs::msg::Pose milk_up;
    milk_up.position.x    = get_parameter("lwf_milk_up_pose.x").as_double();
    milk_up.position.y    = get_parameter("lwf_milk_up_pose.y").as_double();
    milk_up.position.z    = get_parameter("lwf_milk_up_pose.z").as_double();
    milk_up.orientation.x = get_parameter("lwf_milk_up_pose.qx").as_double();
    milk_up.orientation.y = get_parameter("lwf_milk_up_pose.qy").as_double();
    milk_up.orientation.z = get_parameter("lwf_milk_up_pose.qz").as_double();
    milk_up.orientation.w = get_parameter("lwf_milk_up_pose.qw").as_double();

    // 5a: 关节运动到转腕朝上中间位姿 (不经过大角度反转)
    RCLCPP_INFO(get_logger(), "Step 5a: moveToJoints rotate_up, wrist3=%.3f",
                params_.rotate_up_joints[5]);
    if (!robot_->moveToJoints(params_.rotate_up_joints, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 5a: moveToJoints rotate_up failed");
        return false;
    }

    // 5b: 笛卡尔直线平移 → 放置咖啡杯位置 (保持杯口朝上)
    RCLCPP_INFO(get_logger(), "Step 5b: Cartesian straight to (%.3f,%.3f,%.3f)",
                milk_up.position.x, milk_up.position.y, milk_up.position.z);
    if (!robot_->moveCartesianStraight(milk_up, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 5b: Cartesian straight failed");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 6: 嘴口倾倒 (准备拉花), 仅 wrist3 旋转
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step6_approachLizhu()
{
    const float vel = static_cast<float>(params_.vel);
    const float acc = static_cast<float>(params_.acc);

    // 笛卡尔直线: milk_up 位姿 → milk_pour 位姿 (仅 wrist3 倾倒, 位置不变)
    geometry_msgs::msg::Pose milk_pour;
    milk_pour.position.x    = get_parameter("lwf_milk_pour_pose.x").as_double();
    milk_pour.position.y    = get_parameter("lwf_milk_pour_pose.y").as_double();
    milk_pour.position.z    = get_parameter("lwf_milk_pour_pose.z").as_double();
    milk_pour.orientation.x = get_parameter("lwf_milk_pour_pose.qx").as_double();
    milk_pour.orientation.y = get_parameter("lwf_milk_pour_pose.qy").as_double();
    milk_pour.orientation.z = get_parameter("lwf_milk_pour_pose.qz").as_double();
    milk_pour.orientation.w = get_parameter("lwf_milk_pour_pose.qw").as_double();

    RCLCPP_INFO(get_logger(), "Step 6: Cartesian tilt to pour (%.3f,%.3f,%.3f)",
                milk_pour.position.x, milk_pour.position.y, milk_pour.position.z);

    if (!robot_->moveCartesianStraight(milk_pour, vel, acc)) {
        RCLCPP_ERROR(get_logger(), "Step 6: moveCartesianStraight failed");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════
// Step 7: 调用拉花服务
// ═══════════════════════════════════════════════════════════════

bool LatteWorkflowNode::step7_callLatte()
{
    if (!params_.execute_latte) {
        RCLCPP_INFO(get_logger(), "Step 7: SKIPPED (lwf_execute_latte=false)");
        return true;
    }

    RCLCPP_INFO(get_logger(), "Step 7: calling /latte/replay_trajectory "
                "(pattern=%s, mode=action)", params_.pattern_type.c_str());

    if (!latte_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Step 7: /latte/replay_trajectory not available");
        return false;
    }

    auto req = std::make_shared<ivg_interfaces::srv::ReplayLatteTrajectory::Request>();
    req->mode          = "action";
    req->pattern_type  = params_.pattern_type;
    req->arm           = "right";
    req->speed_scale   = 1.0;
    // start_pose 全零 → latte_node 自动 TF 查询当前 EE 位姿
    // cup params 使用 latte_node 默认值 (牛奶杯已位于杯子上方)

    auto future = latte_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Step 7: latte service timeout");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(get_logger(), "Step 7: latte service failed: %s",
                     response->message.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "Step 7: latte service ok — %s",
                response->message.c_str());
    return true;
}

}  // namespace demo_driver

// ═══════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<demo_driver::LatteWorkflowNode>(options);
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "LatteWorkflowNode init failed");
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
