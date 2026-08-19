// Copyright 2026 aubo_e5_jazzy_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the aubo_e5_ros2_ws authors nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// 阶段 B P0 修复的节点层集成测试：进程内构造真实编排器节点（经
// make_orchestrator_node_for_test，模式同 peach_approach_grasp 的
// test_lifecycle）+ 进程内假能力端（假 RunTargetCycle action 服务、假感知
// Trigger 服务、假 lifecycle GetState、三路就绪话题发布），全部行为经
// 原子/回调脚本化。用例：
//   1. AdvanceFailsRetryThenFuseAndRecoveryAllowsRedispatch
//      —— 协议 2.6 推进协议：complete_selected_target 持续失败 → 冷却重试
//      （每次 ERROR 事件）→ 连续 max_retries 次熔断 RECOVERY_REQUIRED；
//      ACKNOWLEDGE_RECOVERY + RESUME 后派发锁已清（2.5-L4-(c)/(e)），同一
//      selected 可重新派发（R1 死锁回归）。
//   2. CancelNowTerminatesRunHarvestAndAcceptsNewGoal
//      —— 协议 2.2-T9/2.9：CANCEL_NOW 落 INTERRUPTED 后 RunHarvest 以
//      canceled 终局退出（不空转占用单 goal 守卫），随后新 goal 可受理。
//   3. RejectionWhileMotionNotReadyNeverFuses
//      —— 协议 2.5-R-2（R3 修复）：goal 拒绝落入本端 motion 失能窗口时只
//      冷却重试，不计 consecutive_rejections、不耗 dispatch_retries。
//   4. OperatorSkipBindsCycleId
//      —— 协议 2.9/2.6-2：SKIP_TARGET 绑定活动 cycle_id；取消落地且周期
//      匹配才记"操作员跳过"；skip 请求后周期自然成功（取消被能力端拒绝）
//      不得误记为操作员跳过。
//   5. CollectTimeoutRejudgesRoundAndReportsProgress（阶段 C 新增）
//      —— 协议 2.3 编排侧监督 + R-D8：收齐窗口超 collect_timeout_s 未锁定
//      发 round_stall 并按本轮处理完重判收口；未锁定期 message 透出
//      "收齐中 N 目标/M 确认中"。
//   6. CycleIdsAreUniqueAndCarryRunId（阶段 C 新增）
//      —— 协议 2.5 goal 契约：cycle_id=<run_id>-cycle-<批次内单调序号>；
//      auto_start 批次 run_id=auto-<时间戳>。
//   7. DroppedTargetBookedAsSkippedUnreachable（阶段 D2 新增）
//      —— 协议 2.4：锁定集目标未派发即从锁定集消失（anchor_drop）→ 记
//      SKIPPED_UNREACHABLE（reason 目标丢失超时）+ AUDIT 事件
//      target_dropped，进 HarvestSummary。
//   8. ResetAndRelockDoesNotBookDroppedTargets（阶段 D2 新增）
//      —— 误记账防御：reset/收齐重开（解锁→换集→再锁定）路径不对旧集
//      目标记丢失；再锁定后新集目标消失仍正常记账。
//   9. ActiveTargetVanishIsNotDoubleBooked（阶段 D2 新增）
//      —— 活动周期目标中途消失：丢失记账让位 result_callback 终局账，
//      不双记（target_dropped=0，周期按 SUCCEEDED 入账一次）。
//  10. FreshSceneClearsMemoryBeforeReset（阶段 D2 新增）
//      —— 协议 2.3：harvest.fresh_scene=true 时拍照前置在
//      reset_global_targets 之前调 clear_target_memory；首次拒绝走
//      photo_step 冷却重试链后成功，顺序 clear→clear→reset。
//  11. FreshSceneDisabledSkipsClearMemory（阶段 D2 新增）
//      —— fresh_scene=false（默认）：拍照前置不调 clear_target_memory，
//      顺序仅 reset。
//  12. ObserveRetryEndgameLiftsQualityThenRetriesFull（阶段 E3 新增）
//      —— 协议 2.8 残局：SKIPPED_QUALITY 残局目标触发 OBSERVE_ONLY 派发，
//      抬质量成功（重新可选）后回正常链 FULL 重试；observe 终局只进审计账；
//      stall 协同——有残局目标在处理不算停滞。
//  13. ObserveRetryRejectionIsBoundedAndDoesNotFuse（阶段 E3 新增）
//      —— observe-retry goal 被拒 best-effort：不记账/不熔断/不推进，
//      次数派发时消耗+本轮去重保证系统性拒绝下有界收口。
//  14. PreflightBlocksPhotoChainUntilGeometryReady（阶段 F3 新增）
//      —— harvest.preflight_check 就位自校：TF 外参链未就位时批次挂起
//      于拍照前置前（blockers 带 preflight、ERROR 事件、零派发）；广播
//      静态 TF 就位后幂等门自动放行，批次恢复派发。
//      注：harness 默认把 harvest.preflight_check 覆盖为 false（假现场
//      无 TF 发布端），仅本用例置 true。
#include <gtest/gtest.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "aubo_msgs/msg/robot_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "peach_harvest_msgs/action/run_harvest.hpp"
#include "peach_harvest_msgs/action/run_target_cycle.hpp"
#include "peach_harvest_msgs/msg/harvest_event.hpp"
#include "peach_harvest_msgs/msg/harvest_state.hpp"
#include "peach_harvest_msgs/msg/harvest_summary.hpp"
#include "peach_harvest_msgs/msg/reconstruction_status.hpp"
#include "peach_harvest_msgs/msg/target_outcome.hpp"
#include "peach_harvest_msgs/srv/control_harvest.hpp"
#include "peach_pose_msgs/msg/peach_target_observation_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "orchestrator_test_access.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
using namespace std::chrono_literals;
using ControlHarvest = peach_harvest_msgs::srv::ControlHarvest;
using HarvestEvent = peach_harvest_msgs::msg::HarvestEvent;
using HarvestState = peach_harvest_msgs::msg::HarvestState;
using RunHarvest = peach_harvest_msgs::action::RunHarvest;
using RunTargetCycle = peach_harvest_msgs::action::RunTargetCycle;
using TargetOutcomeMsg = peach_harvest_msgs::msg::TargetOutcome;
using Trigger = std_srvs::srv::Trigger;

// 假能力端节点基类：接口（服务/action 服务/发布者）一律在 on_configure
// 回调内装配。2026-08-18 本机实测（FastDDS 异常窗口，疑与当日内核升级
// 未重启有关）：普通节点上直接 create_service 的晚建端点会静默丢失
// （图可见但可用性检查失败、请求不到达），而 LifecycleNode 生命周期转换
// 回调内创建的端点正常（编排器/approach_grasp 的真实节点均为此模式）。
class FakeLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ConfigureHook = std::function<void(FakeLifecycleNode &)>;

  FakeLifecycleNode(const std::string & name, ConfigureHook hook)
  : rclcpp_lifecycle::LifecycleNode(name), hook_(std::move(hook))
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    hook_(*this);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
           CallbackReturn::SUCCESS;
  }

private:
  ConfigureHook hook_;
};

// 假现场：能力端 action 服务/lifecycle GetState、感知 Trigger 服务与三路
// 就绪话题发布，全部行为经原子/回调脚本化。
struct FakeField
{
  // --- 脚本 ---
  std::atomic<bool> complete_success{true};
  std::atomic<int> complete_calls{0};
  // 第 n 次 complete 调用后的副作用（如切换 selected）。
  std::function<void(int)> on_complete;
  // 第 n 个 goal 的受理判定（返回 false=拒绝）；默认全部受理。
  std::function<bool(int, const RunTargetCycle::Goal &)> goal_handler;
  // 取消请求处理（返回 false=拒绝取消，周期按脚本终局自然完成）。
  std::function<bool(const std::string & target_id)> cancel_handler;
  // 受理后终局脚本：按 goal 序号与 target_id 调终局延迟/结果
  // （默认 300ms 后 SUCCEEDED）。
  std::function<void(int, const std::string &,
    std::atomic<int64_t> &, std::atomic<uint8_t> &)> finish_script;
  std::atomic<bool> estop{false};
  // 感知锁定沿脚本：false 时观测流持续发未锁定帧（收齐窗口不关闭），
  // 供收齐超时监督（2.3）用例；collecting/pending 为发现进度摘要（R-D8）。
  std::atomic<bool> targets_locked_flag{true};
  std::atomic<int32_t> collecting_count{0};
  std::atomic<int32_t> pending_count{0};
  // selected 由观测发布定时器读取、测试/complete 副作用改写：互斥保护的
  // 普通字符串（std::atomic<std::string> 在 C++17 不可用）。
  std::string selected;
  std::mutex selected_mutex;
  // 锁定集目标 ID：观测发布定时器逐帧读取；启动前直接赋值即可，运行中
  // 修改必须走 set_target_ids（互斥保护，模拟 anchor_drop 移出锁定集）。
  std::vector<std::string> target_ids{"peach_1"};
  std::mutex target_ids_mutex;
  // fresh_scene（协议 2.3）脚本：clear_target_memory 成功判定（默认恒成功，
  // 可挂 clear_handler 按调用序脚本化）与调用计数；reset 调用计数；
  // pose_call_order 记录 clear/reset 到达顺序（断言 clear 先于 reset）。
  std::atomic<bool> clear_success{true};
  std::function<bool(int)> clear_handler;
  std::atomic<int> clear_calls{0};
  std::atomic<int> reset_calls{0};
  std::vector<std::string> pose_call_order;
  std::mutex pose_order_mutex;
  // --- 观测 ---
  std::atomic<int> goals_received{0};
  std::atomic<int> cancels_received{0};
  std::atomic<bool> stop_all{false};
  // 已受理 goal 的 cycle_id 流水（goal 回调按序记录；唯一性/格式断言用）。
  std::vector<std::string> cycle_ids;
  // 与 cycle_ids 等长的 goal mode 流水（FULL/OBSERVE_ONLY，E3 残局抬质量
  // 用例断言用），同一把锁保护。
  std::vector<uint8_t> goal_modes;
  std::mutex cycle_ids_mutex;

  std::shared_ptr<FakeLifecycleNode> approach_node;
  std::shared_ptr<FakeLifecycleNode> pose_node;
  std::shared_ptr<FakeLifecycleNode> field_node;
  rclcpp_action::Server<RunTargetCycle>::SharedPtr cycle_server;
  std::vector<std::thread> finish_threads;
  std::mutex finish_threads_mutex;

  void set_selected(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(selected_mutex);
    selected = value;
  }

  std::string get_selected()
  {
    std::lock_guard<std::mutex> lock(selected_mutex);
    return selected;
  }

  void set_target_ids(const std::vector<std::string> & ids)
  {
    std::lock_guard<std::mutex> lock(target_ids_mutex);
    target_ids = ids;
  }

  std::vector<std::string> get_target_ids()
  {
    std::lock_guard<std::mutex> lock(target_ids_mutex);
    return target_ids;
  }

  std::vector<std::string> pose_order()
  {
    std::lock_guard<std::mutex> lock(pose_order_mutex);
    return pose_call_order;
  }

  // 建节点并入 executor；接口在各节点 on_configure 内装配（见
  // FakeLifecycleNode 注释），configure+activate 后全开闸。
  void start(rclcpp::executors::MultiThreadedExecutor & executor)
  {
    approach_node = std::make_shared<FakeLifecycleNode>(
      "peach_approach_grasp_node",
      [this](FakeLifecycleNode & node) {configure_approach(node);});
    pose_node = std::make_shared<FakeLifecycleNode>(
      "peach_pose_node",
      [this](FakeLifecycleNode & node) {configure_pose(node);});
    field_node = std::make_shared<FakeLifecycleNode>(
      "dispatch_protocol_field",
      [this](FakeLifecycleNode & node) {configure_field(node);});
    executor.add_node(approach_node->get_node_base_interface());
    executor.add_node(pose_node->get_node_base_interface());
    executor.add_node(field_node->get_node_base_interface());
  }

  void activate_all()
  {
    approach_node->configure();
    approach_node->activate();
    pose_node->configure();
    pose_node->activate();
    field_node->configure();
    field_node->activate();
  }

  void stop()
  {
    stop_all.store(true);
    std::lock_guard<std::mutex> lock(finish_threads_mutex);
    for (auto & thread : finish_threads) {
      if (thread.joinable()) {thread.join();}
    }
  }

private:
  // 能力端接口：对端 lifecycle GetState（A8 轮询应答，恒 Active）、
  // go_to_photo_pose、RunTargetCycle action 服务。
  void configure_approach(FakeLifecycleNode & node)
  {
    node.create_service<lifecycle_msgs::srv::GetState>(
      "get_state",
      [](const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,
      std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {
        response->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        response->current_state.label = "active";
      });
    node.create_service<Trigger>(
      "go_to_photo_pose",
      [](const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> response) {
        response->success = true;
        response->message = "已到拍照位姿";
      });
    cycle_server = rclcpp_action::create_server<RunTargetCycle>(
      node.shared_from_this(), "run_target_cycle",
      [this](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const RunTargetCycle::Goal> goal) {
        const int index = goals_received.fetch_add(1) + 1;
        {
          std::lock_guard<std::mutex> lock(cycle_ids_mutex);
          cycle_ids.push_back(goal->cycle_id);
          goal_modes.push_back(goal->mode);
        }
        if (goal_handler) {
          return goal_handler(index, *goal) ?
                 rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                 rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunTargetCycle>> handle) {
        cancels_received.fetch_add(1);
        if (cancel_handler && !cancel_handler(handle->get_goal()->target_id)) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunTargetCycle>> handle) {
        // 每 goal 终局参数由 finish_script 写入（默认 300ms/SUCCEEDED）。
        auto delay_ms = std::make_shared<std::atomic<int64_t>>(300);
        auto outcome = std::make_shared<std::atomic<uint8_t>>(
          RunTargetCycle::Result::SUCCEEDED);
        const int index = goals_received.load();
        if (finish_script) {
          finish_script(index, handle->get_goal()->target_id, *delay_ms, *outcome);
        }
        std::lock_guard<std::mutex> lock(finish_threads_mutex);
        finish_threads.emplace_back(
          [this, handle, delay_ms, outcome]() {
            const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(delay_ms->load());
            while (rclcpp::ok() && !stop_all.load()) {
              if (handle->is_canceling()) {
                // 取消落地：显式 outcome=CANCELED（2026-08 契约）。
                auto result = std::make_shared<RunTargetCycle::Result>();
                result->outcome = RunTargetCycle::Result::CANCELED;
                result->reason = "（假）周期已取消";
                try {handle->canceled(result);} catch (...) {}
                return;
              }
              if (std::chrono::steady_clock::now() >= deadline) {
                auto result = std::make_shared<RunTargetCycle::Result>();
                result->outcome = outcome->load();
                result->reason = "（假）周期终局";
                try {
                  if (outcome->load() == RunTargetCycle::Result::SUCCEEDED) {
                    handle->succeed(result);
                  } else {
                    handle->abort(result);
                  }
                } catch (...) {}
                return;
              }
              std::this_thread::sleep_for(20ms);
            }
          });
      },
      // 与编排器自身 action server 完全一致的装配：默认 options + 显式回调组
      rcl_action_server_get_default_options(),
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }

  // 感知接口：complete_selected_target（脚本化成功/失败+副作用）、
  // reset_global_targets、clear_target_memory（fresh_scene，协议 2.3）。
  // clear/reset 到达顺序记入 pose_call_order 供用例断言调用链。
  void configure_pose(FakeLifecycleNode & node)
  {
    node.create_service<Trigger>(
      "complete_selected_target",
      [this](const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> response) {
        const int call = complete_calls.fetch_add(1) + 1;
        response->success = complete_success.load();
        response->message = response->success ? "已推进" : "（假）感知拒绝推进";
        if (on_complete) {on_complete(call);}
      });
    node.create_service<Trigger>(
      "reset_global_targets",
      [this](const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> response) {
        reset_calls.fetch_add(1);
        {
          std::lock_guard<std::mutex> lock(pose_order_mutex);
          pose_call_order.push_back("reset");
        }
        response->success = true;
        response->message = "已重置";
      });
    node.create_service<Trigger>(
      "clear_target_memory",
      [this](const std::shared_ptr<Trigger::Request>,
      std::shared_ptr<Trigger::Response> response) {
        const int call = clear_calls.fetch_add(1) + 1;
        {
          std::lock_guard<std::mutex> lock(pose_order_mutex);
          pose_call_order.push_back("clear");
        }
        response->success = clear_handler ? clear_handler(call) : clear_success.load();
        response->message = response->success ? "已清空身份记忆" : "（假）感知拒绝清记忆";
      });
  }

  // 三路就绪话题：感知观测（锁定+selected）、重建诊断（transient_local +
  // offered deadline 1.5s，与订阅侧 requested 2s 兼容）、robot_status。
  void configure_field(FakeLifecycleNode & node)
  {
    const auto observations_pub =
      node.create_publisher<peach_pose_msgs::msg::PeachTargetObservationArray>(
      "/peach/perception/target_observations", 10);
    observations_timer_ = node.create_wall_timer(
      200ms,
      [this, observations_pub]() {
        peach_pose_msgs::msg::PeachTargetObservationArray message;
        message.header.stamp = field_node->now();
        message.target_set_locked = targets_locked_flag.load();
        // 发现进度摘要（R-D8）：锁定前透出收齐进度；锁定后 collecting=
        // 锁定集大小、pending 恒 0（消息契约注释语义）。
        const auto ids = get_target_ids();
        if (message.target_set_locked) {
          message.target_count = static_cast<uint32_t>(ids.size());
          message.selected_target_id = get_selected();
          message.collecting_count = static_cast<int32_t>(ids.size());
          message.pending_count = 0;
          for (const auto & id : ids) {
            peach_pose_msgs::msg::PeachTargetObservation observation;
            observation.header = message.header;
            observation.target_id = id;
            observation.confirmed = true;
            observation.selected = id == message.selected_target_id;
            message.observations.push_back(observation);
          }
        } else {
          // 未锁定帧：observations/selected 恒空（契约），只发摘要计数。
          message.target_count = 0;
          message.collecting_count = collecting_count.load();
          message.pending_count = pending_count.load();
        }
        observations_pub->publish(message);
      });
    const auto recon_pub =
      node.create_publisher<peach_harvest_msgs::msg::ReconstructionStatus>(
      "/peach/reconstruction/diagnostics",
      rclcpp::QoS(1).transient_local().deadline(rclcpp::Duration::from_seconds(1.5)));
    recon_timer_ = node.create_wall_timer(
      500ms,
      [this, recon_pub]() {
        peach_harvest_msgs::msg::ReconstructionStatus message;
        message.header.stamp = field_node->now();
        recon_pub->publish(message);
      });
    const auto robot_pub = node.create_publisher<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", 10);
    robot_timer_ = node.create_wall_timer(
      100ms,
      [this, robot_pub]() {
        aubo_msgs::msg::RobotStatus message;
        const bool stopped = estop.load();
        message.e_stopped = stopped ? 1 : 0;
        message.in_error = 0;
        message.drives_powered = stopped ? 0 : 1;
        message.motion_possible = stopped ? 0 : 1;
        robot_pub->publish(message);
      });
  }

  rclcpp::TimerBase::SharedPtr observations_timer_;
  rclcpp::TimerBase::SharedPtr recon_timer_;
  rclcpp::TimerBase::SharedPtr robot_timer_;
};

// 测试侧观察面：~/state 最新快照 + ~/events 事件流水。
struct Observer
{
  rclcpp::Node::SharedPtr node;
  std::mutex mutex;
  HarvestState latest_state;
  bool has_state{false};
  bool recovery_seen{false};
  std::vector<std::string> event_codes;
  std::vector<std::string> event_messages;

  void start(rclcpp::executors::MultiThreadedExecutor & executor)
  {
    node = std::make_shared<rclcpp::Node>("dispatch_protocol_observer");
    state_sub_ = node->create_subscription<HarvestState>(
      "/peach_harvest_orchestrator/state", rclcpp::QoS(1).transient_local(),
      [this](const HarvestState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        latest_state = *message;
        has_state = true;
        recovery_seen = recovery_seen || message->recovery_required;
      });
    event_sub_ = node->create_subscription<HarvestEvent>(
      "/peach_harvest_orchestrator/events", rclcpp::QoS(50),
      [this](const HarvestEvent::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        event_codes.push_back(message->code);
        event_messages.push_back(message->message);
      });
    executor.add_node(node);
  }

  HarvestState state()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return latest_state;
  }

  int count_event(const std::string & code)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return static_cast<int>(std::count(
        event_codes.begin(), event_codes.end(), code));
  }

  bool any_event_message_contains(const std::string & needle)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & message : event_messages) {
      if (message.find(needle) != std::string::npos) {return true;}
    }
    return false;
  }

private:
  rclcpp::Subscription<HarvestState>::SharedPtr state_sub_;
  rclcpp::Subscription<HarvestEvent>::SharedPtr event_sub_;
};

template<typename Predicate>
bool wait_for(Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(50ms);
  }
  return predicate();
}

// 控制命令带 expected_revision 乐观锁：版本漂移被拒后按最新 revision 重试
// （每次新 request_id 避开幂等缓存）。
bool send_control(
  const rclcpp::Client<ControlHarvest>::SharedPtr & client, Observer & observer,
  uint8_t command, const std::string & tag)
{
  for (int attempt = 0; attempt < 30; ++attempt) {
    auto request = std::make_shared<ControlHarvest::Request>();
    request->request_id = tag + "-" + std::to_string(attempt);
    request->command = command;
    request->expected_revision = observer.state().revision;
    auto future = client->async_send_request(request);
    if (future.wait_for(2s) == std::future_status::ready &&
      future.get()->accepted)
    {
      return true;
    }
    std::this_thread::sleep_for(200ms);
  }
  return false;
}

struct FakeRemote
{
  struct Flag
  {
    FakeRemote * self{nullptr};
    const char * key{nullptr};
    void store(bool value)
    {
      self->send(std::string(key) + (value ? "=1" : "=0"));
    }
  };
  struct IntFlag
  {
    FakeRemote * self{nullptr};
    const char * key{nullptr};
    void store(int value)
    {
      self->send(std::string(key) + "=" + std::to_string(value));
    }
  };
  struct Goals
  {
    FakeRemote * self{nullptr};
    int load() const {return self->status_int("goals_received");}
  };
  struct Calls
  {
    FakeRemote * self{nullptr};
    const char * key{nullptr};
    int load() const {return self->status_int(key);}
  };

  Flag complete_success{this, "complete_success"};
  Flag targets_locked_flag{this, "locked"};
  Flag estop{this, "estop"};
  IntFlag collecting_count{this, "collecting_count"};
  IntFlag pending_count{this, "pending_count"};
  Goals goals_received{this};
  Calls clear_calls{this, "clear_calls"};
  Calls reset_calls{this, "reset_calls"};
  std::mutex cycle_ids_mutex;
  std::vector<std::string> cycle_ids;
  std::vector<uint8_t> goal_modes;
  std::function<void(int)> on_complete;
  std::function<bool(int, const RunTargetCycle::Goal &)> goal_handler;
  std::function<bool(const std::string &)> cancel_handler;
  std::function<void(int, const std::string &,
    std::atomic<int64_t> &, std::atomic<uint8_t> &)> finish_script;
  std::function<bool(int)> clear_handler;
  std::vector<std::string> target_ids{"peach_1"};

  void bind(const rclcpp::Node::SharedPtr & node)
  {
    cmd_pub_ = node->create_publisher<std_msgs::msg::String>(
      "/fake_field/command", 20);
    status_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/fake_field/status", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::vector<std::string> to_flush;
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          status_json_ = message->data;
          refresh_vectors_locked();
          if (!ready_) {
            ready_ = true;
            to_flush.swap(pending_);
          }
        }
        for (const auto & line : to_flush) {publish_line(line);}
      });
  }

  bool wait_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (ready_) {return true;}
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  void send(const std::string & line)
  {
    bool publish_now = false;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      if (!ready_ || !cmd_pub_) {
        pending_.push_back(line);
      } else {
        publish_now = true;
      }
    }
    if (publish_now) {
      publish_line(line);
      std::this_thread::sleep_for(50ms);
    }
  }

  void set_selected(const std::string & value)
  {
    send("set_selected=" + value);
  }

  void set_target_ids(const std::vector<std::string> & ids)
  {
    target_ids = ids;
    std::string joined;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (i) {joined += ",";}
      joined += ids[i];
    }
    send("target_ids=" + joined);
  }

  std::vector<std::string> pose_order()
  {
    return status_string_list("pose_order");
  }

  void apply_scripts()
  {
    if (!target_ids.empty()) {set_target_ids(target_ids);}
    if (target_ids.size() == 2) {
      send("complete_select_seq=peach_2,");
    } else if (on_complete) {
      send("on_complete_clear_selected=1");
    }
    if (clear_handler) {send("clear_fail_first=1");}
    if (cancel_handler) {send("cancel_reject=peach_2");}
    if (goal_handler && finish_script) {
      send("reject_observe_only=1");
    } else if (goal_handler) {
      send("reject_first_n=4");
      send("estop_on_reject=1");
    }
    if (finish_script) {
      std::atomic<int64_t> delay{300};
      std::atomic<uint8_t> outcome{0};
      finish_script(1, "peach_1", delay, outcome);
      if (outcome.load() == RunTargetCycle::Result::SKIPPED_QUALITY) {
        send("first_outcome_quality=1");
      }
      delay.store(300);
      finish_script(1, "peach_2", delay, outcome);
      if (delay.load() == 3000) {
        send("peach2_delay_ms=3000");
        send("finish_delay_ms=10000");
      }
    }
  }

  int status_int(const char * key) const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = status_json_.find(needle);
    if (pos == std::string::npos) {return 0;}
    return std::atoi(status_json_.c_str() + pos + needle.size());
  }

private:
  void publish_line(const std::string & line)
  {
    std_msgs::msg::String message;
    message.data = line;
    cmd_pub_->publish(message);
  }

  void refresh_vectors_locked()
  {
    cycle_ids = status_string_list_locked("cycle_ids");
    goal_modes.clear();
    const std::string needle = "\"goal_modes\":[";
    const auto pos = status_json_.find(needle);
    if (pos == std::string::npos) {return;}
    auto cursor = pos + needle.size();
    while (cursor < status_json_.size() && status_json_[cursor] != ']') {
      if (std::isdigit(static_cast<unsigned char>(status_json_[cursor]))) {
        goal_modes.push_back(static_cast<uint8_t>(
            std::atoi(status_json_.c_str() + cursor)));
        while (cursor < status_json_.size() &&
          std::isdigit(static_cast<unsigned char>(status_json_[cursor])))
        {
          ++cursor;
        }
      } else {
        ++cursor;
      }
    }
  }

  std::vector<std::string> status_string_list(const char * key)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_string_list_locked(key);
  }

  std::vector<std::string> status_string_list_locked(const char * key) const
  {
    std::vector<std::string> out;
    const std::string needle = std::string("\"") + key + "\":[";
    const auto pos = status_json_.find(needle);
    if (pos == std::string::npos) {return out;}
    auto cursor = pos + needle.size();
    while (cursor < status_json_.size() && status_json_[cursor] != ']') {
      if (status_json_[cursor] == '"') {
        const auto end = status_json_.find('"', cursor + 1);
        if (end == std::string::npos) {break;}
        out.push_back(status_json_.substr(cursor + 1, end - cursor - 1));
        cursor = end + 1;
      } else {
        ++cursor;
      }
    }
    return out;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  mutable std::mutex status_mutex_;
  std::string status_json_;
  std::vector<std::string> pending_;
  bool ready_{false};
};

class FakeChild
{
public:
  FakeChild()
  {
    pid_ = fork();
    if (pid_ == 0) {
      execl("/usr/bin/python3", "python3", FAKE_CAPABILITY_PY, nullptr);
      std::_Exit(127);
    }
  }
  ~FakeChild()
  {
    if (pid_ <= 0) {return;}
    kill(pid_, SIGTERM);
    for (int i = 0; i < 20; ++i) {
      if (waitpid(pid_, nullptr, WNOHANG) == pid_) {return;}
      std::this_thread::sleep_for(50ms);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
  }
  bool started() const {return pid_ > 0;}

private:
  pid_t pid_{-1};
};

struct Harness
{
  // extra_params：各用例差异参数（如 execution_enabled / 熔断上限）。
  void start(const std::vector<rclcpp::Parameter> & extra_params)
  {
    // 快进参数（协议行为不变，仅缩短冷却/熔断/超时墙钟）；默认值即
    // config/orchestrator.yaml 权威值，这里只覆盖测试需要加速的键。
    std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("photo_pose.enabled", false),
      rclcpp::Parameter("harvest.rescan_until_empty", false),
      rclcpp::Parameter("harvest.stall_timeout_s", 60.0),
      // 就位自校（F3）默认关：假现场无 TF 外参链发布端，开着会把全部用例
      // 卡在拍照前置前；专门用例经 extra_params 置 true 并自广播静态 TF。
      rclcpp::Parameter("harvest.preflight_check", false),
      rclcpp::Parameter("dispatch.retry_delay_s", 0.5),
      rclcpp::Parameter("advance.retry_delay_s", 0.2),
      rclcpp::Parameter("advance.timeout_s", 1.0),
      rclcpp::Parameter("advance.max_retries", 3),
    };
    overrides.insert(overrides.end(), extra_params.begin(), extra_params.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    fake_child = std::make_unique<FakeChild>();
    orchestrator =
      peach_harvest_orchestrator::make_orchestrator_node_for_test(options);
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 8);
    executor->add_node(orchestrator->get_node_base_interface());
    observer.start(*executor);
    field.bind(observer.node);
    spin_thread = std::thread([this]() {executor->spin();});
    field.wait_ready(10s);
    field.apply_scripts();
    control_client = observer.node->create_client<ControlHarvest>(
      "/peach_harvest_orchestrator/control");
    run_client = rclcpp_action::create_client<RunHarvest>(
      observer.node, "/peach_harvest_orchestrator/run_harvest");
  }

  // 生命周期转换直调（同 test_lifecycle；configure 装配接口，activate 开闸）。
  // LifecycleNode::configure()/activate() 返回转换后的状态（非 CallbackReturn）。
  void configure_activate()
  {
    orchestrator->configure();
    ASSERT_EQ(
      orchestrator->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    orchestrator->activate();
    ASSERT_EQ(
      orchestrator->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(control_client->wait_for_service(5s));
  }

  void stop()
  {
    if (!executor) {return;}
    fake_child.reset();
    executor->cancel();
    if (spin_thread.joinable()) {spin_thread.join();}
    executor.reset();
    orchestrator.reset();
  }

  // 环境自检：假能力端 action 服务可达性。2026-08-18 本机出现 FastDDS 层面
  // 异常：非框架（参数/生命周期内建）服务端点无法完成匹配（图可见但
  // matched_count=0，请求不到达；普通节点/LifecycleNode、进程内/跨进程、
  // cyclonedds/zenoh 均复现；当日内核升级 6.8.0-138 未重启，疑为根因）。
  // 真实节点 on_configure 装配的端点不受影响（test_lifecycle/policy 全绿）。
  // 异常窗口内假能力端不可达，依赖它的用例按 GTEST_SKIP 跳过而非误判失败。
  bool capability_end_reachable()
  {
    auto probe = rclcpp_action::create_client<RunTargetCycle>(
      observer.node, "/peach_approach_grasp_node/run_target_cycle");
    return probe->wait_for_action_server(10s);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> orchestrator;
  FakeRemote field;
  std::unique_ptr<FakeChild> fake_child;
  Observer observer;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spin_thread;
  rclcpp::Client<ControlHarvest>::SharedPtr control_client;
  rclcpp_action::Client<RunHarvest>::SharedPtr run_client;
};

class DispatchProtocolTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override
  {
    harness.stop();
    rclcpp::shutdown();
  }

  Harness harness;
};

// 协议 2.6 推进协议 + 2.5-L4-(c)/(e) 清除点（R1 回归）。
TEST_F(DispatchProtocolTest, AdvanceFailsRetryThenFuseAndRecoveryAllowsRedispatch)
{
  harness.field.set_selected("peach_1");
  harness.field.complete_success.store(false);
  harness.start({rclcpp::Parameter("execution_enabled", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  // 熔断：首个周期 SUCCEEDED 后推进连续失败 advance.max_retries(3) 次。
  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.state().recovery_required;}, 40s))
    << "推进持续失败未熔断进 RECOVERY_REQUIRED";
  EXPECT_EQ(harness.field.goals_received.load(), 1);
  EXPECT_EQ(harness.observer.count_event("target_succeeded"), 1);
  // 每次失败一条 ERROR 事件：前 2 次冷却重试 + 第 3 次熔断。
  EXPECT_EQ(harness.observer.count_event("advance_retry"), 2);
  EXPECT_EQ(harness.observer.count_event("advance_failed"), 1);
  EXPECT_TRUE(harness.observer.any_event_message_contains("感知推进持续失败"));

  // 恢复：ACKNOWLEDGE_RECOVERY（仅清标志保持暂停）+ RESUME；推进修好后
  // 同一 selected 必须能重新派发（清除点 (c)/(e) 生效，R1 不再死锁）。
  harness.field.complete_success.store(true);
  ASSERT_TRUE(send_control(
      harness.control_client, harness.observer,
      ControlHarvest::Request::ACKNOWLEDGE_RECOVERY, "ack"));
  ASSERT_TRUE(send_control(
      harness.control_client, harness.observer,
      ControlHarvest::Request::RESUME, "resume"));
  ASSERT_TRUE(wait_for(
      [this]() {return harness.field.goals_received.load() >= 2;}, 30s))
    << "熔断恢复后未能重新派发（R1 回归）";
  // 第二次推进成功：批次保持运行，不再进恢复。
  std::this_thread::sleep_for(3s);
  const auto state = harness.observer.state();
  EXPECT_FALSE(state.recovery_required);
  EXPECT_EQ(state.batch_state, HarvestState::RUNNING);
}

// 协议 2.2-T9/2.9：CANCEL_NOW 终结 RunHarvest（canceled）且新 goal 可受理。
TEST_F(DispatchProtocolTest, CancelNowTerminatesRunHarvestAndAcceptsNewGoal)
{
  harness.field.set_selected("peach_1");
  // execution 关闭：不派发任何单目标周期，纯验 RunHarvest 生命周期。
  harness.start({rclcpp::Parameter("execution_enabled", false)});
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }
  harness.configure_activate();
  ASSERT_TRUE(harness.run_client->wait_for_action_server(5s));

  std::atomic<bool> first_done{false};
  std::atomic<int> first_code{-1};
  RunHarvest::Goal goal;
  goal.request_id = "cancel-now-run-1";
  goal.target_source = RunHarvest::Goal::AUTO_CATALOG;
  rclcpp_action::Client<RunHarvest>::SendGoalOptions options;
  options.result_callback =
    [&first_done, &first_code](const auto & wrapped) {
      first_code.store(static_cast<int>(wrapped.code));
      first_done.store(true);
    };
  const auto goal_handle_future = harness.run_client->async_send_goal(goal, options);
  ASSERT_EQ(goal_handle_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(goal_handle_future.get() != nullptr) << "首个 RunHarvest goal 被拒";

  ASSERT_TRUE(send_control(
      harness.control_client, harness.observer,
      ControlHarvest::Request::CANCEL_NOW, "cancel-now"));
  ASSERT_TRUE(wait_for([&first_done]() {return first_done.load();}, 10s))
    << "CANCEL_NOW 后 RunHarvest 未终结（空转占用单 goal 守卫）";
  EXPECT_EQ(
    first_code.load(), static_cast<int>(rclcpp_action::ResultCode::CANCELED));
  EXPECT_EQ(harness.observer.count_event("batch_canceled"), 1);

  // T9 后半：canceled 终结后新 goal 可受理（INTERRUPTED 终局批次豁免
  // mode 门，accepted_callback 走 reset_batch 清账重开）。
  RunHarvest::Goal second;
  second.request_id = "cancel-now-run-2";
  second.target_source = RunHarvest::Goal::AUTO_CATALOG;
  const auto second_future = harness.run_client->async_send_goal(second);
  ASSERT_EQ(second_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(second_future.get() != nullptr)
    << "CANCEL_NOW 终结后新 RunHarvest goal 仍被拒";
}

// 协议 2.5-R-2（R3 修复）：失能窗口内的 goal 拒绝只冷却重试，不进熔断链。
TEST_F(DispatchProtocolTest, RejectionWhileMotionNotReadyNeverFuses)
{
  harness.field.set_selected("peach_1");
  // 前 4 个 goal：假能力端先制造本端急停（robot_status 失能）再拒绝——
  // 编排器处理拒绝时本端 motion 不就绪，必须只冷却重试不计熔断；
  // 第 5 个 goal 受理并成功。max_consecutive_rejections=3 下，若失能拒绝
  // 被误计数，第 3 次拒绝即熔断（本用例的回归判据）。
  harness.field.goal_handler =
    [this](int index, const RunTargetCycle::Goal &) {
      if (index <= 4) {
        harness.field.estop.store(true);
        // 等编排器 robot_callback 处理急停帧（100ms 周期）后再拒绝。
        std::this_thread::sleep_for(300ms);
        harness.field.estop.store(false);
        return false;
      }
      return true;
    };
  harness.start({
      rclcpp::Parameter("execution_enabled", true),
      rclcpp::Parameter("dispatch.max_consecutive_rejections", 3)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.field.goals_received.load() >= 5 &&
               harness.observer.count_event("target_succeeded") == 1;
      }, 60s))
    << "失能拒绝序列后第 5 次派发未成功";
  EXPECT_FALSE(harness.observer.recovery_seen)
    << "失能窗口的拒绝被误计入熔断（R3 回归）";
  EXPECT_EQ(harness.observer.count_event("target_dispatch_retry"), 4);
  EXPECT_EQ(harness.observer.count_event("target_rejected"), 0);
  EXPECT_TRUE(harness.observer.any_event_message_contains("失能"));
}

// 协议 2.9/2.6-2：skip 绑 cycle_id；取消未落地的 skip 不得误记操作员跳过。
TEST_F(DispatchProtocolTest, OperatorSkipBindsCycleId)
{
  harness.field.target_ids = {"peach_1", "peach_2"};
  harness.field.set_selected("peach_1");
  // 推进副作用：第 1 次推进后 selected 切到 peach_2，第 2 次后清空（收口）。
  harness.field.on_complete = [this](int call) {
      if (call == 1) {
        harness.field.set_selected("peach_2");
      } else {
        harness.field.set_selected("");
      }
    };
  // peach_1 给足取消窗口（10s 自然终局，SKIP 取消先行落地）；peach_2
  // 3s 自然成功且拒绝取消——skip 已请求但取消未落地，不得按操作员跳过记账。
  harness.field.finish_script =
    [](int, const std::string & target_id,
    std::atomic<int64_t> & delay_ms, std::atomic<uint8_t> &) {
      delay_ms.store(target_id == "peach_2" ? 3000 : 10000);
    };
  harness.field.cancel_handler = [](const std::string & target_id) {
      return target_id != "peach_2";
    };
  harness.start({rclcpp::Parameter("execution_enabled", true)});
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  // configure 后即挂 RunHarvest（activate 前批次未起跑，goal 必受理），
  // 避免与单目标周期争抢 target_active 窗口。
  harness.orchestrator->configure();
  ASSERT_EQ(
    harness.orchestrator->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::this_thread::sleep_for(1500ms);
  std::atomic<bool> run_done{false};
  std::atomic<int> run_code{-1};
  peach_harvest_msgs::msg::HarvestSummary summary;
  RunHarvest::Goal goal;
  goal.request_id = "skip-binding-run";
  goal.target_source = RunHarvest::Goal::AUTO_CATALOG;
  rclcpp_action::Client<RunHarvest>::SendGoalOptions options;
  options.result_callback =
    [&run_done, &run_code, &summary](const auto & wrapped) {
      run_code.store(static_cast<int>(wrapped.code));
      if (wrapped.result) {summary = wrapped.result->summary;}
      run_done.store(true);
    };
  ASSERT_TRUE(harness.run_client->wait_for_action_server(5s));
  const auto goal_future = harness.run_client->async_send_goal(goal, options);
  ASSERT_EQ(goal_future.wait_for(5s), std::future_status::ready);
  ASSERT_TRUE(goal_future.get() != nullptr) << "RunHarvest goal 被拒";
  harness.orchestrator->activate();
  ASSERT_EQ(
    harness.orchestrator->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(harness.control_client->wait_for_service(5s));

  const auto target_active = [this](const std::string & id) {
      const auto state = harness.observer.state();
      return state.target_id == id &&
             state.target_phase != HarvestState::TARGET_IDLE;
    };
  ASSERT_TRUE(wait_for([&target_active]() {return target_active("peach_1");}, 30s))
    << "peach_1 未被派发";
  ASSERT_TRUE(send_control(
      harness.control_client, harness.observer,
      ControlHarvest::Request::SKIP_TARGET, "skip-1"));

  ASSERT_TRUE(wait_for([&target_active]() {return target_active("peach_2");}, 30s))
    << "操作员跳过后未推进到 peach_2";
  ASSERT_TRUE(send_control(
      harness.control_client, harness.observer,
      ControlHarvest::Request::SKIP_TARGET, "skip-2"));

  // 两目标处理完 + selected 清空 → 批次 COMPLETED → RunHarvest succeed。
  ASSERT_TRUE(wait_for([&run_done]() {return run_done.load();}, 40s))
    << "批次未按预期完成";
  EXPECT_EQ(run_code.load(), static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED));
  // 账本：peach_1 操作员跳过（CANCELED），peach_2 自然成功——skip 绑定
  // 不得把取消未落地的周期误记为操作员跳过。
  EXPECT_EQ(summary.attempted, 2u);
  EXPECT_EQ(summary.canceled, 1u);
  EXPECT_EQ(summary.succeeded, 1u);
  EXPECT_EQ(harness.observer.count_event("target_operator_skipped"), 1);
  EXPECT_EQ(harness.observer.count_event("target_succeeded"), 1);
  EXPECT_EQ(harness.observer.count_event("target_canceled"), 0);
}
// 协议 2.3 编排侧监督 + R-D8 发现进度透出：收齐窗口迟迟不锁定超
// collect_timeout_s 时发 round_stall 并按本轮处理完重判（首轮 0 目标
// → COMPLETE，不无限等）；未锁定时 message 透出"收齐中 N 目标/M 确认中"。
TEST_F(DispatchProtocolTest, CollectTimeoutRejudgesRoundAndReportsProgress)
{
  harness.field.targets_locked_flag.store(false);
  harness.field.collecting_count.store(2);
  harness.field.pending_count.store(1);
  harness.start({
      rclcpp::Parameter("execution_enabled", false),
      rclcpp::Parameter("harvest.collect_timeout_s", 2.0)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  // R-D8：DISCOVERY 未锁定时 message 透出收齐进度摘要。
  ASSERT_TRUE(wait_for(
      [this]() {
        const auto state = harness.observer.state();
        return state.batch_state == HarvestState::DISCOVERY &&
               state.message.find("收齐中 2 目标/1 确认中") != std::string::npos;
      }, 15s))
    << "DISCOVERY 未锁定时 message 未透出收齐进度";

  // 2.3 监督：2s 超时 → round_stall WARNING + 按本轮处理完重判
  // （首轮 last_target_count_=0 → COMPLETE"本轮未锁定到目标"）。
  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.observer.count_event("round_stall") == 1 &&
               harness.observer.state().batch_state == HarvestState::COMPLETED;
      }, 20s))
    << "收齐窗口超时未触发重判收口（批次应落 COMPLETED）";
  EXPECT_EQ(harness.observer.count_event("batch_completed"), 1);
  EXPECT_TRUE(harness.observer.any_event_message_contains("收齐窗口超时"));
  EXPECT_TRUE(harness.observer.any_event_message_contains("本轮未锁定到目标"));
}

// 协议 2.5 goal 契约：cycle_id = <run_id>-cycle-<批次内单调序号>；auto_start
// 自动批次（无 RunHarvest goal）run_id 生成 auto-<时间戳>，同批次两周期
// cycle_id 单调不重复。
TEST_F(DispatchProtocolTest, CycleIdsAreUniqueAndCarryRunId)
{
  harness.field.target_ids = {"peach_1", "peach_2"};
  harness.field.set_selected("peach_1");
  // 推进副作用：第 1 次推进后 selected 切 peach_2，第 2 次后清空（收口）。
  harness.field.on_complete = [this](int call) {
      if (call == 1) {
        harness.field.set_selected("peach_2");
      } else {
        harness.field.set_selected("");
      }
    };
  harness.start({rclcpp::Parameter("execution_enabled", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.field.goals_received.load() >= 2 &&
               harness.observer.count_event("target_succeeded") == 2;
      }, 60s))
    << "两个目标未依次派发成功";

  std::vector<std::string> cycle_ids;
  {
    std::lock_guard<std::mutex> lock(harness.field.cycle_ids_mutex);
    cycle_ids = harness.field.cycle_ids;
  }
  ASSERT_EQ(cycle_ids.size(), 2u);
  EXPECT_NE(cycle_ids[0], cycle_ids[1]);
  // auto 批次 run_id 唯一前缀：auto-<纳秒时间戳>
  const std::string run_id = harness.observer.state().run_id;
  ASSERT_TRUE(run_id.rfind("auto-", 0) == 0) << "auto 批次 run_id 缺失: " << run_id;
  EXPECT_EQ(cycle_ids[0], run_id + "-cycle-1");
  EXPECT_EQ(cycle_ids[1], run_id + "-cycle-2");
}

// 挂 RunHarvest goal 并捕获终局 HarvestSummary（丢失记账对账用）。
bool send_run_goal(
  Harness & harness, const std::string & request_id,
  std::atomic<bool> & run_done, peach_harvest_msgs::msg::HarvestSummary & summary)
{
  RunHarvest::Goal goal;
  goal.request_id = request_id;
  goal.target_source = RunHarvest::Goal::AUTO_CATALOG;
  rclcpp_action::Client<RunHarvest>::SendGoalOptions options;
  options.result_callback =
    [&run_done, &summary](const auto & wrapped) {
      if (wrapped.result) {summary = wrapped.result->summary;}
      run_done.store(true);
    };
  if (!harness.run_client->wait_for_action_server(5s)) {return false;}
  auto future = harness.run_client->async_send_goal(goal, options);
  return future.wait_for(5s) == std::future_status::ready && future.get() != nullptr;
}

// 协议 2.4（阶段 D2）：锁定集目标未派发即从锁定集消失（感知 anchor_drop）
// → 记 SKIPPED_UNREACHABLE（reason 目标丢失超时）+ AUDIT 事件
// target_dropped，进 HarvestSummary；本帧 target_count 同步递减，批次按
// "本轮全部处理完"正常收口（decide_round 口径兼容）。
TEST_F(DispatchProtocolTest, DroppedTargetBookedAsSkippedUnreachable)
{
  harness.field.set_target_ids({"peach_1", "peach_2"});
  harness.field.set_selected("");
  harness.start({rclcpp::Parameter("execution_enabled", false)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("round_locked") == 1;}, 20s))
    << "首轮锁定沿未到达";
  std::atomic<bool> run_done{false};
  peach_harvest_msgs::msg::HarvestSummary summary;
  ASSERT_TRUE(send_run_goal(harness, "drop-booking-run", run_done, summary))
    << "RunHarvest goal 被拒";

  // 模拟 anchor_drop：peach_2 未派发即从锁定集消失。
  harness.field.set_target_ids({"peach_1"});
  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("target_dropped") == 1;}, 20s))
    << "锁定集目标消失未记 target_dropped";
  // processed(1) == target_count(1) 且复扫关闭 → 批次收口。
  ASSERT_TRUE(wait_for([&run_done]() {return run_done.load();}, 30s))
    << "丢失记账后批次未收口";
  EXPECT_EQ(summary.attempted, 1u);
  EXPECT_EQ(summary.skipped_unreachable, 1u);
  ASSERT_EQ(summary.outcomes.size(), 1u);
  EXPECT_EQ(summary.outcomes[0].target_id, "peach_2");
  EXPECT_EQ(summary.outcomes[0].outcome, TargetOutcomeMsg::SKIPPED_UNREACHABLE);
  EXPECT_NE(summary.outcomes[0].reason.find("目标丢失超时"), std::string::npos);
}

// 误记账防御（协议 2.4）：reset/收齐重开（解锁→换集→再锁定）属整集重置
// 语义，旧集目标不得记丢失；再锁定后新集目标消失仍正常记账。
TEST_F(DispatchProtocolTest, ResetAndRelockDoesNotBookDroppedTargets)
{
  harness.field.set_target_ids({"peach_1", "peach_2"});
  harness.field.set_selected("");
  harness.start({rclcpp::Parameter("execution_enabled", false)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("round_locked") == 1;}, 20s));
  std::atomic<bool> run_done{false};
  peach_harvest_msgs::msg::HarvestSummary summary;
  ASSERT_TRUE(send_run_goal(harness, "reset-defence-run", run_done, summary))
    << "RunHarvest goal 被拒";

  // 模拟 reset_global_targets 生效：解锁若干帧（收齐重开窗口，丢失跟踪集
  // 应整体清册），随后换集再锁定（复扫新一轮语义）。
  harness.field.targets_locked_flag.store(false);
  std::this_thread::sleep_for(1200ms);
  harness.field.set_target_ids({"peach_3"});
  harness.field.targets_locked_flag.store(true);
  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("round_locked") == 2;}, 20s))
    << "再锁定沿未到达";
  EXPECT_EQ(harness.observer.count_event("target_dropped"), 0)
    << "reset/轮次切换路径误记丢失目标";

  // 再锁定后的新集目标消失仍正常记账（跟踪集已按新集重建）。
  harness.field.set_target_ids({});
  ASSERT_TRUE(wait_for(
      [&run_done, this]() {
        return harness.observer.count_event("target_dropped") == 1 && run_done.load();
      }, 30s))
    << "新集目标消失未记账或批次未收口";
  EXPECT_EQ(summary.attempted, 1u);
  EXPECT_EQ(summary.skipped_unreachable, 1u);
  ASSERT_EQ(summary.outcomes.size(), 1u);
  EXPECT_EQ(summary.outcomes[0].target_id, "peach_3");
}

// 活动周期目标中途消失（协议 2.4 防御）：丢失记账让位 result_callback
// 终局账，不双记——target_dropped=0，周期按 SUCCEEDED 入账一次。
TEST_F(DispatchProtocolTest, ActiveTargetVanishIsNotDoubleBooked)
{
  harness.field.set_target_ids({"peach_1"});
  harness.field.set_selected("peach_1");
  harness.field.on_complete = [this](int) {harness.field.set_selected("");};
  // 拉长周期终局，留出中途移除锁定集的窗口。
  harness.field.finish_script =
    [](int, const std::string &,
    std::atomic<int64_t> & delay_ms, std::atomic<uint8_t> &) {
      delay_ms.store(3000);
    };
  harness.start({rclcpp::Parameter("execution_enabled", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  std::atomic<bool> run_done{false};
  peach_harvest_msgs::msg::HarvestSummary summary;
  ASSERT_TRUE(send_run_goal(harness, "vanish-defence-run", run_done, summary))
    << "RunHarvest goal 被拒";
  ASSERT_TRUE(wait_for(
      [this]() {return harness.field.goals_received.load() >= 1;}, 30s))
    << "peach_1 未被派发";
  // 活动周期中途目标从锁定集消失（anchor_drop）：丢失记账必须让位终局账。
  harness.field.set_target_ids({});
  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("target_succeeded") == 1;}, 30s))
    << "活动周期未成功终局";
  EXPECT_EQ(harness.observer.count_event("target_dropped"), 0)
    << "活动目标被丢失记账双记";
  ASSERT_TRUE(wait_for([&run_done]() {return run_done.load();}, 30s))
    << "批次未收口";
  EXPECT_EQ(summary.attempted, 1u);
  EXPECT_EQ(summary.succeeded, 1u);
  EXPECT_EQ(summary.skipped_unreachable, 0u);
}

// 协议 2.3（阶段 D2）：harvest.fresh_scene=true 时拍照前置在
// reset_global_targets 之前调 clear_target_memory；首次拒绝走 photo_step
// 冷却重试链（与 reset 失败同级），最终顺序 clear→clear(重试)→reset。
TEST_F(DispatchProtocolTest, FreshSceneClearsMemoryBeforeReset)
{
  harness.field.set_target_ids({"peach_1"});
  harness.field.set_selected("");
  // 首次清记忆被拒，重试成功（脚本按调用序判定）。
  harness.field.clear_handler = [](int call) {return call > 1;};
  harness.start({
      rclcpp::Parameter("execution_enabled", true),
      rclcpp::Parameter("photo_pose.enabled", true),
      rclcpp::Parameter("photo_pose.retry_cooldown_s", 0.5),
      rclcpp::Parameter("harvest.fresh_scene", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("round_started") == 1;}, 40s))
    << "拍照前置未完成（fresh_scene 清记忆链）";
  EXPECT_EQ(harness.field.clear_calls.load(), 2);
  EXPECT_EQ(harness.field.reset_calls.load(), 1);
  EXPECT_EQ(
    harness.field.pose_order(), (std::vector<std::string>{"clear", "clear", "reset"}))
    << "fresh_scene 调用链应为 clear→(冷却重试)→reset";
  EXPECT_EQ(harness.observer.count_event("photo_step_retry"), 1);
  EXPECT_EQ(harness.observer.count_event("target_memory_cleared"), 1);
  EXPECT_TRUE(
    harness.observer.any_event_message_contains("clear_target_memory 被拒绝"));
}

// fresh_scene=false（默认）：拍照前置不调 clear_target_memory，顺序仅 reset。
TEST_F(DispatchProtocolTest, FreshSceneDisabledSkipsClearMemory)
{
  harness.field.set_target_ids({"peach_1"});
  harness.field.set_selected("");
  harness.start({
      rclcpp::Parameter("execution_enabled", true),
      rclcpp::Parameter("photo_pose.enabled", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {return harness.observer.count_event("round_started") == 1;}, 40s))
    << "拍照前置未完成";
  EXPECT_EQ(harness.field.clear_calls.load(), 0);
  EXPECT_EQ(harness.field.reset_calls.load(), 1);
  EXPECT_EQ(harness.field.pose_order(), (std::vector<std::string>{"reset"}));
}
// 协议 2.8 残局抬质量（阶段 E3）主流：一轮正常派发耗尽后，SKIPPED_QUALITY
// 残局目标触发 mode=OBSERVE_ONLY 派发；抬质量成功（感知重新可选）后回正常
// 派发链以 FULL 重试；observe 终局只进审计账（attempted/succeeded 不计，
// reason 标 observe_retry）；全程无 round_stall/提前收口（stall 协同：
// 有残局目标在处理不算停滞）。
TEST_F(DispatchProtocolTest, ObserveRetryEndgameLiftsQualityThenRetriesFull)
{
  harness.field.set_target_ids({"peach_1"});
  harness.field.set_selected("peach_1");
  // goal#1（FULL）质量门未过 → SKIPPED_QUALITY；goal#2（OBSERVE_ONLY）与
  // goal#3（FULL 重试）走默认 300ms SUCCEEDED。
  harness.field.finish_script =
    [](int index, const std::string &,
    std::atomic<int64_t> &, std::atomic<uint8_t> & outcome) {
      if (index == 1) {outcome.store(RunTargetCycle::Result::SKIPPED_QUALITY);}
    };
  // FULL 终局推进感知计划：selected 清空进入残局；第二次推进（FULL 重试
  // 成功后）保持空，批次收口。
  harness.field.on_complete = [this](int) {harness.field.set_selected("");};
  harness.start({rclcpp::Parameter("execution_enabled", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  // 残局形成：goal#1 SKIPPED_QUALITY + 推进后 selected 空 → OBSERVE_ONLY 派发。
  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.field.goals_received.load() >= 2 &&
               harness.observer.count_event("observe_retry_finished") == 1;
      }, 40s))
    << "残局目标未触发 OBSERVE_ONLY 抬质量或抬质量未终局";
  // 抬质量成功 → 感知重新可选（质量改善）：回正常链 FULL 重试。
  harness.field.set_selected("peach_1");
  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.field.goals_received.load() >= 3 &&
               harness.observer.count_event("target_succeeded") == 1;
      }, 40s))
    << "抬质量成功后未回正常派发链 FULL 重试";

  // 批次收口（复扫关闭，attempted=2 ≥ 锁定数 1 → COMPLETED）。
  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.observer.count_event("batch_completed") == 1;
      }, 30s))
    << "批次未按预期完成";

  // goal 序列与 mode：FULL → OBSERVE_ONLY → FULL。
  std::vector<uint8_t> modes;
  {
    std::lock_guard<std::mutex> lock(harness.field.cycle_ids_mutex);
    modes = harness.field.goal_modes;
  }
  ASSERT_EQ(modes.size(), 3u);
  EXPECT_EQ(modes[0], RunTargetCycle::Goal::FULL);
  EXPECT_EQ(modes[1], RunTargetCycle::Goal::OBSERVE_ONLY);
  EXPECT_EQ(modes[2], RunTargetCycle::Goal::FULL);
  EXPECT_EQ(harness.observer.count_event("observe_retry_dispatched"), 1);

  // stall 协同：残局抬质量期间不算停滞、不提前收口（无 round_stall）。
  EXPECT_EQ(harness.observer.count_event("round_stall"), 0);
  EXPECT_FALSE(harness.observer.recovery_seen);

  // 记账：observe 终局进审计账但不重复计为采摘成功/失败——attempted=2
  // （FULL + FULL 重试），succeeded=1、skipped_quality=1；outcomes 三条，
  // 中间一条为 observe_retry 审计条目（本用例未挂 RunHarvest，账后经
  // 事件流水对账：target_skipped 仅 FULL 那次一条）。
  EXPECT_EQ(harness.observer.count_event("target_skipped"), 1);
  // HarvestSummary 由 RunHarvest 终局携带；本用例为 auto_start 批次，
  // 改验状态投影：批次已完成且进度收满（attempted=2/发现=1 钳到 1）。
  const auto state = harness.observer.state();
  EXPECT_EQ(state.batch_state, HarvestState::COMPLETED);
  EXPECT_EQ(state.progress, 1.0f);
}

// 协议 2.8/2.5 协同（E3）：observe-retry goal 被拒是 best-effort——不记账、
// 不进连续拒绝熔断、不耗重试预算、不推进感知计划；次数派发时已消耗，
// 本轮去重保证系统性拒绝下有界收口（直接进轮次判定完成批次）。
TEST_F(DispatchProtocolTest, ObserveRetryRejectionIsBoundedAndDoesNotFuse)
{
  harness.field.set_target_ids({"peach_1"});
  harness.field.set_selected("peach_1");
  harness.field.finish_script =
    [](int index, const std::string &,
    std::atomic<int64_t> &, std::atomic<uint8_t> & outcome) {
      if (index == 1) {outcome.store(RunTargetCycle::Result::SKIPPED_QUALITY);}
    };
  harness.field.on_complete = [this](int) {harness.field.set_selected("");};
  // 假能力端拒一切 OBSERVE_ONLY（模拟真机能力端缓存仍以 selected 为准的
  // 系统性拒绝）；熔断上限压到 2——若 observe 拒绝被误计入熔断计数，
  // 一次拒绝即占半额，本用例以 recovery 永不出现为回归判据。
  harness.field.goal_handler =
    [](int, const RunTargetCycle::Goal & goal) {
      return goal.mode != RunTargetCycle::Goal::OBSERVE_ONLY;
    };
  harness.start({
      rclcpp::Parameter("execution_enabled", true),
      rclcpp::Parameter("dispatch.max_consecutive_rejections", 2)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.observer.count_event("observe_retry_rejected") == 1;
      }, 40s))
    << "OBSERVE_ONLY goal 被拒未走 best-effort 分支";
  // 拒绝后本轮无候选（次数已消耗+本轮去重）→ 直接轮次判定收口。
  ASSERT_TRUE(wait_for(
      [this]() {
        return harness.observer.count_event("batch_completed") == 1;
      }, 30s))
    << "observe-retry 拒绝后批次未有界收口";
  EXPECT_EQ(harness.field.goals_received.load(), 2);
  EXPECT_FALSE(harness.observer.recovery_seen)
    << "observe-retry 拒绝被误计入连续拒绝熔断";
  EXPECT_EQ(harness.observer.count_event("advance_retry"), 0)
    << "observe-retry 拒绝不得推进感知计划";
  EXPECT_EQ(harness.observer.count_event("target_rejected"), 0);
}

// 批次启动就位自校（阶段 F3，harvest.preflight_check）：四路就绪全绿但
// TF 外参链（base_link→camera_link）未就位时，批次不得进入拍照前置——
// state blockers 带 preflight、发 ERROR 事件、零派发；外参链就位（广播
// 静态 TF）后自动放行（幂等门，非一次性熔断），批次照常推进。
TEST_F(DispatchProtocolTest, PreflightBlocksPhotoChainUntilGeometryReady)
{
  harness.field.set_selected("peach_1");
  harness.start({
      rclcpp::Parameter("execution_enabled", true),
      rclcpp::Parameter("harvest.preflight_check", true)});
  harness.configure_activate();
  if (!harness.capability_end_reachable()) {
    GTEST_SKIP() << "假能力端 action 服务不可达：本机 DDS 端点匹配异常"
      "（见 Harness::capability_end_reachable 注释），跳过";
  }

  // 自校失败挂起：blockers 出现 preflight，ERROR 事件列清失败项，
  // 且挂起期间零派发（拍照前置未发起，selected 在也不派）。
  const auto blocked = [this]() {
      const auto blockers = harness.observer.state().blockers;
      return std::find(blockers.begin(), blockers.end(), "preflight") !=
             blockers.end();
    };
  ASSERT_TRUE(wait_for(blocked, 30s))
    << "TF 外参链未就位时就位自校未阻断批次（blockers 无 preflight）";
  EXPECT_GE(harness.observer.count_event("preflight_failed"), 1);
  EXPECT_TRUE(
    harness.observer.any_event_message_contains("base_link→camera_link"));
  std::this_thread::sleep_for(2s);
  EXPECT_EQ(harness.field.goals_received.load(), 0)
    << "就位自校未通过时不应派发任何目标";

  // 环境就位（广播静态外参 TF）：幂等门下拍重试自动放行，批次照常推进
  // 到首个目标派发。
  tf2_ros::StaticTransformBroadcaster tf_broadcaster(harness.observer.node);
  geometry_msgs::msg::TransformStamped extrinsics;
  extrinsics.header.stamp = harness.observer.node->now();
  extrinsics.header.frame_id = "base_link";
  extrinsics.child_frame_id = "camera_link";
  extrinsics.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(extrinsics);
  ASSERT_TRUE(wait_for(
      [this]() {return harness.field.goals_received.load() >= 1;}, 30s))
    << "TF 外参链就位后就位自校未放行（批次未恢复派发）";
  EXPECT_EQ(harness.observer.count_event("preflight_passed"), 1);
  // 恢复后 blockers 不再带 preflight。
  ASSERT_TRUE(wait_for(
      [&blocked]() {return !blocked();}, 10s))
    << "就位自校通过后 blockers 仍挂 preflight";
  EXPECT_FALSE(harness.observer.recovery_seen);
}
}  // namespace
