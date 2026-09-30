// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "peach2_task/ros/ros_context.hpp"

/// Non-blocking action / service leaves (BehaviorTree.ROS2 is not available on this host):
/// onStart sends, onRunning polls state written by rclcpp callbacks, onHalted cancels.
/// Callbacks and ticks run on the same single-threaded executor; the mutex only guards
/// against a multi-threaded executor being used by mistake.
namespace peach2_task::ros
{

enum class LeafError : uint8_t
{
  SERVER_UNAVAILABLE,  ///< server not ready within server_wait_s
  REJECTED,            ///< goal rejected by the server
  TIMEOUT,             ///< no result within the leaf timeout (goal cancel requested)
  NO_RESULT,           ///< action finished without a usable result / service call failed
  BAD_REQUEST,         ///< make_goal / make_request refused (invalid ports)
};

const char * leaf_error_name(LeafError error);

inline double seconds_since(SteadyClock::time_point since)
{
  return std::chrono::duration<double>(SteadyClock::now() - since).count();
}

template<class ActionT>
class AsyncActionLeaf : public BT::StatefulActionNode
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;

  AsyncActionLeaf(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr context,
    typename Client::SharedPtr client, double timeout_s)
  : BT::StatefulActionNode(name, config), ctx_(std::move(context)),
    client_(std::move(client)), timeout_s_(timeout_s) {}

  ~AsyncActionLeaf() override {cancel_inflight();}

protected:
  /// False = FAILURE without sending (the override reports why via on_error or the session).
  virtual bool make_goal(Goal * goal) = 0;
  virtual BT::NodeStatus on_result(const WrappedResult & result) = 0;
  virtual void on_error(LeafError error, const std::string & detail) = 0;
  virtual void on_started() {}
  virtual void on_feedback(const Feedback &) {}

  BT::NodeStatus onStart() override
  {
    state_ = std::make_shared<Shared>();
    started_at_ = SteadyClock::now();
    sent_ = false;
    on_started();
    return try_send();
  }

  BT::NodeStatus onRunning() override
  {
    if (!sent_) {
      return try_send();
    }
    std::optional<WrappedResult> result;
    bool rejected = false;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      rejected = state_->rejected;
      if (state_->result) {
        result = state_->result;
      }
      while (!state_->feedback.empty()) {
        on_feedback(state_->feedback.front());
        state_->feedback.erase(state_->feedback.begin());
      }
    }
    if (rejected) {
      sent_ = false;
      on_error(LeafError::REJECTED, "goal rejected");
      return BT::NodeStatus::FAILURE;
    }
    if (result) {
      sent_ = false;
      return on_result(*result);
    }
    if (timeout_s_ > 0.0 && seconds_since(sent_at_) > timeout_s_) {
      cancel_inflight();
      on_error(LeafError::TIMEOUT, "no result within " + std::to_string(timeout_s_) + " s");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {cancel_inflight();}

  RosContextPtr ctx_;

private:
  struct Shared
  {
    std::mutex mutex;
    typename GoalHandle::SharedPtr handle;
    bool rejected = false;
    bool cancel_on_accept = false;
    std::optional<WrappedResult> result;
    std::vector<Feedback> feedback;
  };

  BT::NodeStatus try_send()
  {
    if (!client_ || !client_->action_server_is_ready()) {
      if (seconds_since(started_at_) > ctx_->timeouts.server_wait_s) {
        on_error(LeafError::SERVER_UNAVAILABLE, "action server not available");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    Goal goal;
    if (!make_goal(&goal)) {
      return BT::NodeStatus::FAILURE;
    }
    typename Client::SendGoalOptions options;
    std::weak_ptr<Client> weak_client = client_;
    auto state = state_;
    options.goal_response_callback =
      [state, weak_client](typename GoalHandle::SharedPtr handle) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!handle) {
          state->rejected = true;
          return;
        }
        state->handle = handle;
        if (state->cancel_on_accept) {
          if (auto client = weak_client.lock()) {
            client->async_cancel_goal(handle);
          }
        }
      };
    options.feedback_callback =
      [state](typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> fb) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->feedback.size() < 16U) {
          state->feedback.push_back(*fb);
        }
      };
    options.result_callback = [state](const WrappedResult & result) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->result = result;
      };
    client_->async_send_goal(goal, options);
    sent_ = true;
    sent_at_ = SteadyClock::now();
    return BT::NodeStatus::RUNNING;
  }

  void cancel_inflight()
  {
    if (!sent_ || !state_) {
      return;
    }
    sent_ = false;
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->result || state_->rejected) {
      return;
    }
    if (state_->handle) {
      try {
        client_->async_cancel_goal(state_->handle);
      } catch (const std::exception & e) {
        RCLCPP_WARN(ctx_->logger, "%s: cancel failed: %s", name().c_str(), e.what());
      }
    } else {
      state_->cancel_on_accept = true;
    }
  }

  typename Client::SharedPtr client_;
  double timeout_s_;
  std::shared_ptr<Shared> state_;
  SteadyClock::time_point started_at_;
  SteadyClock::time_point sent_at_;
  bool sent_ = false;
};

template<class SrvT>
class AsyncServiceLeaf : public BT::StatefulActionNode
{
public:
  using Client = rclcpp::Client<SrvT>;
  using Request = typename SrvT::Request;
  using Response = typename SrvT::Response;

  AsyncServiceLeaf(
    const std::string & name, const BT::NodeConfig & config, RosContextPtr context,
    typename Client::SharedPtr client, double timeout_s)
  : BT::StatefulActionNode(name, config), ctx_(std::move(context)),
    client_(std::move(client)), timeout_s_(timeout_s) {}

  ~AsyncServiceLeaf() override {drop_pending();}

protected:
  virtual bool make_request(Request * request) = 0;
  virtual BT::NodeStatus on_response(const Response & response) = 0;
  /// Returned status becomes the leaf status (FAILURE unless the leaf can degrade).
  virtual BT::NodeStatus on_error(LeafError error, const std::string & detail) = 0;
  virtual void on_started() {}

  BT::NodeStatus onStart() override
  {
    state_ = std::make_shared<Shared>();
    started_at_ = SteadyClock::now();
    pending_id_.reset();
    on_started();
    return try_send();
  }

  BT::NodeStatus onRunning() override
  {
    if (!pending_id_) {
      return try_send();
    }
    typename Response::SharedPtr response;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      response = state_->response;
    }
    if (response) {
      pending_id_.reset();
      return on_response(*response);
    }
    if (timeout_s_ > 0.0 && seconds_since(sent_at_) > timeout_s_) {
      drop_pending();
      return on_error(
        LeafError::TIMEOUT, "no response within " + std::to_string(timeout_s_) + " s");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {drop_pending();}

  RosContextPtr ctx_;

private:
  struct Shared
  {
    std::mutex mutex;
    typename Response::SharedPtr response;
  };

  BT::NodeStatus try_send()
  {
    if (!client_ || !client_->service_is_ready()) {
      if (seconds_since(started_at_) > ctx_->timeouts.server_wait_s) {
        return on_error(LeafError::SERVER_UNAVAILABLE, "service not available");
      }
      return BT::NodeStatus::RUNNING;
    }
    auto request = std::make_shared<Request>();
    if (!make_request(request.get())) {
      return BT::NodeStatus::FAILURE;
    }
    auto state = state_;
    auto sent = client_->async_send_request(
      request, [state](typename Client::SharedFuture future) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->response = future.get();
      });
    pending_id_ = sent.request_id;
    sent_at_ = SteadyClock::now();
    return BT::NodeStatus::RUNNING;
  }

  void drop_pending()
  {
    if (pending_id_ && client_) {
      client_->remove_pending_request(*pending_id_);
    }
    pending_id_.reset();
  }

  typename Client::SharedPtr client_;
  double timeout_s_;
  std::shared_ptr<Shared> state_;
  std::optional<int64_t> pending_id_;
  SteadyClock::time_point started_at_;
  SteadyClock::time_point sent_at_;
};

}  // namespace peach2_task::ros
