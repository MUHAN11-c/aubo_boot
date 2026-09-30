// Copyright 2026 wjz
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>

#include "behaviortree_cpp/basic_types.h"

/// Port lists of the ROS-facing leaves, shared by the real nodes and by test doubles so that
/// trees/harvest_batch.xml parses identically in production and in gtest.
namespace peach2_task::bt::ports
{

inline BT::PortsList check_safety() {return {};}

inline BT::PortsList move_to_named()
{
  return {
    BT::InputPort<std::string>("target", "SRDF group_state name"),
    BT::InputPort<double>("velocity_scaling", 0.0, "0 = manipulation default"),
  };
}

inline BT::PortsList build_scene_snapshot()
{
  return {BT::InputPort<bool>("clear_previous", true, "true at batch start, false = merge")};
}

inline BT::PortsList begin_scene() {return {};}

inline BT::PortsList wait_target_set_locked() {return {};}

inline BT::PortsList select_target()
{
  return {BT::OutputPort<std::string>("target_id")};
}

inline BT::PortsList observe_target()
{
  return {
    BT::InputPort<std::string>("target_id"),
    BT::InputPort<unsigned>("max_views"),
    BT::InputPort<bool>("neck_remeasure", false, ""),
    BT::OutputPort<uint64_t>("model_revision"),
  };
}

inline BT::PortsList check_decision()
{
  return {
    BT::InputPort<std::string>("target_id"),
    BT::InputPort<std::string>("tool_id"),
    BT::InputPort<std::string>("level", "approach", "approach | sleeve | cut"),
    BT::InputPort<uint64_t>("min_model_revision", 0, "0 = any"),
  };
}

inline BT::PortsList harvest_target()
{
  return {
    BT::InputPort<std::string>("target_id"),
    BT::InputPort<std::string>("tool_id"),
    BT::InputPort<int>("intent", "RunBatch.INTENT_*; FULL -> MODE_FULL, else PREGRASP_ONLY"),
  };
}

}  // namespace peach2_task::bt::ports
