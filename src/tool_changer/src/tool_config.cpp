// tools.yaml 解析实现 — 源自 aubo_boot gripper_swap_worker::loadToolConfig。
#include "tool_changer/tool_config.hpp"

#include <yaml-cpp/yaml.h>

namespace tool_changer
{

bool parseSwapDirection(
  const std::string & direction, std::string & source, std::string & target)
{
  source.clear();
  target.clear();
  if (direction.empty()) {
    return false;
  }
  auto pos = direction.find("_to_");
  if (pos != std::string::npos) {
    source = direction.substr(0, pos);
    target = direction.substr(pos + 4);
  } else {
    target = direction;
  }
  return true;
}

bool loadToolConfigs(
  const std::string & yaml_path,
  std::map<std::string, ToolConfig> & out,
  std::string & err)
{
  out.clear();
  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    err = std::string("YAML 加载失败 ") + yaml_path + ": " + e.what();
    return false;
  }

  auto tools = config["tools"];
  if (!tools) {
    err = "tools.yaml 缺少 'tools' 节点";
    return false;
  }

  for (const auto & kv : tools) {
    std::string tid = kv.first.as<std::string>();
    const auto & t = kv.second;
    ToolConfig cfg;

    cfg.id = tid;
    cfg.name = t["name"].as<std::string>("");
    cfg.type = t["type"].as<std::string>("");
    cfg.parameters = t["parameters"].as<std::string>("");

    // dock_approach_joints（优先）或 dock_above XYZ 笛卡尔直线（回退）
    if (t["dock_approach_joints"] && t["dock_approach_joints"].size() == 6) {
      for (int i = 0; i < 6; ++i) {
        cfg.dock_approach_joints[i] = t["dock_approach_joints"][i].as<double>();
      }
    } else if (t["dock_above"] && t["dock_above"].size() == 3) {
      cfg.has_dock_approach_xyz = true;
      cfg.dock_approach_xyz[0] = t["dock_above"]["x"].as<double>();
      cfg.dock_approach_xyz[1] = t["dock_above"]["y"].as<double>();
      cfg.dock_approach_xyz[2] = t["dock_above"]["z"].as<double>();
    } else {
      // 与 aubo_boot 一致：缺 dock 定位的工具跳过（如仿真杯具）
      continue;
    }

    if (t["trajectory"]) {
      const auto & tp = t["trajectory"];
      std::string strat = tp["strategy"].as<std::string>("vertical");
      if (strat == "slide") {
        cfg.strategy = TrajectoryStrategy::kSlide;
        cfg.slide.depth = tp["depth"].as<double>(cfg.slide.depth);
        cfg.slide.lift = tp["lift"].as<double>(cfg.slide.lift);
        cfg.slide.slide_y = tp["slide_y"].as<double>(cfg.slide.slide_y);
        cfg.slide.seat = tp["seat"].as<double>(cfg.slide.seat);
        cfg.slide.settle_sec = tp["settle_sec"].as<double>(cfg.slide.settle_sec);
        cfg.slide.release_sec = tp["release_sec"].as<double>(cfg.slide.release_sec);
        cfg.slide.lock_sec = tp["lock_sec"].as<double>(cfg.slide.lock_sec);
      } else {
        cfg.vertical.depth = tp["depth"].as<double>(cfg.vertical.depth);
        cfg.vertical.lift = tp["lift"].as<double>(cfg.vertical.lift);
        cfg.vertical.settle_sec = tp["settle_sec"].as<double>(cfg.vertical.settle_sec);
      }
    }

    out[tid] = cfg;
  }
  return !out.empty();
}

}  // namespace tool_changer
