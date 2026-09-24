#!/usr/bin/env bash
# 纯核门：零 ROS pytest + interface_manifest。colcon 绿 ≠ 套袋验收。
# 2026-09-18 测试改名（消除 pytest 不收集假绿）：vision_test_* →
# test_vision_*、supervisor_test_* → test_supervisor_*（对齐 test_*.py
# 默认收集规则；内容零变化）。
# 2026-09-20 W1：新增 peach_common 参数设施单源组（PYTHONPATH 前置 +
# 纯核测试；三包 yaml_params/param_rules 已 shim 转发）。
# 2026-09-24 阶段一补齐（testing-log.md:564 记档漂移）：全量 diff 各包
# test/ 后补 13 文件（vision 4/reconstruction 1/harvester yaml_params 1/
# common 1/observability 2/system_tests 3/bringup 1）+ 新增 peach_sim 组。
# 排除（非零 ROS 或需起栈，仍由 colcon test 覆盖）：各包 test_flake8/
# test_pep257（lint）、test_hotpaths（imports rclpy/rosbag2）、test_tcp_trajectory（geometry_msgs）、
# test_mock_launch（launch_testing）、test_vision_estimator（pipeline→
# msg_builders→geometry_msgs）、test_reconstruction_decision_validity
# （依赖已 build 的 peach_interfaces 生成消息；其 docstring 自述属 colcon
# 层——:564 漂移记档对此文件的暗示系误报）。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT/src/peach_common:$ROOT/src/peach_harvester:$ROOT/src/peach_observability:$ROOT/src/peach_bringup:$ROOT/src/peach_vegetation:$ROOT/src/peach_sim${PYTHONPATH:+:$PYTHONPATH}"

python3 - <<'PY'
import numpy
assert numpy.__version__.startswith('1.26.'), numpy.__version__
print('numpy', numpy.__version__)
PY

python3 -c "import peach_harvester.vision.scene_perception.identity"
python3 -c "import peach_harvester.supervisor.harvest_fsm"
python3 -c "import peach_bringup.preflight"
python3 -c "import peach_vegetation.split"
python3 -c "import peach_observability.path_metrics"
python3 -c "import peach_common"

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/test_vision_param_rules.py \
  src/peach_harvester/test/test_vision_identity.py \
  src/peach_harvester/test/test_vision_geometry.py \
  src/peach_harvester/test/test_vision_plan_updater.py \
  src/peach_harvester/test/test_vision_tool_budget.py \
  src/peach_harvester/test/test_vision_runtime_core.py \
  src/peach_harvester/test/test_vision_tool_profiles.py \
  src/peach_harvester/test/test_vision_import_guard.py \
  src/peach_harvester/test/test_vision_cross_field.py \
  src/peach_harvester/test/test_vision_model_contract.py \
  src/peach_harvester/test/test_vision_evidence.py \
  src/peach_harvester/test/test_vision_frozen_keys.py \
  src/peach_harvester/test/test_vision_gating.py \
  src/peach_harvester/test/test_vision_ransac.py \
  src/peach_harvester/test/test_vision_sam_fallback.py \
  src/peach_harvester/test/test_vision_scene_params.py \
  src/peach_harvester/test/test_reconstruction_session.py \
  src/peach_harvester/test/test_reconstruction_refine_result.py \
  src/peach_harvester/test/test_refit_orchestrator.py \
  src/peach_harvester/test/test_reconstruction_session_recorder.py \
  src/peach_harvester/test/test_reconstruction_icp_cache.py \
  src/peach_harvester/test/test_reconstruction_mask_gate_f1.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/test_supervisor_harvest_fsm.py \
  src/peach_harvester/test/test_supervisor_reducer.py \
  src/peach_harvester/test/test_supervisor_batch.py \
  src/peach_harvester/test/test_supervisor_lifecycle.py \
  src/peach_harvester/test/test_supervisor_idl_constants.py \
  src/peach_harvester/test/test_supervisor_import_guard.py \
  src/peach_harvester/test/test_supervisor_frozen_keys.py \
  src/peach_harvester/test/test_supervisor_cross_field.py \
  src/peach_harvester/test/test_supervisor_param_rules.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_common/test/test_yaml_params.py \
  src/peach_common/test/test_param_rules.py \
  src/peach_common/test/test_paths.py \
  src/peach_common/test/test_qos.py \
  src/peach_common/test/test_lifecycle.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/test_view_policy.py \
  src/peach_harvester/test/test_view_planner_port.py \
  src/peach_harvester/test/test_batch_policy.py \
  src/peach_harvester/test/test_yaml_params.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_observability/test/test_path_metrics.py \
  src/peach_observability/test/test_bag_report.py \
  src/peach_observability/test/test_params.py \
  src/peach_observability/test/test_pipeline.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_vegetation/test/test_leaf_mask.py \
  src/peach_vegetation/test/test_frangi.py \
  src/peach_vegetation/test/test_params.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_system_tests/test/test_preflight.py \
  src/peach_system_tests/test/test_perf_baseline.py \
  src/peach_system_tests/test/test_baseline_inventory.py \
  src/peach_system_tests/test/test_replay_approach.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_bringup/test/test_params.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_sim/test/test_params.py \
  src/peach_sim/test/test_scene.py

python3 src/peach_interfaces/scripts/check_interface_manifest.py
echo 'r0_gate ok'
