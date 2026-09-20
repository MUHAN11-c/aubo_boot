#!/usr/bin/env bash
# 纯核门：零 ROS pytest + interface_manifest。colcon 绿 ≠ 套袋验收。
# 2026-09-18 测试改名（消除 pytest 不收集假绿）：vision_test_* →
# test_vision_*、supervisor_test_* → test_supervisor_*（对齐 test_*.py
# 默认收集规则；内容零变化）。
# 2026-09-20 W1：新增 peach_common 参数设施单源组（PYTHONPATH 前置 +
# 纯核测试；三包 yaml_params/param_rules 已 shim 转发）。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT/src/peach_common:$ROOT/src/peach_harvester:$ROOT/src/peach_observability:$ROOT/src/peach_bringup:$ROOT/src/peach_vegetation${PYTHONPATH:+:$PYTHONPATH}"

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
  src/peach_harvester/test/test_vision_tool_budget.py \
  src/peach_harvester/test/test_vision_runtime_core.py \
  src/peach_harvester/test/test_vision_tool_profiles.py \
  src/peach_harvester/test/test_vision_import_guard.py \
  src/peach_harvester/test/test_vision_cross_field.py \
  src/peach_harvester/test/test_vision_model_contract.py \
  src/peach_harvester/test/test_vision_evidence.py \
  src/peach_harvester/test/test_vision_frozen_keys.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/test_supervisor_harvest_fsm.py \
  src/peach_harvester/test/test_supervisor_reducer.py \
  src/peach_harvester/test/test_supervisor_ledger.py \
  src/peach_harvester/test/test_supervisor_watchdog.py \
  src/peach_harvester/test/test_supervisor_lifecycle.py \
  src/peach_harvester/test/test_supervisor_idl_constants.py \
  src/peach_harvester/test/test_supervisor_import_guard.py \
  src/peach_harvester/test/test_supervisor_frozen_keys.py \
  src/peach_harvester/test/test_supervisor_cross_field.py \
  src/peach_harvester/test/test_supervisor_param_rules.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_common/test/test_yaml_params.py \
  src/peach_common/test/test_param_rules.py \
  src/peach_common/test/test_qos.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/test_view_policy.py \
  src/peach_harvester/test/test_view_planner_port.py \
  src/peach_harvester/test/test_batch_policy.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_observability/test/test_path_metrics.py \
  src/peach_observability/test/test_bag_report.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_vegetation/test/test_leaf_mask.py \
  src/peach_vegetation/test/test_frangi.py \
  src/peach_vegetation/test/test_params.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_system_tests/test/test_preflight.py

python3 src/peach_interfaces/scripts/check_interface_manifest.py
echo 'r0_gate ok'
