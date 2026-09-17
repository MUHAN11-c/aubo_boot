#!/usr/bin/env bash
# 纯核门：零 ROS pytest + interface_manifest。colcon 绿 ≠ 套袋验收。
# 2026-09-17 对齐 3a 并包后路径：peach_perception/peach_executor →
# peach_harvester（vision_*/supervisor_* 前缀即两组旧测试面）。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT/src/peach_harvester:$ROOT/src/peach_observability:$ROOT/src/peach_bringup${PYTHONPATH:+:$PYTHONPATH}"

python3 - <<'PY'
import numpy
assert numpy.__version__.startswith('1.26.'), numpy.__version__
print('numpy', numpy.__version__)
PY

python3 -c "import peach_harvester.vision.scene_perception.identity"
python3 -c "import peach_harvester.supervisor.harvest_fsm"
python3 -c "import peach_bringup.preflight"
python3 -c "import peach_observability.path_metrics"

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/vision_test_param_rules.py \
  src/peach_harvester/test/vision_test_identity.py \
  src/peach_harvester/test/vision_test_tool_budget.py \
  src/peach_harvester/test/vision_test_runtime_core.py \
  src/peach_harvester/test/vision_test_tool_profiles.py \
  src/peach_harvester/test/vision_test_import_guard.py \
  src/peach_harvester/test/vision_test_cross_field.py \
  src/peach_harvester/test/vision_test_model_contract.py \
  src/peach_harvester/test/vision_test_evidence.py \
  src/peach_harvester/test/vision_test_frozen_keys.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_harvester/test/supervisor_test_harvest_fsm.py \
  src/peach_harvester/test/supervisor_test_reducer.py \
  src/peach_harvester/test/supervisor_test_ledger.py \
  src/peach_harvester/test/supervisor_test_watchdog.py \
  src/peach_harvester/test/supervisor_test_lifecycle.py \
  src/peach_harvester/test/supervisor_test_idl_constants.py \
  src/peach_harvester/test/supervisor_test_import_guard.py \
  src/peach_harvester/test/supervisor_test_frozen_keys.py \
  src/peach_harvester/test/supervisor_test_cross_field.py \
  src/peach_harvester/test/supervisor_test_param_rules.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_observability/test/test_path_metrics.py \
  src/peach_observability/test/test_bag_report.py

python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_system_tests/test/test_preflight.py

python3 src/peach_interfaces/scripts/check_interface_manifest.py
echo 'r0_gate ok'
