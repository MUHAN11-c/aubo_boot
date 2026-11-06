#!/usr/bin/env bash
# Source this file from the checkout root or any other directory.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Use: source scripts/setup_env.bash' >&2
  exit 2
fi
export PEACH_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$PEACH_WS_ROOT/install/setup.bash" ||
      ! -f "$PEACH_WS_ROOT/aubo_py3.12/bin/activate" ]]; then
  echo 'Build the workspace and create aubo_py3.12 first; see docs/deployment.md.' >&2
  return 2
fi
source /opt/ros/jazzy/setup.bash
source "$PEACH_WS_ROOT/install/setup.bash"
source "$PEACH_WS_ROOT/aubo_py3.12/bin/activate"
PEACH_PYTHON_SITE="$PEACH_WS_ROOT/aubo_py3.12/lib/python3.12/site-packages"
export PYTHONPATH="$PEACH_PYTHON_SITE${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1
python -c 'import numpy, cv_bridge; assert numpy.__version__ == "1.26.4"; print("Peach environment ready:", numpy.__version__, cv_bridge.__file__)'
