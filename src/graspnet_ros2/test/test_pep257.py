# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_pep257.main import main
import pytest

# vendored 上游推理子集（graspnet_lib/）不参与本包 docstring 风格检查；
# Google 风格的 Args/Returns 分节无下划线（D406/D407/D413）作为本包约定忽略。
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_FIRST_PARTY = sorted(
    str(p) for p in (_PACKAGE_ROOT / 'graspnet_ros2').glob('*.py')
)


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    assert _FIRST_PARTY, '未找到待检查文件'
    rc = main(
        argv=_FIRST_PARTY
        + ['--add-ignore', 'D406', 'D407', 'D413']
    )
    assert rc == 0, 'Found code style errors / warnings'
