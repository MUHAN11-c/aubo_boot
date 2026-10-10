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

from ament_pep257.main import main
import pytest

# 移植包 docstring 为中文自由格式：句读/分节下划线等排版类（D2xx/D4xx 排版项）
# 类别忽略；结构类问题（缺失、缩进等）保持检查。口径与包根 flake8.ini 一致。


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(
        argv=['.', 'test', '--add-ignore',
              'D205', 'D209', 'D400', 'D403', 'D405', 'D406', 'D407', 'D413', 'D415']
    )
    assert rc == 0, 'Found code style errors / warnings'
