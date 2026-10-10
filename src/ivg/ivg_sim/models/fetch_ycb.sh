#!/usr/bin/env bash
# 恢复 YCB meshes（约 118MB，不入 git）：克隆上游镜像后按本库对象清单拷贝
set -euo pipefail
cd "$(dirname "$0")"

# 本库 16 对象（与 setup.py 安装清单一致）
OBJECTS="apple banana bleach_cleanser bowl chips_can cracker_box gelatin_box \
master_chef_can mustard_bottle pitcher_base potted_meat_can pudding_box \
sugar_box tomato_soup_can tuna_fish_can windex_bottle"

# 已齐则退出
missing=0
for obj in $OBJECTS; do
  [ -f "$obj/meshes/textured.dae" ] || missing=1
done
if [ "$missing" -eq 0 ]; then
  echo "YCB meshes 已齐"
  exit 0
fi

echo "克隆上游 YCB SDF 镜像（约 400MB，临时目录用后即删）..."
TMP=$(mktemp -d)
git clone --depth 1 https://github.com/CentralLabFacilities/gazebo_ycb "$TMP/ycb"

for obj in $OBJECTS; do
  if [ -d "$TMP/ycb/$obj/meshes" ]; then
    mkdir -p "$obj/meshes"
    cp -f "$TMP/ycb/$obj/meshes/"* "$obj/meshes/"
    echo "  $obj ✓"
  else
    echo "  $obj：上游缺失 meshes，跳过" >&2
  fi
done

rm -rf "$TMP"
echo "完成：$(du -sh . | cut -f1)"
