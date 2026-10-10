#!/usr/bin/env bash
# pack_workspace.sh —— 把工作区打包成迁移压缩包（源码 + 资产 + 配置 + git 历史）。
#
# 排除的都是在目标环境可重新搭建/重新下载/重新生成的，或本机工具态：
#   归档          _archive/
#   编译产物      build/  install/  log/  （新环境重新 colcon build）
#   虚拟环境      aubo_py3.12/ 与任意层 .venv* / venv / __pycache__（按 requirements.txt 重装）
#   会话/运行输出 runs/  test_results/  rosbag2_* 等（含 *.mcap / *.db3 / *.bag）
#   IDE/AI 工具态 .vscode .cursor .zcode .mimosa .serena .codegraph …
#   可重下大件    _tools/blender-*（官方发行版）、ivg 模型权重 *.onnx/*.pt（见其
#                 models/README.md 与 fetch_*.sh）、ivg_sim YCB meshes（fetch_ycb.sh）、
#                 mobileclip2_b.ts
#   可重渲/重建   output/ 内未跟踪的重渲大件（跟踪的验证产物随包迁移）、*.blend、
#                 *.preview.png、.dinov2_index.npz
#
# 保留：src/ 全部源码与资产（含 peach_sim/meshes、peach_harvester/model 自训练模型、
#       驱动 vendor、web/public 前端 vendor JS——three.js/robotwebtools 的 build 发行件，
#       仓内无重下脚本）、docs/、plans/、根文档、requirements.txt、.clangd/.colcon 构建配置、
#       .github、.agents、.cursor/rules（git 跟踪的规则）、scripts/（含本脚本）、
#       _tools/*.sh 小脚本；.git/ 历史默认入包（--no-git 去掉）。
#
# 用法：
#   scripts/pack_workspace.sh [--dry-run] [--no-git] [--out DIR] [--gz | --zstd]
#
# 输出默认写到工作区上一级：<工作区名>-src-<时间戳>.tar.zst（无 zstd 则 .tar.gz）。
# 目标环境恢复示例：
#   tar -xf aubo_e5_jazzy_ws-src-*.tar.zst
#   rosdep install --from-paths src --ignore-src -r -y
#   python3 -m venv aubo_py3.12 && aubo_py3.12/bin/pip install -r requirements.txt
#   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# 大件重取：ivg 权重见 src/ivg/ivg_pose_estimation/models/README.md；
#   YCB meshes 见 src/ivg/ivg_sim/models/fetch_ycb.sh；Blender 重下放 _tools/。

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="$(basename "$WS")"

usage() {
  sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

DRY_RUN=0
WITH_GIT=1
OUT_DIR="$(dirname "$WS")"
FORMAT="auto"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --no-git) WITH_GIT=0 ;;
    --out) [[ $# -ge 2 ]] || { echo "--out 需要目录参数" >&2; exit 2; }; OUT_DIR="$2"; shift ;;
    --out=*) OUT_DIR="${1#*=}" ;;
    --gz) FORMAT="gz" ;;
    --zstd) FORMAT="zstd" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if [[ "$FORMAT" == "auto" ]]; then
  command -v zstd >/dev/null 2>&1 && FORMAT="zstd" || FORMAT="gz"
fi
STAMP="$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"
if [[ "$FORMAT" == "zstd" ]]; then
  COMP=(--use-compress-program='zstd -T0'); EXT="zst"
else
  command -v pigz >/dev/null 2>&1 && COMP=(--use-compress-program=pigz) || COMP=(-z)
  EXT="gz"
fi
OUT_FILE="$OUT_DIR/${NAME}-src-${STAMP}.tar.${EXT}"

# 顶层白名单：不在此列的根目录（_archive/build/install/log/runs/各工具点目录）整体不入包。
INCLUDE=(AGENTS.md CLAUDE.md LICENSE README.md .gitignore .zcodeignore .clangd .colcon
         .github .agents .cursor/rules docs plans scripts src requirements.txt _tools)
[[ $WITH_GIT -eq 1 ]] && INCLUDE+=(.git)
PATHS=()
for e in "${INCLUDE[@]}"; do
  [[ -e "$WS/$e" ]] && PATHS+=("$NAME/$e")
done
[[ ${#PATHS[@]} -gt 0 ]] || { echo "工作区内容为空？$WS" >&2; exit 1; }

# 任意层级排除（按名字/后缀匹配，与 .gitignore 的生成物策略对齐）
# 任意层级排除（按名字/后缀匹配，与 .gitignore 的生成物策略对齐）。
# 注意：不用裸名 build/install/log 排除——web/public/ 下的 build/ 是 three.js、
# robotwebtools 前端 vendor 发行文件（importmap.js 引用、仓内无重下脚本），必须入包；
# colcon 误在 src 包内留下的产物目录由下方打包时现查的锚定排除处理。
EXC_ANY=(
  __pycache__ .venv-yolo .venv venv .pytest_cache .cache
  .vscode .codegraph .serena .mimosa .zcode .v2c .video_agent .codex .mimocode
  .remember .claude
  test_results harvest_runs web_runs peach_sessions validation_runs rosbag2_* depth_eval_out
  '*.pyc' '*.pyo' '*.mcap' '*.db3' '*.bag' '*.blend' '*.blend1' '*.exr' '*.orig' '*~'
  .DS_Store Thumbs.db .patch_probe
  '*.preview.png' .dinov2_index.npz mobileclip2_b.ts
)
# 锚定路径排除（相对工作区根）
EXC_ANCHOR=(
  "$NAME/_tools/blender-*"
  "$NAME/_tools/cadenv"
  "$NAME/_tools/shear_pipeline"
  "$NAME/src/peach_sim/reconstruction/matrix"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.onnx"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.pt"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.pth"
  "$NAME/src/ivg/ivg_sim/models/*/meshes"
  "$NAME/src/aubo_hand_eye_calibration/hand_eye/candidates"
)
# 注意：peach_harvester/model/*.pt 是 git 跟踪的自训练模型，必须入包，勿加全局 *.pt 排除。

# peach_sim/reconstruction/output/：1.3G 未跟踪重渲大件不入包，但其中 103 个验证产物
# （图/清单，64M）在 gitignore 兜底行之前已入库、必须随包完整迁移。手工模式追不上
# （如根级 matrix_report.md 被跟踪而 matrix*/ 子目录没有），改为按 git 索引现算：
# 完全未跟踪的目录整目录排除，混合目录中的未跟踪文件逐个排除。
OUT_REL=src/peach_sim/reconstruction/output
while IFS= read -r -d '' f; do
  EXC_ANCHOR+=("$NAME/$f")
done < <(git -C "$WS" ls-files --others --ignored --exclude-standard -z -- "$OUT_REL")
while IFS= read -r d; do
  if [[ -z "$(git -C "$WS" ls-files -- "$OUT_REL/$d/")" ]]; then
    EXC_ANCHOR+=("$NAME/$OUT_REL/$d")
  fi
done < <(find "$WS/$OUT_REL" -mindepth 1 -type d -printf '%P\n')

# colcon 误在包源码树里留下的 build/install/log 产物目录：打包时现查现排
# （新环境重新 colcon build 即有）。web/public/ 下的前端 vendor build 不在排除之列。
while IFS= read -r d; do
  EXC_ANCHOR+=("$NAME/$d")
done < <(find src docs plans _tools .agents .github .colcon -mindepth 1 -type d \
           \( -name build -o -name install -o -name log \) -not -path '*/web/public/*' -print)

TAR_ARGS=(--create --file="$OUT_FILE" "${COMP[@]}" -C "$(dirname "$WS")"
          --exclude="$NAME/$(basename "$OUT_FILE")")
for p in "${EXC_ANY[@]}" "${EXC_ANCHOR[@]}"; do
  TAR_ARGS+=(--exclude="$p")
done
TAR_ARGS+=("${PATHS[@]}")

# 粗估入包体积（白名单合计减锚定排除项），并核对目标盘剩余空间
inc_kb=0
for e in "${PATHS[@]}"; do
  inc_kb=$(( inc_kb + $(du -sk "$WS/${e#"$NAME"/}" | cut -f1) ))
done
exc_kb=0
for p in "${EXC_ANCHOR[@]}"; do
  p="${p#"$NAME"/}"
  [[ -e "$WS/$p" ]] && exc_kb=$(( exc_kb + $(du -sk "$WS/$p" | cut -f1) ))
done
est_kb=$(( inc_kb - exc_kb ))
avail_kb="$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}')"

echo "工作区    : $WS"
echo "输出      : $OUT_FILE ($(command -v zstd >/dev/null 2>&1 && [[ $FORMAT == zstd ]] && echo zstd || echo ${FORMAT}))"
echo "入包约    : $(( est_kb / 1024 )) MiB（未压），目标盘可用 $(( avail_kb / 1024 / 1024 )) GiB"
echo "包含顶层  : ${PATHS[*]}"
echo "排除锚点（存在者，含大小上界）:"
for p in _archive build install log aubo_py3.12 runs \
         _tools/blender-4.5.14-linux-x64 _tools/cadenv _tools/shear_pipeline \
         src/peach_sim/reconstruction/output src/peach_sim/reconstruction/matrix \
         src/ivg/ivg_pose_estimation/models src/ivg/ivg_sim/models \
         src/aubo_hand_eye_calibration/hand_eye/candidates mobileclip2_b.ts; do
  [[ -e "$WS/$p" ]] && du -sh "$WS/$p" | sed 's/^/  /'
done
echo "排除模式  : ${EXC_ANY[*]}"

if [[ $DRY_RUN -eq 1 ]]; then
  echo "dry-run，不执行打包。将执行的命令："
  printf '  tar'; printf ' %q' "${TAR_ARGS[@]}"; echo
  exit 0
fi

if (( avail_kb < est_kb )); then
  echo "目标盘空间不足（需约 $(( est_kb / 1024 )) MiB，可用 $(( avail_kb / 1024 )) MiB）" >&2
  exit 1
fi

tar "${TAR_ARGS[@]}"

echo "完成: $(du -h "$OUT_FILE" | cut -f1)  $OUT_FILE"
echo "抽查: tar -tf '$OUT_FILE' | head；确认大件未入包:"
echo "  tar -tf '$OUT_FILE' | grep -E '_archive/|/build/|aubo_py3.12/|reconstruction/output|\\.onnx' | head"
