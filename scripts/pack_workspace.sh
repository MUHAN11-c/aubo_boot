#!/usr/bin/env bash
# pack_workspace.sh —— 把工作区打包成迁移压缩包（源码 + 资产 + 配置 + git 历史），
# 可选携带测试过程记录（runs/ 最近 N 天）、_archive/ 与 _tools/。
#
# 排除的都是在目标环境可重新搭建/重新下载/重新生成的，或本机工具态：
#   编译产物      build/  install/  log/  （新环境重新 colcon build）
#   虚拟环境      aubo_py3.12/ 与任意层 .venv* / venv / __pycache__（按 requirements.txt 重装）
#   前端依赖      node_modules/（npm ci / npm install 再生；web/public 下 vendor 发行件保留）
#   IDE/AI 工具态 .mimosa .serena .codegraph .cache .playwright-mcp .zcode .codex .mimocode
#                 .remember .v2c .video_agent 等（含 _archive 内的 .mimosa/__pycache__）
#   IntelliSense  .vscode/browse.vc.db*（可再生，可达数 GB；.vscode 的两个配置文件入包）
#   旧打包产物    out/（历史 tar.zst）
#   可重下大件    ivg 模型权重 *.onnx/*.pt（见 models/README.md 与 fetch_*.sh）、
#                 ivg_sim YCB meshes（fetch_ycb.sh）、mobileclip2_b.ts
#   可重渲/重建   reconstruction/output/ 内未跟踪的重渲大件（*.blend/*.exr 等，git 索引现算）、
#                 field_no_grass.glb（见 sim_worlds/README.md 重导出）、.dinov2_index.npz
#
# 保留：src/ 全部源码与资产（含 sim_description/sim_worlds meshes、peach_harvester/model 自训练模型、
#       驱动 vendor、web/public 前端 vendor JS——three.js/robotwebtools 的 build 发行件，
#       仓内无重下脚本）、docs/ wiki/ reports/ weights/ peach_autonomous_harvest_design/、
#       根文档与 requirements.txt、.clangd/.colcon 构建配置、.github、.agents、
#       .cursor/rules（git 跟踪的规则）、scripts/（含本脚本）、.git/ 历史（--no-git 去掉）。
#
# 可选入包（默认不含）：
#   --include-archive   _archive/（厂商资料与历史归档）
#   --include-tools     _tools/（Blender 发行版、cadenv、shear_pipeline 等工具与产物）
#   --records-days N    runs/ 里最近 N 天有改动的整条运行记录（含 mcap 回放包、截图、日志）
#
# 用法：
#   scripts/pack_workspace.sh [--dry-run] [--no-git] [--out DIR] [--gz | --zstd]
#                             [--records-days N] [--include-archive] [--include-tools]
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
# 所有相对路径扫描均从工作区根执行，允许从任意 cwd 调用。
cd "$WS"

usage() {
  sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

DRY_RUN=0
WITH_GIT=1
WITH_ARCHIVE=0
WITH_TOOLS=0
RECORD_DAYS=0
OUT_DIR="$(dirname "$WS")"
FORMAT="auto"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --no-git) WITH_GIT=0 ;;
    --include-archive) WITH_ARCHIVE=1 ;;
    --include-tools) WITH_TOOLS=1 ;;
    --records-days) [[ $# -ge 2 ]] || { echo "--records-days 需要天数参数" >&2; exit 2; }
                    RECORD_DAYS="$2"; shift ;;
    --records-days=*) RECORD_DAYS="${1#*=}" ;;
    --out) [[ $# -ge 2 ]] || { echo "--out 需要目录参数" >&2; exit 2; }; OUT_DIR="$2"; shift ;;
    --out=*) OUT_DIR="${1#*=}" ;;
    --gz) FORMAT="gz" ;;
    --zstd) FORMAT="zstd" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if ! [[ "$RECORD_DAYS" =~ ^[0-9]+$ ]]; then
  echo "--records-days 必须是整数天数: $RECORD_DAYS" >&2
  exit 2
fi

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
INCLUDE=(AGENTS.md CLAUDE.md LICENSE README.md README_PACKAGE.md .gitignore .zcodeignore
         .clangd .colcon .github .agents .cursor/rules .vscode docs wiki reports weights
         peach_autonomous_harvest_design scripts src requirements.txt
         cursor_peach_camera_simulation_validati.md)
[[ $WITH_GIT -eq 1 ]] && INCLUDE+=(.git)
[[ $WITH_ARCHIVE -eq 1 ]] && INCLUDE+=(_archive)
[[ $WITH_TOOLS -eq 1 ]] && INCLUDE+=(_tools)
PATHS=()
for e in "${INCLUDE[@]}"; do
  [[ -e "$WS/$e" ]] && PATHS+=("$NAME/$e")
done

# 测试过程记录：runs/ 下最近 N 天有改动的整条运行记录（含二进制回放/截图），
# 以条目（顶层子目录）为单位整条带上，避免同一会话只带一半文件。
if [[ $RECORD_DAYS -gt 0 ]]; then
  [[ -d "$WS/runs" ]] || { echo "--records-days 指定了但 runs/ 不存在" >&2; exit 1; }
  while IFS= read -r -d '' rel; do
    PATHS+=("$NAME/runs/${rel%%/*}")
  done < <(find runs -mindepth 2 -newermt "${RECORD_DAYS} days ago" -printf '%P\0')
  while IFS= read -r -d '' f; do
    PATHS+=("$NAME/runs/$f")
  done < <(find runs -mindepth 1 -maxdepth 1 -type f -newermt "${RECORD_DAYS} days ago" -printf '%f\0')
  # 去重（同一会话多条文件命中时反复追加）
  mapfile -t PATHS < <(printf '%s\n' "${PATHS[@]}" | awk '!seen[$0]++')
fi

[[ ${#PATHS[@]} -gt 0 ]] || { echo "工作区内容为空？$WS" >&2; exit 1; }

# 任意层级排除（按名字/后缀匹配，与 .gitignore 的生成物策略对齐）。
# 注意：不用裸名 build/install/log 排除——web/public/ 下的 build/ 是 three.js、
# robotwebtools 前端 vendor 发行文件（importmap.js 引用、仓内无重下脚本），必须入包；
# colcon 误在 src 包内留下的产物目录由下方打包时现查的锚定排除处理。
EXC_ANY=(
  __pycache__ .venv-yolo .venv venv node_modules .pytest_cache .cache Testing
  .codegraph .serena .mimosa .zcode .v2c .video_agent .codex .mimocode
  .remember .claude .playwright-mcp
  test_results harvest_runs web_runs peach_sessions validation_runs rosbag2_* depth_eval_out
  '*.pyc' '*.pyo' '*.orig' '*~' .DS_Store Thumbs.db .patch_probe
  '*.preview.png' .dinov2_index.npz mobileclip2_b.ts
)
# *.mcap/*.db3/*.bag 默认不入包；带 --records-days 时回放包属于测试证据，需放行。
# 这些后缀在 runs/ 之外基本不出现（_archive 内已核对无 mcap）。
if [[ $RECORD_DAYS -eq 0 ]]; then
  EXC_ANY+=('*.mcap' '*.db3' '*.bag')
fi
# 锚定路径排除（相对工作区根）
EXC_ANCHOR=(
  "$NAME/_tools/blender-*"
  "$NAME/_tools/cadenv"
  "$NAME/_tools/shear_pipeline"
  "$NAME/src/peach_sim/sim_worlds/reconstruction/matrix"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.onnx"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.pt"
  "$NAME/src/ivg/ivg_pose_estimation/models/*.pth"
  "$NAME/src/ivg/ivg_sim/models/*/meshes"
  "$NAME/src/peach_sim/*/meshes/field_no_grass.glb"
  "$NAME/src/aubo_hand_eye_calibration/hand_eye/candidates"
  "$NAME/.vscode/browse.vc.db" "$NAME/.vscode/browse.vc.db-*"
  "$NAME/.vscode/browse.vc.db.lock"
)
# --include-tools 时上面三条 _tools 锚点要撤掉（用户明确要求整体入包）。
if [[ $WITH_TOOLS -eq 1 ]]; then
  _keep=()
  for p in "${EXC_ANCHOR[@]}"; do
    [[ "$p" == "$NAME/_tools/"* ]] || _keep+=("$p")
  done
  EXC_ANCHOR=("${_keep[@]}")
fi
# 注意：peach_harvester/model/*.pt 是 git 跟踪的自训练模型，必须入包，勿加全局 *.pt 排除。
# *.blend/*.exr 不做全局排除：_tools/blender 发行版自带 46 个（14M，启动模板/画笔），
# 排除会让 Blender 不完整；src 内可再生的 *.blend/*.exr 全部落在下面的 git 索引排除里。

# sim_worlds/reconstruction/output/：1.3G 未跟踪重渲大件不入包，但其中 103 个验证产物
# （图/清单，64M）在 gitignore 兜底行之前已入库、必须随包完整迁移。手工模式追不上
# （如根级 matrix_report.md 被跟踪而 matrix*/ 子目录没有），改为按 git 索引现算：
# 完全未跟踪的目录整目录排除，混合目录中的未跟踪文件逐个排除。
OUT_REL=src/peach_sim/sim_worlds/reconstruction/output
while IFS= read -r -d '' f; do
  EXC_ANCHOR+=("$NAME/$f")
done < <(git -C "$WS" ls-files --others --ignored --exclude-standard -z -- "$OUT_REL")
while IFS= read -r -d '' d; do
  if [[ -z "$(git -C "$WS" ls-files -- "$OUT_REL/$d/")" ]]; then
    EXC_ANCHOR+=("$NAME/$OUT_REL/$d")
  fi
done < <(find "$WS/$OUT_REL" -mindepth 1 -type d -printf '%P\0')

# colcon 误在包源码树里留下的 build/install/log 产物目录：打包时现查现排
# （新环境重新 colcon build 即有）。web/public/ 下的前端 vendor build 不在排除之列。
while IFS= read -r -d '' d; do
  EXC_ANCHOR+=("$NAME/$d")
done < <(find src docs wiki reports peach_autonomous_harvest_design .agents .github .colcon \
           -mindepth 1 -type d \( -name build -o -name install -o -name log \) \
           -not -path '*/web/public/*' -print0)

TAR_ARGS=(--create --file="$OUT_FILE" "${COMP[@]}" -C "$(dirname "$WS")"
          --exclude="$NAME/$(basename "$OUT_FILE")")
for p in "${EXC_ANY[@]}" "${EXC_ANCHOR[@]}"; do
  TAR_ARGS+=(--exclude="$p")
done
TAR_ARGS+=("${PATHS[@]}")

# 粗估入包体积（白名单合计减锚定排除项），并核对目标盘剩余空间
inc_kb=0
for e in "${PATHS[@]}"; do
  rel="${e#"$NAME"/}"
  # runs/ 的条目可能因去重后仍重复，du 前先去重路径集
  [[ -e "$WS/$rel" ]] && inc_kb=$(( inc_kb + $(du -sk "$WS/$rel" | cut -f1) ))
done
exc_kb=0
declare -A counted_exclusions=()
while IFS= read -r p; do
  p="${p#"$NAME"/}"
  anchor_in_include=0
  for included_path in "${PATHS[@]}"; do
    included_path="${included_path#"$NAME"/}"
    if [[ "$p" == "$included_path" || "$p" == "$included_path/"* ]]; then
      anchor_in_include=1
      break
    fi
  done
  # 已整体排除的顶层目录不能再次扣体积，否则会出现负数预估。
  if [[ $anchor_in_include -eq 1 && -e "$WS/$p" ]]; then
    ancestor="$p"
    already_counted=0
    while [[ -n "$ancestor" ]]; do
      if [[ -n "${counted_exclusions[$ancestor]:-}" ]]; then
        already_counted=1
        break
      fi
      [[ "$ancestor" == */* ]] || break
      ancestor="${ancestor%/*}"
    done
    if [[ $already_counted -eq 0 ]]; then
      exc_kb=$(( exc_kb + $(du -sk "$WS/$p" | cut -f1) ))
      counted_exclusions[$p]=1
    fi
  fi
# 按路径排序使父目录先处理；父目录已扣除时，子文件/子目录不再重复扣除。
done < <(printf '%s\n' "${EXC_ANCHOR[@]}" | LC_ALL=C sort -u)
est_kb=$(( inc_kb - exc_kb ))
avail_kb="$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}')"

echo "工作区    : $WS"
echo "输出      : $OUT_FILE ($(command -v zstd >/dev/null 2>&1 && [[ $FORMAT == zstd ]] && echo zstd || echo ${FORMAT}))"
echo "入包约    : $(( est_kb / 1024 )) MiB（未压），目标盘可用 $(( avail_kb / 1024 / 1024 )) GiB"
echo "包含顶层  : ${PATHS[*]}"
[[ $RECORD_DAYS -gt 0 ]] && echo "测试记录  : runs/ 最近 $RECORD_DAYS 天（整条运行记录，含 mcap/截图/日志）"
echo "排除/未入包项（存在者，含大小上界）:"
for p in build install log aubo_py3.12 out \
         src/peach_sim/sim_worlds/reconstruction/output src/peach_sim/sim_worlds/reconstruction/matrix \
         src/ivg/ivg_pose_estimation/models src/ivg/ivg_sim/models \
         src/aubo_hand_eye_calibration/hand_eye/candidates mobileclip2_b.ts .vscode/browse.vc.db; do
  [[ -e "$WS/$p" ]] && du -sh "$WS/$p" | sed 's/^/  /'
done
[[ $WITH_ARCHIVE -eq 0 && -e "$WS/_archive" ]] && du -sh "$WS/_archive" | sed 's/^/  /'
if [[ $WITH_TOOLS -eq 0 && -e "$WS/_tools" ]]; then
  du -sh "$WS/_tools" | sed 's/^/  /'
fi
[[ $RECORD_DAYS -eq 0 && -e "$WS/runs" ]] && du -sh "$WS/runs" | sed 's/^/  /'
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
echo "  tar -tf '$OUT_FILE' | grep -E 'aubo_py3.12/|/build/|\\.onnx' | head"
