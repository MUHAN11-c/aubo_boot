#!/bin/bash
# 从 Git 历史中移除超大文件，以便能推送到 GitHub（超过 100MB 会拒收）
# 用法：在 IVG 仓库根目录执行： bash scripts/remove_large_files_from_history.sh
set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# 与 git ls-files 中的路径一致（从仓库根算起）
F1="aubo_ros2_ws/src/.vscode/browse.vc.db"
F2="aubo_ros2_ws/src/aubo_ros2_driver.zip"

echo "=== 从 stable 分支历史中移除大文件 ==="
echo "  文件1: $F1"
echo "  文件2: $F2"
echo ""

# 需要干净工作区（或使用临时克隆）
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
  echo "当前有未提交变更，请先 stash 或提交后再运行："
  echo "  git stash push -u -m 'before remove large files'"
  echo "  bash scripts/remove_large_files_from_history.sh"
  echo "  git push github stable --force"
  echo "  git stash pop"
  exit 1
fi

export FILTER_BRANCH_SQUELCH_WARNING=1
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch \"$F1\" \"$F2\"" \
  --prune-empty --tag-name-filter cat -- stable

echo ""
echo "=== 完成。请执行强制推送（会重写远程 stable）： ==="
echo "  git push github stable --force"
echo ""
echo "若推送仍报大文件，说明这些文件在更早的提交中，需对 master 等分支也做同样 filter-branch 或使用 git filter-repo。"
