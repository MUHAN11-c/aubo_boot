#!/usr/bin/env bash
# MobileSAM 权重获取（约 39MB，不入 git；本仓 peach_harvester 已有副本时也可直接 cp）
# 官方发布物与 ultralytics mobile_sam.pt 一致（MobileSAM Apache-2.0）
set -euo pipefail
cd "$(dirname "$0")"

URL="https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt"

if [ -f mobile_sam.pt ]; then
  echo "mobile_sam.pt 已存在: $(du -h mobile_sam.pt | cut -f1)"
  exit 0
fi

# 本仓已有副本优先（同源文件，零网络）
PEACH_COPY="../../peach_harvester/model/mobile_sam.pt"
if [ -f "$PEACH_COPY" ]; then
  cp "$PEACH_COPY" mobile_sam.pt
  echo "已从仓内 peach_harvester 副本复制 mobile_sam.pt"
  exit 0
fi

echo "下载 $URL ..."
if command -v curl >/dev/null 2>&1; then
  curl -L --fail -o mobile_sam.pt "$URL"
else
  wget -O mobile_sam.pt "$URL"
fi
echo "完成: $(du -h mobile_sam.pt | cut -f1)"
