#!/usr/bin/env bash
# 将 rembg u2net.onnx 拉到本目录（约 168MB）。已存在则跳过。
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
dest="u2net.onnx"
url="https://github.com/danielgatis/rembg/releases/download/v0.0.0/u2net.onnx"
if [[ -f "$dest" && -s "$dest" ]]; then
  echo "already present: $(pwd)/$dest ($(du -h "$dest" | cut -f1))"
  exit 0
fi
echo "downloading $url"
if command -v curl >/dev/null 2>&1; then
  curl -fL --retry 3 -o "$dest.partial" "$url"
else
  wget -O "$dest.partial" "$url"
fi
mv "$dest.partial" "$dest"
echo "wrote $(pwd)/$dest ($(du -h "$dest" | cut -f1))"
