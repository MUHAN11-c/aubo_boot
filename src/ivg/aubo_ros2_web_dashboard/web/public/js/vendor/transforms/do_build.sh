#!/usr/bin/env bash
# transforms vendor 打包脚本（在 /tmp/transforms_build 内执行）
set -e
cd /tmp/transforms_build
if [ ! -d node_modules ]; then
  npm install --silent
fi
./node_modules/.bin/esbuild entry.ts --bundle --format=esm --platform=browser \
  --outfile=transforms.esm.js --log-level=warning
echo "BUNDLE_OK $(wc -c < transforms.esm.js) bytes"
