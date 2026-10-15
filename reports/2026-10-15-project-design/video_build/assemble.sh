#!/bin/bash
# 优化前后对比视频合成（6 段：片头/感知/运动/相机掩膜/数据卡/片尾）
set -e
VB="$(cd "$(dirname "$0")" && pwd)"
ROOT="/home/mu/Desktop/aubo_e5_jazzy_ws"
VR="$ROOT/runs/compare_rviz_20261012"
ST="$ROOT/_archive/code_cleanup_2026-10-12/src/peach/peach_stereo/test/report"
MASKS="$ROOT/runs/harvest_20261014T122700_920926_s3/masks"
AU="$VB/audio"; CD="$VB/cards"; SG="$VB/segs"
ENC="-c:v libx264 -crf 20 -preset medium -pix_fmt yuv420p -r 30 -c:a aac -b:a 160k -ar 48000"

# 视频段公共滤镜：适配 1920x1080 深色底 + 淡入淡出
fit="scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1"

# ---- S1 片头 10.5s ----
ffmpeg -y -v error -loop 1 -t 10.5 -i "$CD/card_title.png" -i "$AU/n1.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.5,fade=t=out:st=10:d=0.5[v];[1:a]adelay=700|700,apad[a]" \
  -map "[v]" -map "[a]" -t 10.5 $ENC "$SG/s1.mp4"

# ---- S2 立体对比 22s ----
ffmpeg -y -v error -ss 4 -t 22 -i "$ST/fig_f_compare.mp4" -i "$CD/label_s2.png" -i "$AU/n2.wav" \
  -filter_complex "[0:v]$fit[l];[l][1:v]overlay=0:780:enable='gte(t,1)'[v];[2:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 22 $ENC "$SG/s2.mp4"

# ---- S3 冻结链完整周期 46s ----
ffmpeg -y -v error -ss 8 -t 46 -i "$VR/v1_frozen_full.mp4" -i "$CD/label_s3.png" -i "$AU/n3.wav" \
  -filter_complex "[0:v]$fit[l];[l][1:v]overlay=0:780:enable='gte(t,1)'[v];[2:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 46 $ENC "$SG/s3.mp4"

# ---- S4 相机流 + 掩膜 17s ----
ffmpeg -y -v error -ss 5 -t 17 -i "$ST/percipio2_rvizwin.mp4" \
  -framerate 10 -pattern_type glob -i "$MASKS/*.png" \
  -i "$CD/label_s4.png" -i "$AU/n4.wav" \
  -filter_complex "\
[0:v]scale=940:705:force_original_aspect_ratio=decrease,pad=940:705:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1[l0];\
[1:v]scale=940:705:force_original_aspect_ratio=decrease,pad=940:705:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1,fps=30[r1];\
[l0][r1]hstack=inputs=2,scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10[base];\
[base][2:v]overlay=0:780:enable='gte(t,0.8)'[v];\
[3:a]adelay=800|800,apad[a]" \
  -map "[v]" -map "[a]" -t 17 $ENC "$SG/s4.mp4"

# ---- S5 数据卡 16.5s ----
ffmpeg -y -v error -loop 1 -t 16.5 -i "$CD/card_data.png" -i "$AU/n5.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.4,fade=t=out:st=16:d=0.5[v];[1:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 16.5 $ENC "$SG/s5.mp4"

# ---- S6 片尾 6s ----
ffmpeg -y -v error -loop 1 -t 6 -i "$CD/card_end.png" -i "$AU/n6.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.5,fade=t=out:st=5.2:d=0.8[v];[1:a]adelay=1200|1200,apad[a]" \
  -map "[v]" -map "[a]" -t 6 $ENC "$SG/s6.mp4"

# ---- 合并 ----
cat > "$SG/list.txt" <<EOF
file 's1.mp4'
file 's2.mp4'
file 's3.mp4'
file 's4.mp4'
file 's5.mp4'
file 's6.mp4'
EOF
ffmpeg -y -v error -f concat -safe 0 -i "$SG/list.txt" -c copy "$VB/../优化前后对比_2026-10.mp4"
ffprobe -v error -show_entries format=duration,size -of default=noprint_wrappers=1 "$VB/../优化前后对比_2026-10.mp4"
echo DONE
