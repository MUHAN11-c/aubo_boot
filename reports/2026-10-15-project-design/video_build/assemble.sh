#!/bin/bash
# 优化前后对比视频 · 数据驱动版（8 段：片头/感知/v1v2图表/运动链+阶段条/自动轮图表/相机链/指标卡/片尾）
set -e
VB="$(cd "$(dirname "$0")" && pwd)"
ROOT="/home/mu/Desktop/aubo_e5_jazzy_ws"
VR="$ROOT/runs/compare_rviz_20261012"
ST="$ROOT/_archive/code_cleanup_2026-10-12/src/peach/peach_stereo/test/report"
AU="$VB/audio"; CD="$VB/cards"; SG="$VB/segs"
ENC="-c:v libx264 -crf 20 -preset medium -pix_fmt yuv420p -r 30 -c:a aac -b:a 160k -ar 48000"

# ---- S1 片头 10.5s ----
ffmpeg -y -v error -loop 1 -t 10.5 -i "$CD/card_title.png" -i "$AU/n1.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.5,fade=t=out:st=10:d=0.5[v];[1:a]adelay=700|700,apad[a]" \
  -map "[v]" -map "[a]" -t 10.5 $ENC "$SG/s1.mp4"

# ---- S2 立体对比 22s（下排两主视图：密集 vs 稀疏） ----
ffmpeg -y -v error -ss 4 -t 22 -i "$ST/fig_f_compare.mp4" -i "$CD/label_s2.png" -i "$AU/n2.wav" \
  -filter_complex "\
[0:v]split=2[s2a][s2b];\
[s2a]crop=535:455:200:290,scale=-2:780:flags=lanczos[l0];\
[s2b]crop=495:470:940:285,scale=-2:780:flags=lanczos[r0];\
[l0][r0]hstack=inputs=2,scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1[base];\
[base][1:v]overlay=0:780:enable='gte(t,1)'[v];\
[2:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 22 $ENC "$SG/s2.mp4"

# ---- S3 v1/v2 数据对照 21s（账本实测图表） ----
ffmpeg -y -v error -loop 1 -t 21 -i "$CD/chart_v1v2.png" -i "$AU/nb.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.4,fade=t=out:st=20:d=0.6[v];[1:a]adelay=1200|1200,apad[a]" \
  -map "[v]" -map "[a]" -t 21 $ENC "$SG/s3.mp4"

# ---- S4 冻结链完整周期 46s（上：主视图+检测面板；下：阶段时间条+实测徽标） ----
ffmpeg -y -v error -ss 8 -t 46 -i "$VR/v1_frozen_full.mp4" -i "$CD/strip_s4.png" -i "$AU/n3.wav" \
  -filter_complex "\
[0:v]split=2[bs][bd];\
[bs]crop=1034:812:366:58,scale=-2:780:flags=lanczos,pad=1330:780:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1[main];\
[bd]crop=364:342:0:558,scale=-2:500:flags=lanczos,pad=590:780:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1[dbg];\
[main][dbg]hstack=inputs=2,setsar=1[base];\
[base][1:v]overlay=0:780:enable='gte(t,0.5)'[v];\
[2:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 46 $ENC "$SG/s4.mp4"

# ---- S5 自动轮+稳定性图表 20s ----
ffmpeg -y -v error -loop 1 -t 20 -i "$CD/chart_rounds.png" -i "$AU/nc.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.4,fade=t=out:st=19:d=0.6[v];[1:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 20 $ENC "$SG/s5.mp4"

# ---- S6 相机流 + 同帧实拍对照 19s ----
ffmpeg -y -v error -ss 5 -t 19 -i "$ST/percipio2_rvizwin.mp4" \
  -loop 1 -t 19 -i "$VB/../assets/final_cmp_live.png" \
  -i "$CD/label_s4.png" -i "$AU/n4.wav" \
  -filter_complex "\
[0:v]crop=705:728:493:110,scale=836:863:flags=lanczos,pad=950:900:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1,fps=30[l0];\
[1:v]scale=2560:1920,zoompan=z='min(zoom+0.0009,1.25)':d=570:x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':s=940x705:fps=30,scale=940:705,setsar=1[r1];\
[r1]pad=950:900:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10[r1p];\
[l0][r1p]hstack=inputs=2,scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=0x0e1a10,setsar=1[base];\
[base][2:v]overlay=0:780:enable='gte(t,0.8)'[v];\
[3:a]adelay=800|800,apad[a]" \
  -map "[v]" -map "[a]" -t 19 $ENC "$SG/s6.mp4"

# ---- S7 数据卡 16.5s ----
ffmpeg -y -v error -loop 1 -t 16.5 -i "$CD/card_data.png" -i "$AU/n5.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.4,fade=t=out:st=16:d=0.5[v];[1:a]adelay=1000|1000,apad[a]" \
  -map "[v]" -map "[a]" -t 16.5 $ENC "$SG/s7.mp4"

# ---- S8 片尾 6s ----
ffmpeg -y -v error -loop 1 -t 6 -i "$CD/card_end.png" -i "$AU/n6.wav" \
  -filter_complex "[0:v]scale=1920:1080,setsar=1,fade=t=in:st=0:d=0.5,fade=t=out:st=5.2:d=0.8[v];[1:a]adelay=1200|1200,apad[a]" \
  -map "[v]" -map "[a]" -t 6 $ENC "$SG/s8.mp4"

# ---- 合并 ----
cat > "$SG/list.txt" <<EOF
file 's1.mp4'
file 's2.mp4'
file 's3.mp4'
file 's4.mp4'
file 's5.mp4'
file 's6.mp4'
file 's7.mp4'
file 's8.mp4'
EOF
ffmpeg -y -v error -f concat -safe 0 -i "$SG/list.txt" -c copy "$VB/../优化前后对比_2026-10.mp4"
ffprobe -v error -show_entries format=duration,size -of default=noprint_wrappers=1 "$VB/../优化前后对比_2026-10.mp4"
echo DONE
