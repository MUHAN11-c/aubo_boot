#!/bin/bash
# 四源拼接对比视频：每前端 composite(叠加图|彩色|深度JET 全尺寸) 上 + rviz(3D 点云+标记) 下，两前端横排
# 输入 /tmp/e2e_live/report/{hh4,percipio}_{composite,rvizwin}.mp4（record_session.sh 产物）
set -e
R=/tmp/e2e_live/report
ffmpeg -y -loglevel error \
  -i "$R/hh4_composite.mp4" -i "$R/hh4_rvizwin.mp4" \
  -i "$R/percipio2_composite.mp4" -i "$R/percipio2_rvizwin.mp4" -filter_complex \
"[0:v]scale=960:-2,drawtext=text='peach_stereo hh4 13.5gps':x=8:y=18:fontsize=22:fontcolor=green:box=1:boxcolor=black@0.6[c0];\
[1:v]scale=960:-2[r0];[c0][r0]vstack[L];\
[2:v]scale=960:-2,drawtext=text='percipio device-depth 2.4fps':x=8:y=18:fontsize=22:fontcolor=green:box=1:boxcolor=black@0.6[c1];\
[3:v]scale=960:-2[r1];[c1][r1]vstack[R];[L][R]hstack" \
  -pix_fmt yuv420p "$R/fig_f_compare.mp4"
ffmpeg -y -loglevel error -i "$R/fig_f_compare.mp4" -vf 'select=eq(n\,30)' -frames:v 1 -update 1 "$R/fig_f_poster.png"
ls -la "$R/fig_f_compare.mp4" "$R/fig_f_poster.png"
