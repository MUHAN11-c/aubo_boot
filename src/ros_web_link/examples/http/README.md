# HTTP 示例（curl）

网关默认 `http://127.0.0.1:8095`。本机 curl 若配置了代理，使用 `--noproxy '*'`。

```bash
BASE=${RWL_BASE_URL:-http://127.0.0.1:8095}

# 1) 健康检查（桥 down 时 status=degraded，仍返回 200）
curl -s --noproxy '*' "$BASE/health" | python3 -m json.tool

# 2) WS 转发统计（active 连接 + 进程累计；console 监控卡同源）
curl -s --noproxy '*' "$BASE/api/ivg/ws/stats" | python3 -m json.tool

# 3) 静态入口：/ 307 → /console.html
curl -sI --noproxy '*' "$BASE/" | sed -n '1,5p'
curl -s --noproxy '*' "$BASE/console.html" | head -5

# 4) 机器人 mesh（路径来自 /robot_description 的 package:// URI）
#    例：package://turtlebot3_gazebo/models/turtlebot3_waffle_pi/meshes/...
curl -sI --noproxy '*' \
  "$BASE/api/ivg/robot-mesh/<package>/<relative/path.stl>"

# 5) 可选视频代理（需 web_video_server 在 8089；未启动返回 502/504）
curl -sI --noproxy '*' \
  "$BASE/api/ivg/proxy/web-video/stream?topic=/camera/image_raw"
```

要点：

- `/api/ivg/robot-mesh` 只接受 ament share 内的相对路径，拒绝 `..`/越界；
- `/api/ivg/ws/stats` 的 `totals` 只在连接断开时归账，活跃连接看 `active[]`；
- `/health` 的 `status` 是 `ok`/`degraded`，`foxglove_bridge`/`web_video` 各自 `ok`/`down`；
- 详见 [HTTP/WS API](../../doc/api/http-ws.md)。
