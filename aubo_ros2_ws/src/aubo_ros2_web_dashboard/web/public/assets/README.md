# 静态资源（`assets/`）

HTTP 文档根为 `web/public/`，故在页面里引用本目录下文件时使用**站点根相对路径**，例如：

- `assets/images/example.png` → `http://<主机>:<WEB_DASH_PORT>/assets/images/example.png`
- CSS：`url("/assets/images/example.png")` 或 `url("../assets/images/example.png")`（视样式表路径而定）

| 子目录 | 用途 |
|--------|------|
| `images/` | 门户/面板位图：PNG、WebP、SVG 等 |
| `icons/` | favicon、小图标等 |

当前 IVG 页面以 CSS、Canvas 与 `web_video_server` MJPEG 为主，可不引用图片；新增文件后执行 `colcon build` 安装到 `share/aubo_ros2_web_dashboard/web/public/assets/`。
