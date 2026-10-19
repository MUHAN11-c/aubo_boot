# Web 示例

浏览器示例源码在包内静态根：

- `web/public/examples/minimal_client.html`
- `web/public/examples/minimal_client.js`

启动网关后直接访问：

```text
http://127.0.0.1:8095/examples/minimal_client.html
```

它通过稳定入口 `/js/ros_web_link.js` 使用：

- `foxgloveRosClient()`：connect / subscribe / publish / callService / getParams
- `createActionClient()`：send_goal + feedback/result 组合

无构建步骤；页面与网关同源，默认连接 `ws://<host>/ws/foxglove`。
