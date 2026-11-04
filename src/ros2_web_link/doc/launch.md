# 启动、视图配置与静态资源

DDS配置选择补充：检测到当前目录的`DEFAULT_FASTRTPS_PROFILES.xml`时，不注入本包默认XML，由上游处理已有隐式/显式配置；此时需要部署者自行加入所需恢复周期。已有显式环境配置同样优先。

0.8.3使用进程局部的标准Fast DDS恢复配置：rosapi/demo的service响应writer、录制播放器的默认writer均为100ms。已有FASTDDS_DEFAULT_PROFILES_FILE（优先）或FASTRTPS_DEFAULT_PROFILES_FILE会保留，需在自定义XML中合并对应配置；可参照config/fastdds_services.xml与fastdds_replay.xml。桥进程不设置这些默认文件，也不设置RMW_FASTRTPS_USE_QOS_FROM_XML或RMW_IMPLEMENTATION。外部业务服务/相机发布者由其部署方管理；本包不会更改它们。当前默认缓存为0；非零缓存的原生兼容门保留。

`web.launch.py` 是统一入口，只组合 WebSocket 桥、rosapi、可选静态网页和可选无硬件演示节点。默认 `demo:=false profile:=robot`，不启动机器人驱动，不发送 Goal，也不修改 ROS_DOMAIN_ID；它继承当前 ROS 环境。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros2_web_link web.launch.py
```

`self_test.launch.py` 使用同一组合，默认 `demo:=true profile:=demo`。示例话题与服务限定在 `/web_link_demo/`，TF 也发布到该命名空间下的专用话题。`bridge.launch.py` 是统一入口的兼容别名，保留原启动参数。三个入口都不使用后台 shell 进程。

```bash
# 如果默认端口已有手测环境，选用另一组空闲端口。
ros2 launch ros2_web_link self_test.launch.py web_port:=18088 ws_port:=19090

# 只提供桥；不启动网页和演示节点。
ros2 launch ros2_web_link web.launch.py web:=false
```

## 启动参数

- `address`：默认 `127.0.0.1`，同时用于 HTTP 和 WebSocket 监听。需要局域网访问时由操作员显式设置。
- `web_port`：默认 `8088`；`ws_port`：默认 `9090`。页面通过 `/config.json` 获得 WebSocket 端口，主机名取浏览器当前地址。
- `web`：默认 `true`，是否启动静态 HTTP 服务。
- `demo`：统一入口默认 `false`，自测入口默认 `true`，是否启动本包无硬件示例。
- `profile`：`robot` 或 `demo`，决定页面初始观测话题，与 `demo` 是否启动分别配置。
- `config`：默认空字符串；可选本地 YAML 路径，用于覆盖选中 profile 的视图字段。不会通过 HTTP 暴露 YAML 文件或服务器路径。
- `project`：项目用例id，默认随profile选择demo/manual；`project:=peach`选择真实Peach用例。
- `projects_config`：可选项目YAML，按id覆盖/增加内置config/projects.yaml；纯数据，不修改ROS白名单。
- `asset_packages`：逗号分隔的 ROS 包名白名单，默认 `ros2_web_link`。空字符串关闭所有包资源读取。包必须已安装并可通过 ament index 找到。
- `write_queue_size=64`、`incoming_queue_size=100`、`max_message_size=16000000`、`fragment_timeout=30`转发上游队列/消息与分片秒数参数；消息数队列不是总字节内存保证。网页SDK预算通过profile里的`transportLimits`独立配置，单位见io.md。
- `topics_pub_glob`：默认 `['/web_link_demo/*']`；`topics_sub_glob`：默认 `['*']`。
- `services_glob`：默认为 `['/rosapi/topics', '/rosapi/services', '/rosapi/nodes', '/rosapi/action_servers', '/web_link_demo/*']`。
- `actions_glob`：默认 `['/web_link_demo/*']`。

保留上游 Jazzy `parse_glob_string` 的单引号列表语法，例如 `services_glob:="['/rosapi/topics', '/my_node/*']"`。不要使用 JSON 双引号列表。机器人 profile 不会扩大写权限；业务接入应明确设置所需的最小允许范围。上游会自动追加 `/rosapi/*` 服务范围，因此 rosapi 节点的 `params_glob` 仍固定为字符串 `[]`，由节点拒绝旧参数读写接口。

桥、rosapi、网页或启用的演示节点在 launch 尚未停机时退出，都会触发整个组合的正常 shutdown；端口占用、网页构建缺失和配置错误会结束该组合，避免桥或页面单独遗留。进程退出监督不判断页面连接状态或机器人是否停止。结束时按 Ctrl-C 等待所有子进程退出；核对自己启动的进程，保留其他操作员已有的程序。

## `/config.json`

GET 返回当前已验证的视图配置；HEAD 返回相同响应头而不含响应体。响应 `Cache-Control: no-store`。公开字段为：

可选`maxPendingRequests`为1..4096的整数，默认256，限制同一客户端Service/Action本地等待；可选`transportLimits`是部署预算对象；服务端验证其为mapping，各字段由同一SDK校验器在连接前校验。replay.launch.py同样接受`config`转发此profile配置。

```json
{
  "wsPort": 9090,
  "profile": "demo",
  "project": "demo",
  "projects": [{"id":"demo","title":"示例项目","cases":[{"id":"graph","label":"图谱","kind":"graph"}]}],
  "scene": {
    "fixedFrame": "demo_base",
    "robotDescription": "/web_link_demo/robot_description",
    "jointStates": "/web_link_demo/joint_states",
    "tf": "/web_link_demo/tf",
    "tfStatic": "/web_link_demo/tf_static",
    "markers": "/web_link_demo/markers",
    "pointCloud": "/web_link_demo/points",
    "path": "/web_link_demo/path"
  },
  "image": {
    "topic": "/web_link_demo/image/compressed",
    "type": "sensor_msgs/msg/CompressedImage"
  },
  "depthImage": {
    "topic": "/web_link_demo/depth",
    "type": "sensor_msgs/msg/Image",
    "depthScale": 0.001
  },
  "plot": {
    "topic": "/web_link_demo/sensor",
    "type": "std_msgs/msg/Float64",
    "fields": ["data"]
  },
  "diagnostics": "/web_link_demo/diagnostics",
  "rosout": "/rosout",
  "assetPackages": ["ros2_web_link"]
}
```

默认值记录在安装目录 `config/profiles.yaml`。robot profile 使用 `base_link`、`/robot_description`、`/joint_states`、`/tf`、`/tf_static`、`/visualization_marker_array`、`/points`、`/plan`；彩色图为 `/camera/color/image_raw/compressed`，深度图为 `/camera/depth/image_raw`；曲线默认读取 JointState 的 `position.0`，诊断为 `/diagnostics`，日志为 `/rosout`。

`image` 和可选 `depthImage` 接受 `topic`、`type` 及可选正数 `depthScale`。robot 深度图不默认假定单位，页面须让使用者确认缩放；demo 的 uint16 毫米示例显式设为 `0.001`。`depthImage: null` 可关闭该默认入口。`plot.fields` 是非空字符串列表。

自定义文件使用相同的 `profiles` 顶层，只需给出要覆盖的项；选中 profile 的每个 section 按字段合并。例如：

```yaml
profiles:
  robot:
    scene:
      fixedFrame: map
      pointCloud: /camera/points
    depthImage:
      depthScale: 0.001
    plot:
      fields: [position.1, velocity.1]
```

```bash
ros2 launch ros2_web_link web.launch.py config:=/absolute/path/views.yaml
```

示例中的缩放需要与实际相机编码和单位一致。非法 YAML、未知视图字段、空字段列表、无效缩放会在监听 HTTP 前失败。网络端口、profile 名和资源白名单由 launch/CLI 参数决定，YAML 不能覆盖。

## 原生 rosbag2 回放入口

`replay.launch.py bag:=/absolute/bag project:=peach_replay`组合原生rosbag2 player与同一Web工作台。必须先设置独立ROS_DOMAIN_ID（本机样例92），避免与在线相机/TF/JointState重名发布。默认web_port=8388/ws_port=9390、rate=1.0、loop=true；可覆盖address/asset_packages/projects_config。bag目录必须有完整metadata.yaml，rate必须正且有限。

只接受纯观测项目；Topic用例必须显式timestampMode=recorded。播放的话题取该项目Topic用例与view引用，不播放服务请求，不启动业务服务/Action、相机、驱动或demo；Web发布与Action白名单为空，服务仅rosapi。静态TF和URDF按transient_local保留，晚客户端仍可加载。播放器退出时整组退出；Ctrl+C同时清理播放器与Web。包依赖新增Jazzy ros2bag与rosbag2_storage_mcap，无浏览器MCAP播放器或新npm依赖。Peach实录命令和已知丢帧限制见[project-testing.md](project-testing.md)。

## 包资源路由

`GET /assets/<包名>/<相对于包 share 的资源路径>` 通过 `ament_index_python` 定位已安装包。URDF 的 `package://my_description/meshes/arm.stl` 对应 `/assets/my_description/meshes/arm.stl`，启动时须显式包含 `asset_packages:=ros2_web_link,my_description`。

允许 URDF、STL、DAE、OBJ、MTL、GLTF、GLB、BIN、PLY 及常见纹理后缀；包内其他文件类型、未声明包和目录列表均拒绝。URL 解码后拒绝 `.`、`..`、绝对资源路径、反斜杠和控制字符；读取前对真实路径做 share 根包含检查，拒绝符号链接逃逸。指向包外的 symlink-install 资源同样会被拒绝，应将所需资源实际安装到该包 share 内。网页目录也执行真实路径包含检查。

HTTP 仅提供配置和静态文件；没有 ROS 业务转发、参数写入或命令端点。标准 Topic、Service、Action 和参数仍由浏览器 SDK 直接连接上游 WebSocket。

`serve.py` 保留 `--address`、`--port`、`--ws-port`、`--directory`，另提供 `--profile`、`--config`、`--asset-packages`。`--directory` 只修改网页根目录，包资源根仍由 ament index 和白名单决定。部署使用上述 ROS launch 入口统一管理退出。

实现依据：[ROS 2 Jazzy launch 事件](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)、本机 Jazzy `OnProcessExit` / `Shutdown` 与 `ament_index_python.packages` 源码、[Python 3.12 http.server](https://docs.python.org/3.12/library/http.server.html) 和 [PyYAML safe_load](https://pyyaml.org/wiki/PyYAMLDocumentation)。静态服务没有采用标准库默认的跟随符号链接行为，而是在发送前限制真实路径。


## 发送队列与大型数据（v0.4）

本包直接转发上游Jazzy参数：write_queue_size默认64、incoming_queue_size默认100、max_message_size默认16000000、fragment_timeout默认30秒。上游默认write/incoming各1000条；这里收紧默认排队，避免大型观测慢客户端积压。队列按消息数而不是字节限制，max_message_size是接收边界，不能据它宣称整个服务器内存绝对上限。队列溢出由上游记录丢弃，服务超时/断线仍标远端结果未知。

视图配置Image可提供subscription={qos:sensor,compression:cbor,throttleMs:500}；Scene可提供pointCloudOptions（默认sensor/cbor/100ms）和jointStatesOptions（默认sensor/0ms）。YAML只配置订阅，不创建ROS话题重映射参数；实际命名仍使用已有Topic名称或应用launch remap。

CBOR是rosbridge的二进制消息编码，不是JPEG压缩；局域网带宽有限时优先订阅相机image_transport已提供的CompressedImage。16UC1深度原值单位必须明确，不能为节省流量改成有损彩图。GUI可切换JSON对照和图像订阅间隔。性能与可重复基准见[performance.md](performance.md)。

## Service缓存与原始对照

0.8.3起，self_test.launch.py/web.launch.py默认service_client_cache_size:=0；两种缓存模式均受service_client_max_in_flight:=128保护，创建等待service_client_creation_timeout:=5.0。cache0为有界逐调用客户端，适合ARM64首轮验证；以下命令也显式选择该路径：

```bash
ros2 launch ros2_web_link self_test.launch.py service_client_cache_size:=0
```

这些是只读部署参数，需结束旧实例再重启变更；不在运行中切换。缓存要求call_services_in_new_thread=true及已核builtin执行器，未知/Events布局在监听前拒绝，可显式cache0运行原路。既有权限白名单不扩大。

replay.launch.py增加可选bridge_service_client_cache_size（空值不覆盖web默认，0..1024才显式透传）。性能CLI对应--service-client-cache-size；历史--trace-late-client在CLI里选择0，当前Node对该归档模式另有明确拒绝；0.8.3普通默认改为0。历史I62真实录制回放读到显式64/显式0/省略参数的实际值64/0/64，队列均64；每轮JSON/CBOR与完整TF通过。I62短测本身不能关闭长期问题；0.7.0已在I63完成默认1800秒，0.7.1独立冻结/默认1800秒2691RPC零失败见I65。

METADATA可与已核缓存共存；仅核确切诊断wrapper和它的原方法，不采用宽泛unwrap。WAIT/native保持opt-in；旧late-reader保留旧reader诊断已经归档，0.8.3启动前明确拒绝；普通WAIT/native诊断仍可使用。自带service_wait_trace_probe.py的--late-reader默认采用0；--server-metadata --client-metadata在独立进程分别启用server-only及bridge客户端metadata，不给单一进程混传互斥模式。

## 退出宽限（I62起，0.7系列保留）

Ctrl+C后等待launch结束，再核对自有进程和端口。上游2.7.1发布者默认unregister_timeout=10秒，部分断开路径会留下非守护Timer；ROS主循环已返回仍可能等待约10秒。这是发布者保留期，不是Service超时。web.launch给bridge公有sigterm_timeout=15秒，保留上游计时器语义；超过此宽限才由launch升级TERM。手工增大上游unregister_timeout时须在自己的Node launch同步配置退出预算；此处仅验证默认10秒。

性能CLI拥有的launch使用DEVNULL标准输入，仅给根PID初次SIGINT，由非交互launch转发；父守卫等18秒，leaf组等8秒，仍保留TERM5/KILL3兜底。不要对launch组和根重复发初始INT。仅remaining为空不能证明正常退出，须另看bridge exit0及launch.log没有escalation。详见[开发记录I62](development.md)。

无缓存模式也会在创建工作线程之前检查全桥Service准入。`service_client_max_in_flight` 默认128，包含所有连接；Orin首轮可显式从16起步，再以目标机测量调整。`service_client_creation_timeout` 默认5.0秒，控制创建等待，不改变客户端的请求截止。非零缓存有效槽位为min(cache_size,max_in_flight)。工厂/销毁未确认时保留名额，正确恢复边界是正常结束旧节点后重新启动。

profile 的 scene 还支持 robotDescriptionOptions、markersOptions、pathOptions，plot 支持 subscription；字段结构与 pointCloudOptions 相同。工作台模板会完整保留这些订阅选项，避免BEST_EFFORT数据源因转换成RELIABLE而收不到数据。
