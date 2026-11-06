# 集成到其他 ROS 2 / Web 工程

DDS配置选择补充：检测到当前目录的`DEFAULT_FASTRTPS_PROFILES.xml`时，不注入本包默认XML，由上游处理已有隐式/显式配置；此时需要部署者自行加入所需恢复周期。已有显式环境配置同样优先。

ARM64/Jetson集成前先看[NX 16GB 本机浏览器压力测试](jetson-orin-nx-performance.md)。0.8.3默认cache0，不再在默认启动时触发缓存的x86原生指纹校验；服务与回放采用分进程的Fast DDS恢复配置。非零缓存仍需对应架构的独立审核。NX实机性能尚未验证，0.8.2失败基线与修复复测分别列明。通用包不加入Peach依赖；选择Peach录制合同则需对应的peach_interfaces类型支持与模型资产。

本包分六个可独立使用的入口：SDK（通信）、scene（空间显示）、media（图像/曲线）、audit（项目验收）、graph（有界ROS声明关系）、workbench（可组合显示与配置）。各入口默认共享一个RosClient，也可显式组合控制/观测两个现有客户端；项目通过数据配置声明已有原生ROS接口，不增加业务HTTP代理，不导入Peach/IVG实现。

0.8.0 的自带工作台和应用自建 UI 都可消费同一 controller / registry；共享空间 Scene 和可重复输入由组件管理，不需要将应用改成固定页面。使用步骤、保存格式、内置类型与本地插件扩展见 [workbench.md](workbench.md)。Peach 当前已复用核心 SDK 和桥，业务映射仍在应用侧；本包不迁移其采摘逻辑。IVG 硬件 SDK 的 FK / IK 对比分析入口保留，规划 / 业务语义仍归各项目。

## Peach 当前接线

`peach_bringup/launch/harvest_system.launch.py::_web_bridge` 直接启动本包的
`bridge_node.py`，默认回环 9090，仅订阅三个 observability Web 投影，放行应用
白名单服务和四个 Action，禁止浏览器发布话题。它不启动本包的 rosapi、demo
或独立静态服务器。8090 及 `/vendor/ros2-web-link.js` 由 Peach 观测包托管，
SDK 文件从本包 ament 安装目录读取；应用请求映射在 `peach_web/web/rosbridge.js`。

关闭 `rosbridge_enabled` 或独立运行观测节点时，页面使用 HTTP 对照接口。
两条路径复用既有 ROS 业务契约，HTTP 调试审计不覆盖 WS 直接调用；最终执行
许可由业务节点与臂侧门负责。此接线不改变通用工作台的默认白名单、HTTP 端口
或项目配置。细节见 [Peach Web 架构](../../peach/peach_web/doc/architecture.md)。

## ROS 端

在目标Jazzy工作区放入本包、安装rosdep依赖，显式npm ci后colcon build。桥所在环境必须安装所用自定义消息包，模型包应正常安装share资源。应用launch可Include本包web.launch.py，显式选择端口、asset_packages和命令白名单；默认不会启动机器人或相机。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92  # 替换为与现有应用一致、且无测试残留的域
ros2 launch ros2_web_link web.launch.py profile:=robot project:=my_cell \
  projects_config:=/absolute/projects.yaml asset_packages:=ros2_web_link,my_robot_description \
  topics_pub_glob:="[]" actions_glob:="[]" services_glob:="['/rosapi/*']"
```

上例为只读观测；Service/Action/参数写入须显式加入真实节点端点，仍由应用自己的权限门决定是否执行。端口/地址/白名单是部署参数，话题名称使用原生ROS命名/remap及项目数据配置。真实机械臂与SetIO须另行授权。

projects.yaml使用`projects: {my_cell: {id: my_cell, title: ... , view: ..., cases: [...]}}`。可从config/projects.yaml的demo或Peach样本复制，再替换接口与断言；无需复制audit执行器或重写通信库。在线数据采用timestampMode: live；录制回放显式recorded，不把历史数据当设备当前状态。每项交互默认关闭，选中后才执行；参数往返及Action收尾均确认结果。

## 浏览器端

ROS安装目录share/ros2_web_link/dist/web包含自带依赖的浏览器脚本、source map和THIRD_PARTY_NOTICES。可复制到自身静态站点同源/vendor/，只加载所需组件；无需CDN、运行期Node或Fastify代理。模型资源应经同源/assets/白名单路由，或者由应用按同一接口提供只读资源。

```html
<script src="/vendor/ros2-web-link.js"></script>
<script src="/vendor/ros2-web-media.js"></script>
<script>
const client = new Ros2WebLink.RosClient({url: 'ws://127.0.0.1:9090'});
let view;
async function mount(root) {
  try {
    await client.connect();
    view = new Ros2WebMedia.ImageView(root, client, {
      topic: '/camera/color/image_raw', type: 'sensor_msgs/msg/Image',
      subscription: {qos: 'sensor', compression: 'cbor', throttleMs: 100},
      onStatus: text => console.log(text)
    });
    view.start();
  } catch (error) {
    unmount();
    throw error;
  }
}
function unmount() { view?.dispose(); client.dispose(); }
</script>
```

本例由一个挂载实例拥有client；多个组件共享时，由父页面拥有并最终dispose客户端，各子组件只dispose自己的view。

组件不绑定React/Vue生命周期；在对应框架mount时创建，在unmount时dispose。多个组件可共享client；同话题类型/QoS/限频/编码应一致，SDK对冲突明确报错。小数据Topic/Service/Action仍用核心SDK，不需要加载Three/uPlot。

打包工程先执行`npm run build && npm run api`，再用`npm pack`生成本地tgz，安装命令为`npm install /absolute/local-ros2-web-link-0.8.3.tgz`。六个公有入口为`@local/ros2-web-link`、`/scene`、`/media`、`/audit`、`/graph`、`/workbench`。包内含dist、.d.ts、自有源码、配置、ROS启动源码和许可证；private=true只防止误发布，不限制本地文件安装。npm构建与ROS colcon部署各自独立。

公有声明可用于严格`NodeNext`或`Bundler`工程，消费端无需设置`skipLibCheck:true`。公开传输预算类型独立于内部roslib适配器，运行时仍使用锁定的roslib，不修改上游类型文件。`npm run api`在临时包外工程中检查六个exports、两种严格模块解析、错误类型拒绝和浏览器打包，并清理临时目录；完整tgz安装及ROS运行仍需另行验收。离线安装要求缓存同时具备锁定tarball及消费端解析所需的元数据，缓存不全时不能把`--offline`失败判为运行代码故障。

从tgz修改并重建时，先解包到普通开发目录，恢复构建锁并安装开发依赖（Node22+）：

```bash
mkdir ros2_web_link_source
tar -xzf /absolute/local-ros2-web-link-0.8.3.tgz -C ros2_web_link_source --strip-components=1
cd ros2_web_link_source
cp dist/build-package-lock.json package-lock.json
npm ci --include=dev
npm run check
npm run build
npm run observe
```

npm按官方规则不直接打包根package-lock.json；dist/build-package-lock.json是同一文件的原样副本，构建时生成，无第二份依赖配置。tsconfig.json随包提供。缓存充分时可给npm ci加`--offline`；本机验证还用`--ignore-scripts --no-audit --no-fund`禁止安装脚本/网络审计。不要在业务工程的node_modules中直接修改依赖并当作源码仓库；ROS安装目录的source便于查阅，完整源码开发使用原src包或本节tgz。

## 可选控制与观测分连接

时钟选择也必须显式：ROS节点use_sim_time=true时，Scene配置useSimTime:true；真实相机/常规mock默认false。独立TfBuffer需clockTopic:'/clock'才接收回退通知。0.6.18改变原来自动侦听/clock的默认；已有回放调用者应补显式选项。本包Peach回放配置已补齐，勿只升级脚本却继续依赖旧默认。更改Scene模式时停止并重新创建组件；不通过页面改写ROS节点参数。RPC超时仍是单调时间，与该显示模式无关。

默认使用一个连接。自测页在连接前勾选“分离观测连接（大数据）”才启用第二条WS，活跃期间锁定选择；两侧首次握手完成前不能发起新操作。停止等待、取消、断开保持可用。该选项不改变桥队列、DDS配置或项目权限。

已有工程直接组合公有客户端，不需要新的代理或通信门面：

```js
const control = new RosClient({url});
const observation = new RosClient({url}); // URL字符串须与control完全相同
try {
  await Promise.all([control.connect(), observation.connect()]);
  const report = await runProjectChecks(control, project, selectedIds,
    {observationClient: observation, signal});
  console.log(report.connectionMode); // separate；省略选项时为shared
} finally {
  // 待项目运行及其取消/恢复收尾结束后，调用者释放两侧。
  observation.dispose();
  control.dispose();
}
```

SceneView、ImageView、SeriesPlot可直接接收observation；手动发布、Service、参数和Action使用control。audit自行管理本次订阅/监听，未取得连接所有权；mockGuard在control上核对。任一连接中断会结束当前用例，远端取消/参数恢复仍通过control确认。不要断开control后把远端任务自动视为已取消。

预算按客户端分别生效：默认各256个本地等待，页面合计显示上限512，而非全局共用256；消息/分片/发送积压等预算也各自独立。两条WS仍共享同一rosbridge进程与DDS，不能承诺服务端或所有大流隔离。demo已核对，完整录制无诊断长测仍有彩图停顿，详见development.md I35–I36。

上段停顿来自0.6.15的双连接/实验XML配置，历史结果保留。0.7.0已在默认共享/q64/无XML完成1800秒，不能据此推导所有双连接/网络/设备配置也通过。新项目先保持默认参数验证原生数据，再独立比较编码、频率或连接组合，避免同时改多项后无法解释结果。

## 0.7系列缓存与部署兼容

0.8.3 ROS桥默认cache0，使用有界逐调用客户端。显式非零缓存时才按Node寿命复用最多指定数量的Service Client；两种模式都在创建线程前使用全桥Service准入上限128，未消费的无缓存实体任务继续占名额；三个只读参数见launch.md。缓存适配明确核本机rclpy7.1.12 builtin关键源码与二进制，适配未知时在监听前报错；其他已支持的Jazzy rclpy环境可显式`service_client_cache_size:=0`用公开API逐调用路径，rosbridge仍固定2.7.1。升级库、换架构或修改其源码时，应复核相关API并重跑，不直接更换哈希绕过门。浏览器六入口与该Python缓存独立，应用也可对接自己已验证的标准rosbridge部署。

自定义接口仍须安装在桥的ROS环境；本包不要求业务包import其Python模块，也不为每个业务服务生成包装。模型用asset_packages与package://资源，项目差异用projects_config/profile或自己的SDK调用表达。关闭本地等待不取消远端Service；Action取消和使能等业务恢复由应用确认。

本机0.7.0来源归档见development.md I63，0.7.1图像预算补丁/正式1800秒与最终归档绑定见I65–I66；归档是否验收以同目录verification及版本为准。源码包里的share/source用于审阅；运行开发探针应使用原src或解包且恢复lock后的源码目录，不能假设安装审阅副本的相对../dist完整可执行。

## 频率、数据规模与故障

ClientOptions可直接指定`transportLimits`；自测页也可用`config:=/absolute/views.yaml`（replay入口同样转发）部署预算，无需改网页源码：

```yaml
profiles:
  robot:
    maxPendingRequests: 256
    transportLimits:
      maxMessageBytes: 16000000
      maxFragmentBytes: 32000000
      fragmentTimeoutMs: 30000
      maxSendBufferedBytes: 16000000
```

省略的字段使用SDK默认值，完整字段/范围见io.md；无效预算在连接前明确报错。服务端`max_message_size/write_queue_size`、SDK消息预算与Image/Scene自身解析上限各自生效，增加其中一项不会自动增加其他项。模板用于表达部署配置，不是建议通过不断加大缓存解决过载。

业务工程直接构造`new RosClient({url,maxPendingRequests:64})`即可限制同一客户端的Service/Action本地等待，默认256、有效1..4096；构造后的该预算不可修改。网页用相同profile字段，等待统计显示“当前 / 上限”。满额不发送、不排队；Service以Promise拒绝、sendGoal同步抛出REQUEST_LIMIT，因此Action调用及await goal.result应放在同一个try/catch中。图谱查询并发四个服务，需要至少四个空闲名额。不要以轮询自动重试写参数、服务或Action；先从应用状态核对远端是否仍在执行。本地超时释放名额不等于取消远端工作。

- Image默认CBOR+500ms，可覆盖subscription；16UC1必须明确depthScale，32FC1按米。优先使用相机已有image_transport压缩输出做普通彩色预览；CBOR是二进制编码，不是图像压缩。
- Scene点云默认CBOR+100ms，pointCloudOptions可指定QoS；maxPoints默认100000，仅对显示做有界采样。源消息仍为原生PointCloud2，字段/步长/大小端/颜色按实际数据解码。
- TF静态可靠transient_local/depth100，动态保留时间语义，不默认整条限频丢差量。积分/重建应在ROS侧tf2按精确stamp执行；Web显示不能代替算法TF契约。
- 不根据“best_effort名字更适合传感器”就假设任何大流都能收到；核对真实发布QoS、DDS与网络。已验证的大点云Reliable样例是环境配置，不是对所有项目的强制修改。
- 断线不重放命令，超时不代表远端取消。图像/场景/曲线/订阅都有dispose；原始预览只显示有界摘要，避免逐帧格式化大数组。
- 自测页的结果、项目摘要和操作记录只显示有界快照；操作记录导出也保留摘要。需要完整项目证据时使用“导出验收 JSON”，SDK返回值与该项目报告按原契约保留完整数据。`npm run preview`显式核对格式/保留边界，无ROS和浏览器；实际页面与完整导出需另行核对。
- 用performance.md中的有界CLI与项目网页做验收，保留失败、数据规模、资源统计和未执行项。渲染首载、慢客户端、网络与Android的专项结果需独立记录。

自定义压缩图标准预览若需更小像素门，可复制现有项目契约到projects_config并设置完整view.image及其maxPixels，或直接调用media入口。此字段不属于profile YAML schema，普通图像表单仍使用ImageView默认4M像素；不要在config:=views.yaml中添加后期待服务器接受。新入口只做数据/header预检，Blob URL归业务页面所有；错误提示可恢复合法图，不自动扩预算或改图像单位。
