# 项目自测与交互使用

自测页现在可以选择项目与测试项。Peach 配置读取真实相机、mock机械臂及应用节点；demo配置单独保留。报告区分通过、失败、取消和未执行，不以demo结果代替Peach验收。项目差异在 `config/projects.yaml`，SDK和audit不import Peach业务；Gazebo路径已暂停。

## 当前优先入口：无硬件与已录制项目

真实相机已经关闭时，不启动相机或Gazebo。第一次使用先按README执行`ros2 launch ros2_web_link self_test.launch.py`，打开8088并连接：默认6项只读项目用例，2项交互需手选；展开“独立demo通信回归”运行12项基础回归，再按testing.md核对各表单。需要实际Peach大数据时，使用下文的录制入口；完整云选择`peach_replay_large`及60秒宽范围bag，运行10项只读，不会调用Peach业务服务。

页面基础步骤中的17项与3项交互专指Peach在线配置，不能套在demo/录制项目。普通“运行所选用例”是当前项目验收；demo按钮只验证demo，未启动demo的Peach回放环境不能据它判项目失败。Android按用户2026-10-30指示本轮暂不测试；桌面LAN自访问和989×900触摸模拟仅记录为环境/布局验证。

## 页面怎么用

1. 打开对应网页，选择 **Peach · 真实相机 + mock机械臂**，核对WebSocket连接地址，点击连接。项目选择切换配置，不会擅自更换当前连接。
2. 默认勾选17项只读检查。可逐项勾选/取消；“只选只读项”恢复默认，“清空选择”便于单项复查。
3. 点击“运行所选用例”，观察每项状态、耗时与证据。运行前会暂停手动Topic/图像/3D/曲线订阅，避免同话题不同QoS或throttle造成误报；检查后可重新开始观测。
4. 如需交互，单独勾选3项中的所需项：mock SurveyScene、BeginScene、观测参数往返。前两者按Peach约定先Survey再BeginScene执行；参数往返自动恢复原值。不会自动执行RunHarvest、ExecuteTarget或SetIO。
5. “停止并收尾”取消本轮当前Action、恢复参数、关闭测试使能，并等待确认。取消失败、恢复失败或远端状态未知都记FAIL。交互运行期间有离页保护；请先停止并确认结果再关闭页面，浏览器强制关闭/网络断开不能保证远端恢复。
6. 展开行内证据查看原生接口、参数、源时间、解码尺寸、频率、反馈和终态；页面分析保留具体失败原因。`degraded=true`不能当场景质量通过，status=4不等于采摘成功。
7. 展开“查看本轮 JSON 报告摘要（完整内容可导出）”检查有界预览，或用“导出验收 JSON”取得完整项目报告。报告包含项目、连接URL、runId、所选项、未执行项与真实证据，不嵌入图像/点云大数组。下方操作记录及其导出也只保留摘要；需要完整项目证据时使用项目导出。“填入面板”只预填通用工具，不自动调用接口。

## 启动真实相机 + mock 对照

终端一按Peach测试指导source Jazzy、工作区和aubo_py3.12（numpy1.26.4），使用独立DDS域：

```bash
export ROS_DOMAIN_ID=91
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo force_enabled:=false \
  autostart:=false skip_reconstruction:=true reconstruction_enabled:=true \
  rosbridge_enabled:=false observability_port:=8090 rviz:=true
```

终端二source Jazzy与工作区，使用同域，启动本包，不需要demo节点：

```bash
export ROS_DOMAIN_ID=91
ros2 launch ros2_web_link web.launch.py profile:=robot project:=peach demo:=false \
  web_port:=8288 ws_port:=9290 asset_packages:=ros2_web_link,aubo_description \
  topics_pub_glob:="[]" actions_glob:="['/peach_arm/survey_scene']" \
  services_glob:="['/rosapi/*', '/peach*/get_*', '/peach*/list_parameters', '/peach*/describe_parameters', '/peach_scene_perception_node/begin_scene', '/peach_observability/set_parameters', '/peach_supervisor/set_enables']"
```

打开 `http://127.0.0.1:8288`。只读验收可去掉最后两个写服务及Survey Action白名单；选中相应交互时将明确报失败，网页按钮不是权限边界。真实相机与真实机械臂是独立选择：此入口连接真实相机，机械臂始终mock；项目不会启动Gazebo。

mock动作先从实际ROS robot_description验证 `mock_components/GenericSystem`，经应用 `SetEnables` 使能；grasp/tool始终false。服务accepted后还要确认臂侧status生效，结束确认三项全关。此项测试通信与mock到位，不证明固定实相机随mock腕移动，也不作外参、重建精度、真实抓取或功能安全结论。

默认重建策略为skip=true，capture.auto_mode=false；不为了取得绿色结果而开启重建或放宽业务门。完整组合中若Survey返回曝光/场景超时，保留原始终态与message，并按Peach时序和数据新鲜度排查。

## 关闭相机后：录制数据自测

原生rosbag2已经保存真实RGB-D、内参、Peach观测、TF、URDF和JointState。回放时关闭原相机/机器人栈，使用独立DDS域；无需连接相机或安装驱动来运行本包。自定义Peach消息类型及模型资源仍须已安装在工作区。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
export ROS_DOMAIN_ID=92
ros2 launch ros2_web_link replay.launch.py \
  bag:=/home/mu/Desktop/aubo_e5_jazzy_ws/runs/ros2_web_live_camera_mock_20261028/capture_before_camera_off/peach_rgbd_reliable_30s \
  project:=peach_replay asset_packages:=ros2_web_link,aubo_description \
  web_port:=8288 ws_port:=9290
```

打开8288页面，默认项目为“Peach · 录制数据回放”，连接9290，运行10项只读检查。可清空后单选，停止、重跑、展开证据、导出报告，再开始3D/图像/深度/曲线。默认loop=true；循环处/clock倒退会清动态TF和曲线历史。运行终端Ctrl+C同时退出播放器与Web，不留下后台进程。地址、端口、rate、loop和projects_config均可显式修改；默认端口为8388/9390，上例为保持原工作台地址而覆盖。

Peach回放的彩色/深度检查和图像view显式使用Reliable、CBOR、500ms。录制写端为Reliable；本机FastDDS原生Best Effort大图存在严重丢帧，已用不经过Web的官方ros2 topic echo对照。同一bag/single_cloud的Web负载只改变RGB-D QoS后可持续通过；这不是要求其他项目的传感器都用Reliable。若设备仅提供Best Effort，Reliable订阅不匹配；保留sensor或配置兼容策略，并优先测试已有image_transport压缩输出。

“填入面板”后核对QoS下拉框：预设必须与用例一致，显式对象显示自定义策略与depth；编码/间隔/深度单位同样保留，再点击订阅/显示。未指定的TopicSpec按验收引擎sensor/none/0填入，不能把组件默认的500ms当作项目断言。

回放不会重建Peach的参数/生命周期/Service/Action服务端，不能选在线项目来替代离线验收。回放项目没有任何交互写入用例，桥仅开放rosapi服务，禁止Topic发布和Action。报告timestampMode=recorded、liveFreshnessChecked=false，保留源时间，不将旧帧标为实时新鲜。在线Peach配置仍使用原新鲜度门。

本次主数据集约435MiB、29.24秒，彩色366帧、深度365帧、精确同stamp配对364组，21个TF帧连通；深度0.00025m/原值。录制器报告82条传输丢失，内参/图像数量存在少量差异，不是无损标定或重建精度基准。另保留约1.1GiB的60秒宽范围录制及其丢帧日志。质量、原生metadata、录制命令、校验和与报告位于同级capture_before_camera_off目录。

## 加入其他项目

`config/projects.yaml`提供Peach在线、Peach录制数据、完整点云回放、demo、manual五个数据配置。通过 `projects_config:=/absolute/projects.yaml` 按id覆盖/增加项目，再以 `project:=my_project` 选择。自定义文件使用同一 `projects` 顶层，每项id必须与键一致；连接和ROS侧白名单仍由launch参数决定。任意新项目可以只写标准消息/服务/Action名称与断言，无需修改核心源码或安装Peach业务包。

页面保留通用Topic、Service、Action、参数、Lifecycle与观测工具。demo项目需要连接已启动demo的桥（例如8088对应9090），其结果标为demo通信回归；当前Peach域未部署demo时不能据它验证Peach。

## Audit契约

`audit/index.ts`导出`runProjectChecks(client, project, selectedIds, {signal?,onResult?,onProgress?}) => Promise<ProjectReport>`，以及类型/配置校验函数。client为现有RosClient，不创建第二条WebSocket。

ProjectSpec字段：id/title/description，cases数组，可选view（scene/image/depthImage/plot等现有视图配置）；mockGuard={topic,type,qos,field,includes}。Case公共字段id/label/group/kind/description/interactive/defaultSelected/timeoutMs/mockOnly。

用例kind与字段：
- graph：nodes/topics/actions字符串数组。
- topic/image/cloud：topic={name,type,qos,throttleMs,minSamples,maxAgeMs,requiredFields,expected,jsonField,encoding,depthScale,minPoints,width,height,minHz,timestampMode}。timestampMode默认live，检查浏览器与源时间的新鲜度；recorded保留录制时间且报告liveFreshnessChecked=false，不代表设备在线。回放时间倒退时重新收集本段采样。验证不同header源时间（无header用接收时间），采样新鲜度/频率；image用现有decodeImage，cloud用decodePointCloud。不保留原始大数组至报告。
- parameters：node/names/expected（参数名到期望JSON值）；读取值+描述。
- lifecycle：node/state，原生getState。
- tf：target/sources，使用现有TfBuffer查询，记录帧列表/失败原因。
- interface：endpoint/interfaceKind/interfaceType，实际类型+完整接口定义。
- service：service={name,type,request,expected}，expected为响应普通字段路径到值的等值断言。
- parameterRoundtrip：parameter={node,name,type,value}；保存旧值→写入→读回→finally恢复并确认。恢复失败必须FAIL并保留错误。
- action：action={name,type,goal,requireFeedback?}；setup/cleanup为ServiceSpec数组，afterSetup/afterCleanup为TopicSpec数组（可JSON.parse String.data并等值断言臂侧实际使能）；成功终态4，反馈可验证；取消仅当前句柄，停止后有界等待取消终态。setup部分失败也必须cleanup。

请求JSON中的精确字符串`$runId`替换为本轮唯一ID（不求值）。普通字段路径仅解析点/数组索引，不eval。mockOnly必须从ROS实际robot_description读取mockGuard并匹配mock插件；不能只信项目名。各用例执行前检查连接/取消，超时/断线标明确结果未知。只读项默认选中，交互项必须显式选中。

报告包含projectId/title、连接URL、runId、起止时间、用例结果/证据/耗时、总计及尚未执行项；不含图像/点云大数组，不把FAIL或未执行隐藏成PASS。

## 功能口径

Peach默认17项：节点/话题图谱、真实彩色与深度、JointState、目标集合、感知点云、完整TF、使能参数与实际使能快照、重建策略、四节点生命周期、原生服务/Action定义。三项交互默认不选：SurveyScene、BeginScene、观测参数往返恢复。未选项总是NOT_RUN，选择动作仅证明相应用例，不等于完整采摘流程。

本机联调发现并修复：JointState与曲线QoS冲突；多静态writer需要depth100；rosbridge2.7.1晚客户端首样本后销毁reader导致TF回放不全；WebSocket静态TF队列1覆盖突发回放。验证含多客户端、重订阅、重连及reader释放。细节见source.md，测试命令与结果见testing.md。


## v0.4 编码、限频与完整点云

图像页面可显式选择CBOR/JSON、QoS和订阅间隔，Topic面板也可调整编码与间隔。SceneOptions JSON支持pointCloudOptions和jointStatesOptions，复用SDK的TopicOptions；不新增业务控制面。原始标准数据视图跟随项目，并用有界摘要显示大型数据。

选择`peach_replay_large`时，bag应使用capture_before_camera_off/peach_rgbd_mock_60s（宽范围录制），而非不含完整相机云的30秒主包。此样例显式Reliable订阅本机大数据；启动参数示例：`project:=peach_replay_large`。源约17万点/4.1MB每帧，显示均匀采样至约8.6万点，保留原始计数与codec证据。不将源点数与显示采样点数混淆。

标准Peach回放仍适合轻量自测；两个模式不能互相代替数据规模验收。详细成功、失败、资源和渲染未完成项见performance.md。
