# peach_vision_core 数值与 API 边界

本包没有 ROS 图端口。这里记录共享 Python API 的单位和限制；跨包消息字段仍以 peach_interfaces 为准。

## 几何与袋地标

`normalize_depth_to_uint16_mm(depth, depth_scale_unit)` 返回毫米 uint16：整数输入的有效值乘比例因子，原始 0/65535 保持无效；浮点输入视为米。`camera_intrinsics` 从 K 读取 fx/fy/cx/cy，要求这四项有限且焦距为正；宽高由调用方传入并转成整数，不在此函数校验完整 K 或标定尺寸。反投影输出米制 `(N,3)` 点，TF 约定为目标系←源系的 4×4 矩阵；方向只乘旋转，点同时加平移。

`estimate_bag_landmarks` 返回同曝光 BagLandmarks，袋轴方向为 bottom→neck。PCA 各向异性或相对重力不可靠时使用重力先验；显式 axis_hint 保持独立调用语义。`condition_bag_landmarks` 只重构当前曝光的退化几何，不保存时间、身份或许可；调用方必须以原完整评估作为资格下限，保留较大直径和保守误差。直径标量不代表两种轴线空间包络的并集。

## 预算、能力与模型身份

`evaluate_sleeve_cut` 返回 radial_available/error/margin、axial_error/margin、结构性/轴角诊断、reason、failure_code，以及 sleeve_ok/cut_ok。当前实现 sleeve_ok 恒为 true，cut_ok 只比较 cut_to_fruit_m 与 fruit_safety_clearance；径向/轴向余量为诊断，不在此函数否决两个布尔值。负余量可以与 sleeve_ok/cut_ok 为真并存，reason 也不保证与布尔值一一对应。

`domain.budget.evaluate_capabilities` 将布尔值投影为 VALID/INVALID/UNKNOWN；geometry 与 pregrasp 在这个通用评估器内是 VALID 占位，不包含独立拟合状态、IK 或场景碰撞查询。上层重建还会据几何与观测证据组装最终能力，执行端另有门。

`allowed_from_capabilities` 只合并 geometry、sleeve、cut 三项 VALID，显式忽略 pregrasp；不能把 allowed 当作完整运动授权。模型身份为 run_id、scene_epoch、target_id、model_revision、tool_profile_id、calibration_revision、config_revision。完整性检查要求六个字符串非空，相等比较还包含 scene_epoch。`model_executable` 只检查完整性、valid_until>generated 和未过期；preview 可旁路这些检查，它不验证能力、可达性或生成时刻是否在未来。

`evidence.may_commit_identity` 只接受调用方提供的精确 TF 事实；遮挡/走廊输入缺失返回 unknown，不得把空数据解释成 clear。

## Worker、时钟与落盘

`BoundedWorker(process, capacity, drop_oldest)` 串行执行任务。满队列时可替换最旧任务或拒绝新任务并计 dropped；discard_pending 不停止在途任务；close(drain=False) 丢弃排队项后仍等待在途任务完成。异常记日志，线程继续处理。

`HarvestDataStore` 使用容量 512 的单写线程队列保存事件和图像，队列满时同步降级；JSONL 用文件锁避免行撕裂，极端背压仍可能改变事件顺序。close(drain=True) 先等待所有队列任务完成，再投递哨兵并最多等待线程 5 秒；drain=False 不清空队列，不能保证立即退出。调用方应明确销毁顺序和退出预算。

目录根由 peach_common.paths 解析。批次运行时 base_dir 由节点绑定到 `runs/<run_id>/perception_data`，其下按感知 harvest_run_id 建轮目录；无批次时使用独立根。不要将库的事件目录与调度账本视为同一写入者。
