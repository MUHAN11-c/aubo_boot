# 真实相机 + 固定相机 TF + mock FULL 验证

真实相机 stereo、bite_shear_v1、mock GenericSystem、域83、use_sim_time=false。固定 base_link→camera_link 使用拍照位 TF，原随腕外参关闭；未改驱动、真实手眼标定、刀具尺寸、误差预算或抓取行程。

FULL action 正常结算，Result.success=false / no_targets_succeeded。耗时38.0s，发现1、尝试1、成功0。completion_level=0，无预抓取/套入/剪切到位证据。tool IO关闭，不是完整采摘成功。

重建8.213s，1帧2392点，TSDF926点、mesh1535顶点；refit ACCEPT，RMSE1.858mm、内点率0.679。关键点与点云轴冲突约6.07°，融合侧移20.116mm。外袋辅助球半径42.864mm，cut_to_fruit=-3.651mm，使cut_capability=INVALID / cut_plane_fruit_clearance；geometry、pregrasp、sleeve能力VALID，聚合allowed=false，在Finalize质量门拒绝接触。负预算仅诊断，不是主拒因。

补视LIN规划也失败；独立完整轴向路径查询返回sleeve_no_ik，不能把它解读为已执行失败或全部规划均不可达。后续两个探针在限时栈已退出后没有服务，不作为IK失败证据。

修复了drive_harvest使用旧IDL字段、忽略产品success、忽略使能accepted、超时未取消及420s默认预算。8个离线回归从红到绿；相关43项通过/1历史数据缺失跳过，选定三个CMake测试组通过。实时SURVEY_ONLY10.4s成功，发现1；它不是完整抓取。

真实新画面已能检出目标，掩膜有效深度约96%、相机13–14Hz。与上午黄色袋漏检/袋口深度空洞场景不同。下一轮必须围绕外袋语义扎口、融合中心偏置与辅助球误作果实的门继续处理，同时核对咬合式30mm档案与感知50mm最小行程的语义。没有删门、伪造许可或把更短路径当完整抓取。

全过程的运行栈、相机驱动、静态TF、探针和Web已退出。后台ucvcameraservice按用户许可保留。完整目标仍未达成。
