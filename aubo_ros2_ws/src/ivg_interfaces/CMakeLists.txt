cmake_minimum_required(VERSION 3.5)
project(ivg_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ═══════════════════════════════════════════════════════════
# 统一接口定义 — IVG2.0 所有自定义消息、服务集中于此喵~
# 共 17 msg + 35 srv = 52 个接口类型喵~
# ═══════════════════════════════════════════════════════════

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 机械臂底层状态 (来自原 aubo_msgs) ──
  "msg/Analog.msg"
  "msg/Digital.msg"
  "msg/IOStates.msg"
  "msg/JointTrajectoryFeedback.msg"
  "msg/RobotStateRTMsg.msg"
  "msg/RobotModeDataMsg.msg"
  "msg/MasterboardDataMsg.msg"
  "msg/ToolDataMsg.msg"

  # ── 机械臂高层状态 (来自原 demo_interface) ──
  "msg/RobotStatus.msg"
  "msg/RobotIOStatus.msg"
  "msg/ToolIOStatus.msg"
  "msg/ImageData.msg"
  "msg/NodeStatus.msg"
  "msg/SystemLog.msg"

  # ── 相机 (来自原 percipio_camera_interface) ──
  "msg/CameraStatus.msg"

  # ── 工具快换 (来自原 tool_changer_interface) ──
  "msg/ToolChangerStatus.msg"

  # ── 视觉位姿 (来自原 interface) ──
  "msg/CartesianPosition.msg"

  # ── 机械臂底层服务 (来自原 aubo_msgs) ──
  "srv/GetFK.srv"
  "srv/GetIK.srv"
  "srv/SetIO.srv"
  "srv/SetPayload.srv"
  "srv/SetSpeedSliderFraction.srv"

  # ── 运动控制服务 (来自原 demo_interface) ──
  "srv/MoveToPose.srv"
  "srv/SetRobotPose.srv"
  "srv/PlanTrajectory.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/GetCurrentState.srv"
  "srv/SetSpeedFactor.srv"
  "srv/SetRobotEnable.srv"
  "srv/SetRobotIO.srv"
  "srv/ReadRobotIO.srv"
  "srv/Movel.srv"
  "srv/ExecuteGraspPose.srv"
  "srv/MoveJoint.srv"
  "srv/MoveLine.srv"
  "srv/TeachStart.srv"
  "srv/SetCollisionClass.srv"
  "srv/SetToolKinematics.srv"
  "srv/SetToolVoltage.srv"

  # ── 相机控制服务 (来自原 percipio_camera_interface) ──
  "srv/SetCameraParameters.srv"
  "srv/SoftwareTrigger.srv"

  # ── 工具快换服务 (来自原 tool_changer_interface) ──
  "srv/RunGripperSwap.srv"
  "srv/ChangeTool.srv"
  "srv/GetCurrentTool.srv"

  # ── 拉花轨迹回放服务 (来自 latte_imitation) ──
  "srv/ReplayLatteTrajectory.srv"

  # ── 视觉位姿估计服务 (来自原 interface) ──
  "srv/EstimatePose.srv"
  "srv/EstimatePose2D.srv"
  "srv/ListTemplates.srv"
  "srv/StandardizeTemplate.srv"
  "srv/UpdateParams.srv"
  "srv/ProcessDebugStep.srv"
  "srv/VisualizeGraspPose.srv"

  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    trajectory_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
