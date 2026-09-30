# v1 三末端混合测试

**形式：** 真实立体相机在环（Percipio `169.254.10.110`，`camera_frontend:=stereo`）+ mock 臂仿真全流程抓取。  
**门：** 一次 `ExecuteTarget` FULL 干跑（用例 `1757`，`--velocity 1.0`，`skip_observation`，刀具关）墙钟 **≤ 120 s**（不含栈启动）。  
**禁止：** `hardware_mode:=real`、SetIO、宽泛 pkill。DDS 域 **71**，只清本域。

```bash
source /opt/ros/jazzy/setup.bash
source /home/mu/Desktop/aubo_e5_jazzy_ws/install/setup.bash
python3 plans/v1_三末端混合测试/run_hil.py
# 结束后：
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe'
```

三档必须整栈重启切换 `tool_profile`。imu_follow 仅 `adaptive_shear_v1` 应在图。

产物：`results/summary.json`、`results/<profile>.json`、`results/<profile>_grasp_1757.txt`。
