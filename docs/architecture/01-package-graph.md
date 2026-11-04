# 包依赖

```mermaid
flowchart TB
  IF[peach_interfaces]
  CM[peach_common]
  VC[peach_vision_core]
  SP[peach_scene_perception]
  TR[peach_target_reconstruction]
  SO[peach_scene_obstacles]
  SUP[peach_supervisor]
  ARM[peach_arm]
  BR[peach_bringup]
  OBS[peach_observability]
  WEB[peach_web]
  SF[serial_force]

  IF --> SP
  IF --> TR
  IF --> SO
  IF --> SUP
  IF --> ARM
  IF --> SF

  CM --> VC
  CM --> SP
  CM --> TR
  CM --> SO
  CM --> SUP
  CM --> ARM
  CM --> SF

  VC --> SP
  VC --> TR

  SP --> BR
  TR --> BR
  SO --> BR
  SUP --> BR
  ARM --> BR
  OBS --> BR
  WEB --> OBS
```

## 允许/禁止

- 允许：能力包 → `peach_interfaces` / `peach_common` / `peach_vision_core`；组合/应用层 → 能力包。
- 禁止：能力包之间互相 import；核心 import `serial_force`/相机/臂驱动；`peach_web` import ROS。
- `peach_arm` 只依赖 `peach_common` + `peach_interfaces`，不再依赖视觉包。
