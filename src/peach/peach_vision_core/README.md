# peach_vision_core

Peach pure vision core: geometry, runtime, domain.

感知和重建共用 `normalize_depth_to_uint16_mm`。对 uint16 输入，只对有效测距应用
`depth_scale_unit`，原始值 0 与 65535 均保留为无效深度；对浮点输入按米换算为毫米。

挂袋长轴：`bag_landmarks._long_axis` 在点云 PCA 各向异性不足（s0/s1 小于 1.2）或
相对逆重力超过 45° 时回退重力先验。自由悬垂时剪影宽度不是袋轴；18 图案月牙
可把 SVD 主方向拧到侧面（台架袋2 官方 +65° 对 stereo 重力 +7°）。
