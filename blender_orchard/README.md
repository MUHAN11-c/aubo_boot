# blender_orchard

只在 Blender 里看的套袋桃园。不进 colcon，也不导出到 Gazebo / glTF，等确认后再导出。

## 口径

布局和袋具跟 `src/peach_sim/config/orchard.yaml`：行距 4 m、株距 2.2 m、干高 0.50–0.65 m、主枝开角 45–60°、袋底离地 1.30–1.42 m、簇内最小间距 0.055 m、袋轴倾角 `46°·u^1.9`（中位约 13°）。袋轮廓跟 `src/peach_sim/reconstruction/geometry.py` 的 `bag_rings`：平底、肩宽、收颈。

纸色仍用 PeachDataSet 中位 (101, 60, 55)，统计在 `data/priors.json`。现场袋底世界高度约 1.34 m（`field_pregrasp_cases.yaml`，臂座离地 0.75 m）。`peach_sim/reconstruction/` 是另一条按单帧 RGB-D 重建的入口，本目录不覆盖它。

## 重跑

```bash
aubo_py3.12/bin/python blender_orchard/tools/measure_priors.py
aubo_py3.12/bin/python blender_orchard/tools/make_textures.py
_tools/blender-4.5.14-linux-x64/blender -b -P blender_orchard/tools/build_scene.py
aubo_py3.12/bin/python blender_orchard/tools/verify_render.py
```

场景文件：`scene/peach_orchard.blend`。渲染：`renders/cam_alley.png`（作业道）、`renders/cam_up.png`（袋面近景）、`renders/cam_rows.png`（顺行）。
检测分割核对写到 `data/render_check.json`，掩膜叠图为 `renders/*_sam.png`。
袋口接在果枝上，每树 2–3 簇、每簇 3–5 只，步距按 `peach_sim` 的贴袋/散袋混合。叶片用 `reconstruction/geometry.py` 的连通叶面。
纸袋按 Peach_bag/RGB/1200 的观感改成扁红纸袋（宽 12–16 cm、高 15–19 cm、厚约 4 cm），不再用 7 cm 的鼓肩。
主干半径约 9.5 cm，主枝约 4 cm，叶片都从枝上长出。
叶只挡袋正面，侧面和背后仍有叶。
近景机位在冠外，对着挂果枝。过强的袋面位移会把纸袋挤成扇贝边，已收回。
袋面贴图改成长短不一的斜折，不再是成排波纹。外轮廓没动。
最近一次核对（阈值 0.35，全是 `peach_bag`）：作业道 6 只、最高 0.84；近景 3 只、最高 0.90；顺行 7 只、最高 0.77；斜视 7 只、最高 0.85。全园 84 只袋。确认前不导出。
