"""
offline 子包 — 离线数据集评估/回放工具，不参与在线管线.

职责:
  录制 RGB-D 数据集上的可复现评估与可视化参考：端到端批量验证
  （``e2e_validate``，输出 JSON/Markdown 报告）、真值标注加载与指标
  （``validation``）、全局配置常量（``config``：内参/数据集路径/版本标识）、
  球拟合可视化参考（``sphere_ref``，仅人工验收，不进安全判定）。

与在线算法分层:
  本子包只被离线 CLI / 测试 import；在线路径（节点 → candidates →
  pipeline/fitting）不依赖本子包。相机内参约定：实机/本机工具一律
  ``K_PERCIPIO``，``K_AZURE`` 仅历史 Azure 离线录包。
"""
