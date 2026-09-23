# 注入矩阵 M1–M5 门判定

## sim_field_targets_20260923_105756.jsonl（20 例，网格）

- M1 网格期望一致: 19/20 = 95%（门 100%: 未过）
  - 不一致: deep_left_low_axis expect=succeed outcome=None completion=None reason=None
- 失败 1 例，无码 0 例，>300s 挂起 0 例（门: 无码 0、挂起 0: 过）
- 失败原因分布: {'reachability 拒: sleeve_no_cartesian': 1}

## sim_field_targets_20260923_110314.jsonl（29 例）

- SUCCEEDED: 25/29 = 86%
- completion≥3 且未抓取: 25/29 = 86%（门 ≥90%: 未过）
- 绕行比: 0.14（门 ≤1.70: 过）
- 失败 4 例，无码 0 例，>300s 挂起 0 例（门: 无码 0、挂起 0: 过）
- 失败原因分布: {'到预抓取失败: MTC planning failed: Failing stage(s):\nlin to on-axis pregrasp (0/1)': 2, '接触阶段取消或撤离未确认；保持停止，须现场人工撤离后确认恢复': 1, '到预抓取失败: MTC short-path guard rejected: TCP 姿态行程 66.982deg > 57.0325deg': 1}

## sim_field_targets_20260923_111235.jsonl（30 例）

- SUCCEEDED: 7/30 = 23%
- completion≥3 且未抓取: 7/30 = 23%（门 ≥90%: 未过）
- 绕行比: 0.00（门 ≤1.70: 过）
- 失败 23 例，无码 0 例，>300s 挂起 1 例（门: 无码 0、挂起 0: 未过）
- 失败原因分布: {'到预抓取失败: staging 转移无可行 IK 候选（无接近解）': 3, '接触阶段取消或撤离未确认；保持停止，须现场人工撤离后确认恢复': 4, '到预抓取失败: MTC planning failed: Failing stage(s):\nlin align tool z (0/1)': 1, '到预抓取失败: MTC short-path guard rejected: 工具筒体接触果实胶囊 s=0.115972m r=0.0905628m 间隙=-0.000584119m (果半径 0.04m)': 1, '到预抓取失败: MTC planning failed: Failing stage(s):\nlin to on-axis pregrasp (0/1)': 1, '到预抓取失败: MTC short-path guard rejected: 累计关节行程 12.8062rad > 12rad': 1, 'goal 超时（已取消）': 1, 'goal 被拒': 1}
