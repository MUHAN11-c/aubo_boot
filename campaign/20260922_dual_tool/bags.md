# bag 索引（bag 本体在 runs/session_*/bag，本表记录归属与分析结论）

| session 目录 | 归属轮次 (request_id) | 起止时刻 | 大小 | record.level | 分析结论 | purge 状态 |
|--------------|----------------------|----------|------|--------------|----------|------------|
| （待填，逐轮追加） | | | | | | |

说明：
- session bag 由 peach_observability 随栈启停自动开合（MCAP），`record.level: std` 长跑 / `all` 失败短抓。
- 100G 预算超限自动回收最旧（`runs/retention_audit.jsonl`）；分析完成的轮用 `python3 scripts/purge_analyzed_bags.py runs/session_<该轮>` 删 MCAP 留 bag_report，并在本表登记。
