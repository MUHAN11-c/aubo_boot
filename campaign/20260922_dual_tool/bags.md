# bag 索引（bag 本体在 runs/session_*/bag，本表记录归属与分析结论）

| session 目录 | 归属轮次 (request_id) | 起止时刻 | 大小 | record.level | 分析结论 | purge 状态 |
|--------------|----------------------|----------|------|--------------|----------|------------|
| session_20260922_225652 | 注入轮（M1 首跑 4/20 级联污染） | 09-22 23:56 起 | 23G→0 | std | 级联根因=TEM stop 风暴+超时不取消，已修（d7efa43） | 已 purge（jsonl 留证入库） |
| session_20260923_040004 | 注入轮（M1 二跑 6/20，travel_max hang 后取消修复未解 TEM） | 09-23 04:00 起 | 33G→0 | std | TEM 风暴证据源（8.2 万行 stop 事件）；防线=execution_guard | 已 purge（jsonl 留证入库） |
| session_20260923_094702 | 注入链全量（M1 19/20 + 修复后 M1–M4） | 09-23 09:47 起 | 5.6G+ | std | 见 analysis/injection/ | 链毕后按分析结论处置 |

说明：
- session bag 由 peach_observability 随栈启停自动开合（MCAP），`record.level: std` 长跑 / `all` 失败短抓。
- 100G 预算超限自动回收最旧（`runs/retention_audit.jsonl`）；分析完成的轮用 `python3 scripts/purge_analyzed_bags.py runs/session_<该轮>` 删 MCAP 留 bag_report，并在本表登记。
