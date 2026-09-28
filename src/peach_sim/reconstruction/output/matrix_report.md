# 感知验证矩阵报告

近距视行（primary+supplemental）828，远眺视行 24，光照 4 档。

| 分层 | recall@0.35 |
|---|---|
| late_afternoon|alley_stop | 0.235 |
| late_afternoon|cov=high | 0.034 |
| late_afternoon|cov=low | 0.556 |
| late_afternoon|cov=mid | 0.275 |
| late_afternoon|level=heavy | 0.109 |
| late_afternoon|level=light | 0.265 |
| late_afternoon|level=none | 0.173 |
| late_afternoon|level=reference | 0.716 |
| late_afternoon|primary | 0.255 |
| late_afternoon|supplemental | 0.262 |
| morning|alley_stop | 0.259 |
| morning|cov=high | 0.035 |
| morning|cov=low | 0.569 |
| morning|cov=mid | 0.275 |
| morning|level=heavy | 0.106 |
| morning|level=light | 0.285 |
| morning|level=none | 0.177 |
| morning|level=reference | 0.707 |
| morning|primary | 0.265 |
| morning|supplemental | 0.263 |
| noon|alley_stop | 0.296 |
| noon|cov=high | 0.034 |
| noon|cov=low | 0.592 |
| noon|cov=mid | 0.285 |
| noon|level=heavy | 0.130 |
| noon|level=light | 0.282 |
| noon|level=none | 0.175 |
| noon|level=reference | 0.731 |
| noon|primary | 0.271 |
| noon|supplemental | 0.273 |
| overcast|alley_stop | 0.136 |
| overcast|cov=high | 0.012 |
| overcast|cov=low | 0.539 |
| overcast|cov=mid | 0.239 |
| overcast|level=heavy | 0.090 |
| overcast|level=light | 0.232 |
| overcast|level=none | 0.142 |
| overcast|level=reference | 0.702 |
| overcast|primary | 0.238 |
| overcast|supplemental | 0.231 |

## 袋级多视聚合（conf .35，单光照内 3 视序列）

| 口径 | 检出率 |
|---|---|
| any_view|level=heavy | 0.537 |
| any_view|level=light | 0.775 |
| any_view|level=none | 0.825 |
| any_view|level=reference | 0.944 |
| two_views|level=heavy | 0.487 |
| two_views|level=light | 0.662 |
| two_views|level=none | 0.787 |
| two_views|level=reference | 0.917 |

## 遮挡标定（深度反测覆盖率）

| 档 | n | 均值 | p90 |
|---|---|---|---|
| none | 1916 | 0.416 | 0.805 |
| light | 1360 | 0.399 | 0.716 |
| heavy | 1472 | 0.516 | 0.792 |
| reference | 832 | 0.148 | 0.369 |

## SAM 框提示掩膜 IoU（primary 视）

| 档 | n | 均值 |
|---|---|---|
| heavy | 484 | 0.490 |
| light | 444 | 0.645 |
| none | 656 | 0.581 |
| reference | 300 | 0.834 |
