# 感知验证矩阵报告

近距视行（primary+supplemental）1035，远眺视行 30，光照 5 档。

| 分层 | recall@0.35 |
|---|---|
| backlit|alley_stop | 0.218 |
| backlit|cov=high | 0.037 |
| backlit|cov=low | 0.626 |
| backlit|cov=mid | 0.352 |
| backlit|level=heavy | 0.210 |
| backlit|level=light | 0.223 |
| backlit|level=none | 0.211 |
| backlit|level=reference | 0.875 |
| backlit|primary | 0.298 |
| backlit|supplemental | 0.306 |
| late_afternoon|alley_stop | 0.230 |
| late_afternoon|cov=high | 0.035 |
| late_afternoon|cov=low | 0.633 |
| late_afternoon|cov=mid | 0.350 |
| late_afternoon|level=heavy | 0.208 |
| late_afternoon|level=light | 0.219 |
| late_afternoon|level=none | 0.223 |
| late_afternoon|level=reference | 0.870 |
| late_afternoon|primary | 0.315 |
| late_afternoon|supplemental | 0.298 |
| morning|alley_stop | 0.230 |
| morning|cov=high | 0.030 |
| morning|cov=low | 0.618 |
| morning|cov=mid | 0.289 |
| morning|level=heavy | 0.165 |
| morning|level=light | 0.210 |
| morning|level=none | 0.202 |
| morning|level=reference | 0.822 |
| morning|primary | 0.278 |
| morning|supplemental | 0.275 |
| noon|alley_stop | 0.230 |
| noon|cov=high | 0.042 |
| noon|cov=low | 0.638 |
| noon|cov=mid | 0.357 |
| noon|level=heavy | 0.223 |
| noon|level=light | 0.221 |
| noon|level=none | 0.228 |
| noon|level=reference | 0.870 |
| noon|primary | 0.311 |
| noon|supplemental | 0.309 |
| overcast|alley_stop | 0.230 |
| overcast|cov=high | 0.032 |
| overcast|cov=low | 0.599 |
| overcast|cov=mid | 0.282 |
| overcast|level=heavy | 0.167 |
| overcast|level=light | 0.191 |
| overcast|level=none | 0.178 |
| overcast|level=reference | 0.856 |
| overcast|primary | 0.269 |
| overcast|supplemental | 0.269 |

## 袋级多视聚合（conf .35，单光照内 3 视序列）

| 口径 | 检出率 |
|---|---|
| any_view|level=heavy | 0.640 |
| any_view|level=light | 0.810 |
| any_view|level=none | 0.770 |
| any_view|level=reference | 1.000 |
| two_views|level=heavy | 0.550 |
| two_views|level=light | 0.720 |
| two_views|level=none | 0.660 |
| two_views|level=reference | 1.000 |

## 遮挡标定（深度反测覆盖率）

| 档 | n | 均值 | p90 |
|---|---|---|---|
| none | 2105 | 0.399 | 0.713 |
| light | 2355 | 0.420 | 0.766 |
| heavy | 2305 | 0.527 | 0.826 |
| reference | 1040 | 0.150 | 0.391 |

## SAM 框提示掩膜 IoU（primary 视）

| 档 | n | 均值 |
|---|---|---|
| heavy | 765 | 0.464 |
| light | 800 | 0.540 |
| none | 715 | 0.582 |
| reference | 380 | 0.856 |
