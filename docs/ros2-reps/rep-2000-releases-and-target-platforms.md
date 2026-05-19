# REP 2000 -- ROS 2 Releases and Target Platforms

- **Status**: Active
- **Type**: Informational
- **Source**: https://www.ros.org/reps/rep-2000.html

## 核心内容摘要

### 发布节奏
- **每年发布一次**，每 12 个月一个新版本
- **偶数年**：LTS 发布（5 年支持，对应 Ubuntu LTS）
- **奇数年**：非 LTS 发布（1.5 年支持，与上一个 LTS 共用 Ubuntu LTS）

### Humble Hawksbill (May 2022 - May 2027)
- 目标平台：Ubuntu Jammy (22.04) Tier 1
- 最低语言要求：C++17, Python 3.6
- 默认 RMW：eProsima Fast-DDS
- 本项目的目标 ROS 2 发行版

### 支持层级 (Support Tiers)
- **Tier 1**: CI 持续测试，错误优先修复
- **Tier 2**: 定期 CI 测试，尽力修复
- **Tier 3**: 社区支持，不运行测试套件

### 中间件支持（Humble）
| Middleware | 提供商 | 等级 |
|-----------|--------|------|
| rmw_fastrtps_cpp* | eProsima Fast-DDS | Tier 1 |
| rmw_cyclonedds_cpp | Eclipse Cyclone DDS | Tier 1 |
| rmw_connextdds | RTI Connext | Tier 1 |
| rmw_fastrtps_dynamic_cpp | eProsima Fast-DDS | Tier 2 |
| rmw_gurumdds_cpp | GurumNetworks GurumDDS | Tier 3 |

### Jazzy Jalisco (May 2024 - May 2029)
- 目标平台：Ubuntu Noble (24.04) Tier 1
- C++17, Python 3.8

### Kilted Kaiju (May 2025 - November 2026)
- 新增 Zenoh RMW (Tier 1)
- 默认 RMW：eProsima Fast-DDS

完整内容见：https://www.ros.org/reps/rep-2000.html
