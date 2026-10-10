#pragma once

// 上游 0.8.5 foxglove_bridge.hpp 原样保留（版本符号声明）。
// 实现见本包 src/version.cpp（上游 foxglove_bridge.cpp 的本地替代）。
namespace foxglove {

const char* WebSocketUserAgent();

extern const char FOXGLOVE_BRIDGE_VERSION[];

extern const char FOXGLOVE_BRIDGE_GIT_HASH[];

}  // namespace foxglove
