// 上游 0.8.5 foxglove_bridge.cpp 的本地替代：版本符号定义。
// 上游由 CMake 注入 FOXGLOVE_BRIDGE_VERSION；本包直接钉字符串，来源见 VENDOR_PROVENANCE.md。
#include "foxglove_bridge/foxglove_bridge.hpp"

#include <websocketpp/version.hpp>

namespace foxglove {

const char* WebSocketUserAgent() {
  return websocketpp::user_agent;
}

// 语义：服务器库 = ws-protocol 最终独立版；glue = 0.8.5 ros2 适配。
const char FOXGLOVE_BRIDGE_VERSION[] = "0.8.5-ivg2";

const char FOXGLOVE_BRIDGE_GIT_HASH[] = "ws-protocol@f3b4135";

}  // namespace foxglove
