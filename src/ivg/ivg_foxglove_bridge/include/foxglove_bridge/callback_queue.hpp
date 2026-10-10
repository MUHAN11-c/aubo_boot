#pragma once

// Vendor shim: 0.8.5-era glue 以 <foxglove_bridge/xxx.hpp> 引用服务器库；
// 本包 vendor 的最终独立库（foxglove/ws-protocol f3b4135）头在 <foxglove/websocket/>。
// 保持上游 glue 文件零改动，适配收敛到本转发头。见 VENDOR_PROVENANCE.md。
#include <foxglove/websocket/callback_queue.hpp>
