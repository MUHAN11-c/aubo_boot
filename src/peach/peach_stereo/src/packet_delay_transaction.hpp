// Copyright 2026 Peach contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PACKET_DELAY_TRANSACTION_HPP_
#define PACKET_DELAY_TRANSACTION_HPP_

#include <cstdint>

#include "TYDefs.h"

namespace peach_stereo
{

// Caller serializes access and configures before StartCapture, restores after StopCapture.
// The API adapter also lets offline probes exercise partial SDK failures without a device.
struct PacketDelayTransaction
{
  int32_t requested_us{-1}, original_us{-1}, applied_us{-1}, restored_us{-1};
  bool original_known{false}, configured{false}, restore_pending{false};
  bool restore_attempted{false}, restore_verified{false};
  int restore_attempts{0};
  int info_status{0}, range_status{0}, original_status{0};
  int set_status{0}, readback_status{0}, restore_set_status{0}, restore_get_status{0};
  const char * setup_error{"not_requested"};

  template<typename Api>
  bool configure(Api & api, int32_t requested)
  {
    requested_us = requested;
    if (requested == -1) {return true;}
    if (requested < 0) {
      setup_error = "invalid_requested_value";
      return false;
    }
    TY_FEATURE_INFO info{};
    info_status = api.getInfo(info);
    if (info_status != TY_STATUS_OK || !info.isValid ||
      !(info.accessMode & TY_ACCESS_READABLE) || !(info.accessMode & TY_ACCESS_WRITABLE))
    {
      setup_error = "feature_not_readable_and_writable";
      return false;
    }
    TY_INT_RANGE range{};
    range_status = api.getRange(range);
    if (range_status != TY_STATUS_OK || range.inc <= 0) {
      setup_error = "invalid_integer_range";
      return false;
    }
    original_status = api.get(original_us);
    original_known = original_status == TY_STATUS_OK;
    if (!original_known) {
      setup_error = "original_value_unknown";
      return false;
    }
    const auto in_range = [&range](int32_t value) {
        return value >= range.min && value <= range.max &&
               (static_cast<int64_t>(value) - range.min) % range.inc == 0;
      };
    if (!in_range(requested) || !in_range(original_us)) {
      setup_error = "requested_or_original_out_of_range";
      return false;
    }
    applied_us = original_us;
    if (requested == original_us) {
      configured = true;
      setup_error = "none";
      return true;
    }
    // Arm restoration before the write: an error return can still mean partial mutation.
    restore_pending = true;
    set_status = api.set(requested);
    readback_status = api.get(applied_us);
    if (set_status != TY_STATUS_OK || readback_status != TY_STATUS_OK || applied_us != requested) {
      setup_error = set_status != TY_STATUS_OK ? "set_failed" :
        (readback_status != TY_STATUS_OK ? "readback_failed" : "readback_mismatch");
      restore(api);
      return false;
    }
    configured = true;
    setup_error = "none";
    return true;
  }

  template<typename Api>
  bool restore(Api & api)
  {
    if (!restore_pending) {return true;}
    restore_attempted = true;
    ++restore_attempts;
    restore_verified = false;
    if (!original_known) {
      // No guessed factory value, even if a caller presents inconsistent state.
      restore_set_status = TY_STATUS_NOT_PERMITTED;
      restore_get_status = TY_STATUS_NOT_PERMITTED;
      return false;
    }
    restore_set_status = api.set(original_us);
    restore_get_status = api.get(restored_us);
    restore_verified = restore_set_status == TY_STATUS_OK &&
      restore_get_status == TY_STATUS_OK && restored_us == original_us;
    if (restore_verified) {restore_pending = false;}
    return restore_verified;
  }
};

}  // namespace peach_stereo

#endif  // PACKET_DELAY_TRANSACTION_HPP_
