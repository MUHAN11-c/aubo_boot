// Survey等的是到位后曝光的新观测，不是只在锁定时递增的snapshot_id。
#ifndef PEACH_ARM__SURVEY_FRESHNESS_HPP_
#define PEACH_ARM__SURVEY_FRESHNESS_HPP_

#include <cstdint>

namespace peach_arm
{
inline bool surveyFrameFresh(
  int64_t image_stamp_ns, int64_t pose_reached_ns,
  uint32_t observed_epoch, uint32_t expected_epoch)
{
  return image_stamp_ns > pose_reached_ns && observed_epoch == expected_epoch;
}
}  // namespace peach_arm

#endif  // PEACH_ARM__SURVEY_FRESHNESS_HPP_
