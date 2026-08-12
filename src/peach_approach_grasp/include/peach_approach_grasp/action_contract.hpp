// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef PEACH_APPROACH_GRASP__ACTION_CONTRACT_HPP_
#define PEACH_APPROACH_GRASP__ACTION_CONTRACT_HPP_

#include <string>

namespace peach_approach_grasp
{
enum class CycleOutcome {RUNNING, SUCCEEDED, CANCELED, FAILED, RECOVERY_REQUIRED};

inline CycleOutcome classifyTerminalState(const std::string & state)
{
  if (state == "SUCCEEDED" || state == "PREVIEW_READY") {
    return CycleOutcome::SUCCEEDED;
  }
  if (state == "CANCELED") {
    return CycleOutcome::CANCELED;
  }
  if (state == "RECOVERY_REQUIRED") {
    return CycleOutcome::RECOVERY_REQUIRED;
  }
  if (state == "FAILED" || state == "PREVIEW_FAILED") {
    return CycleOutcome::FAILED;
  }
  return CycleOutcome::RUNNING;
}
}  // namespace peach_approach_grasp
#endif  // PEACH_APPROACH_GRASP__ACTION_CONTRACT_HPP_
