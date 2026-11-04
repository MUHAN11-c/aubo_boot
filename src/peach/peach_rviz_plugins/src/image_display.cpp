// Copyright 2026 peach contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
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

#include "message_filters/connection.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display.hpp"
#include "rviz_default_plugins/displays/image/image_display.hpp"

namespace peach_rviz_plugins
{

/// Keep the upstream image display, making its subscription teardown repeatable.
class ImageDisplay : public rviz_default_plugins::displays::ImageDisplay
{
public:
  ~ImageDisplay() override
  {
    // The upstream base destructor also unsubscribes. Leave it an empty connection.
    unsubscribe();
  }

protected:
  void unsubscribe() override
  {
    if (subscription_) {
      subscription_callback_.disconnect();
    }
    // Connection::disconnect() retains its callback, which captures Signal1 by raw address.
    // Clear that callback before releasing the SubscriberFilter that owns Signal1.
    subscription_callback_ = message_filters::Connection();
    subscription_.reset();
  }
};

}  // namespace peach_rviz_plugins

PLUGINLIB_EXPORT_CLASS(peach_rviz_plugins::ImageDisplay, rviz_common::Display)
