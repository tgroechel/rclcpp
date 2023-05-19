// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "change_state_handler_impl.hpp"

namespace rclcpp_lifecycle
{

ChangeStateHandlerImpl::ChangeStateHandlerImpl(
  const std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl)
: state_manager_hdl_(state_manager_hdl)
{
}

bool
ChangeStateHandlerImpl::send_callback_resp(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  if (!response_sent_.load()) {
    if (auto state_manager_hdl = state_manager_hdl_.lock()) {
      state_manager_hdl->process_callback_resp(cb_return_code);
      response_sent_.store(true);
      return true;
    }
  }
  return false;
}

}  // namespace rclcpp_lifecycle
