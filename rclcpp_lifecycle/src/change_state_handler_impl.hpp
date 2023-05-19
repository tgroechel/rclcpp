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

#ifndef CHANGE_STATE_HANDLER_IMPL_HPP_
#define CHANGE_STATE_HANDLER_IMPL_HPP_

#include <memory>
#include <atomic>

#include "rclcpp_lifecycle/change_state_handler.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "lifecycle_node_state_manager.hpp"

namespace rclcpp_lifecycle
{

class ChangeStateHandlerImpl : public ChangeStateHandler
{
public:
  ChangeStateHandlerImpl(const std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl);

  bool send_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) override;

private:
  std::weak_ptr<LifecycleNodeStateManager> state_manager_hdl_;
  std::atomic<bool> response_sent_{false};
};

}  // namespace rclcpp_lifecycle
#endif  // CHANGE_STATE_HANDLER_IMPL_HPP_
