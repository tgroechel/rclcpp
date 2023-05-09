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

#ifndef RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_
#define RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace rclcpp_lifecycle
{
class ChangeStateHandler
{
public:
  /// Continues the change state process handling proper callback order
  /** Used within the user defined transition callback to continue the change state process
   *  similar to a service call response
   *  Also used within the lifecycle_node_interface_impl to continue the change state process
   * \param[in] cb_return_code result of user defined transition callback
   */
  virtual void send_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) = 0;

  virtual ~ChangeStateHandler() {}
};
}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_
