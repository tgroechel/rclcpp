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

#include "rclcpp_lifecycle/change_state_handler.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include "rclcpp/service.hpp"
#include "rmw/types.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


namespace rclcpp_lifecycle
{
class ChangeStateHandlerImpl : public ChangeStateHandler,
  public std::enable_shared_from_this<ChangeStateHandlerImpl>
{
public:
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code_;
  rcl_ret_t rcl_ret_;

  ChangeStateHandlerImpl(
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl,
    std::recursive_mutex & state_machine_mutex,
    rcl_lifecycle_state_machine_t & state_machine,
    State & current_state,
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface);

  bool
  register_callback(
    std::uint8_t lifecycle_transition,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb);

  bool
  register_async_callback(
    std::uint8_t lifecycle_transition,
    std::function<void(const State &, std::shared_ptr<ChangeStateHandler>)> & cb);

  void send_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) override;

  void change_state(
    uint8_t transition_id,
    const std::shared_ptr<rmw_request_id_t> header = nullptr);

  bool is_transitioning() const;

  virtual ~ChangeStateHandlerImpl();

private:
  std::atomic<bool> is_transitioning_{false};
  std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_;
  std::shared_ptr<rmw_request_id_t> header_;

  std::recursive_mutex & state_machine_mutex_;
  rcl_lifecycle_state_machine_t & state_machine_;
  State & current_state_;
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface_;


  std::map<
    std::uint8_t,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)>> cb_map_;
  std::map<
    std::uint8_t,
    std::function<void(const State &, std::shared_ptr<ChangeStateHandler>)>> async_cb_map_;

  State pre_transition_primary_state_;
  uint8_t transition_id_;

  void received_user_cb_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);
  void received_on_error_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn error_cb_code);
  void finalize_change_state(bool success);

  bool
  is_async_callback(unsigned int cb_id) const;

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  execute_callback(unsigned int cb_id, const State & previous_state) const;

  void
  execute_async_callback(unsigned int cb_id, const State & previous_state);

  const char *
  get_label_for_return_code(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

  void rcl_ret_error();

  bool in_non_error_transition_state() const;
  bool in_error_transition_state() const;
};
}  // namespace rclcpp_lifecycle

#endif  // CHANGE_STATE_HANDLER_IMPL_HPP_
