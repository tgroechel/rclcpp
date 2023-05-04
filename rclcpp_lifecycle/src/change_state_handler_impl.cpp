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

#include "change_state_handler_impl.hpp"
#include <cassert>
#include <memory>
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"

namespace rclcpp_lifecycle
{
ChangeStateHandlerImpl::ChangeStateHandlerImpl(
  std::recursive_mutex & state_machine_mutex,
  rcl_lifecycle_state_machine_t & state_machine,
  State & current_state,
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface)
: state_machine_mutex_(state_machine_mutex),
  state_machine_(state_machine),
  current_state_(current_state),
  node_base_interface_(node_base_interface)
{
}

void
ChangeStateHandlerImpl::set_change_state_srv_hdl(
  const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl)
{
  assert(change_state_srv_hdl_ == nullptr);
  change_state_srv_hdl_ = change_state_srv_hdl;
}

bool
ChangeStateHandlerImpl::register_callback(
  std::uint8_t lifecycle_transition,
  std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb)
{
  cb_map_[lifecycle_transition] = cb;
  auto it = async_cb_map_.find(lifecycle_transition);
  if (it != async_cb_map_.end()) {
    async_cb_map_.erase(it);
  }
  return true;
}

bool
ChangeStateHandlerImpl::register_async_callback(
  std::uint8_t lifecycle_transition,
  std::function<void(const State &, std::shared_ptr<ChangeStateHandler>)> & cb)
{
  async_cb_map_[lifecycle_transition] = cb;
  auto it = cb_map_.find(lifecycle_transition);
  if (it != cb_map_.end()) {
    cb_map_.erase(it);
  }
  return true;
}

void
ChangeStateHandlerImpl::continue_change_state(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  unsigned int current_state_id;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    current_state_id = state_machine_.current_state->id;
  }

  if (current_state_id ==
    lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    post_on_error_change_state(cb_return_code);
  } else if (!utf_has_been_called_.load()) {
    utf_has_been_called_.store(true);
    post_utf_change_state(cb_return_code);
  } else {
    RCUTILS_LOG_ERROR(
      "continue_change_state failed: called too many times");
    rcl_ret_error();
  }
}

void
ChangeStateHandlerImpl::change_state(
  uint8_t transition_id,
  const std::shared_ptr<rmw_request_id_t> header)
{
  assert(change_state_srv_hdl_ != nullptr);
  if (is_transitioning()) {
    RCUTILS_LOG_ERROR(
      "Currently in transition, failing requested transition id %d.", transition_id);
    if (header) {
      ChangeStateSrv::Response resp;
      resp.success = false;
      change_state_srv_hdl_->send_response(*header, resp);
    }
    return;
  }

  is_transitioning_.store(true);
  utf_has_been_called_.store(false);
  header_ = header;
  transition_id_ = transition_id;

  unsigned int current_state_id;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    constexpr bool publish_update = true;
    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR(
        "Unable to change state for state machine for %s: %s",
        node_base_interface_->get_name(), rcl_get_error_string().str);
      rcl_ret_error();
      return;
    }

    pre_transition_primary_state_ = State(state_machine_.current_state);

    if (
      rcl_lifecycle_trigger_transition_by_id(
        &state_machine_, transition_id_, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Unable to start transition %u from current state %s: %s",
        transition_id_, state_machine_.current_state->label,
        rcl_get_error_string().str);
      rcutils_reset_error();
      rcl_ret_error();
      return;
    }
    current_state_id = state_machine_.current_state->id;
    // Update the internal current_state_
    current_state_ = State(state_machine_.current_state);
  }

  if (is_async_callback(current_state_id)) {
    execute_async_callback(current_state_id, pre_transition_primary_state_);
  } else {
    cb_return_code_ = execute_callback(
      current_state_id,
      pre_transition_primary_state_);
    continue_change_state(cb_return_code_);
  }
}

void
ChangeStateHandlerImpl::post_utf_change_state(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  constexpr bool publish_update = true;
  unsigned int current_state_id;

  cb_return_code_ = cb_return_code;
  auto transition_label = get_label_for_return_code(cb_return_code);
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    if (
      rcl_lifecycle_trigger_transition_by_label(
        &state_machine_, transition_label, publish_update) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR(
        "Failed to finish transition %u: Current state is now: %s (%s)",
        transition_id_,
        state_machine_.current_state->label,
        rcl_get_error_string().str);
      rcutils_reset_error();
      rcl_ret_error();
      return;
    }
    current_state_id = state_machine_.current_state->id;

    // Update the internal current_state_
    current_state_ = State(state_machine_.current_state);
  }

  // error handling ?!
  // TODO(karsten1987): iterate over possible ret value
  if (cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR) {
    RCUTILS_LOG_WARN("Error occurred while calling transition function, calling on_error.");
    if (is_async_callback(current_state_id)) {
      execute_async_callback(current_state_id, pre_transition_primary_state_);
    } else {
      auto error_cb_code = execute_callback(
        current_state_id,
        pre_transition_primary_state_);
      continue_change_state(error_cb_code);
    }
  } else {
    finalize_change_state(
      cb_return_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }
}

void
ChangeStateHandlerImpl::post_on_error_change_state(
  node_interfaces::LifecycleNodeInterface::CallbackReturn error_cb_code)
{
  constexpr bool publish_update = true;
  auto error_cb_label = get_label_for_return_code(error_cb_code);
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (
    rcl_lifecycle_trigger_transition_by_label(
      &state_machine_, error_cb_label, publish_update) != RCL_RET_OK)
  {
    RCUTILS_LOG_ERROR("Failed to call cleanup on error state: %s", rcl_get_error_string().str);
    rcutils_reset_error();
    rcl_ret_error();
    return;
  }
  finalize_change_state(
    error_cb_code == node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

void
ChangeStateHandlerImpl::finalize_change_state(bool success)
{
  // TODO(karsten1987): Lifecycle msgs have to be extended to keep both returns
  // 1. return is the actual transition
  // 2. return is whether an error occurred or not
  rcl_ret_ = success ? RCL_RET_OK : RCL_RET_ERROR;

  if (header_) {
    ChangeStateSrv::Response resp;
    resp.success = success;
    change_state_srv_hdl_->send_response(*header_, resp);
    header_.reset();
  }

  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    // Update current state
    current_state_ = State(state_machine_.current_state);
  }
  is_transitioning_.store(false);
}

bool ChangeStateHandlerImpl::is_transitioning() const
{
  return is_transitioning_.load();
}

bool
ChangeStateHandlerImpl::is_async_callback(
  unsigned int cb_id) const
{
  return async_cb_map_.find(static_cast<uint8_t>(cb_id)) != async_cb_map_.end();
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
ChangeStateHandlerImpl::execute_callback(
  unsigned int cb_id, const State & previous_state) const
{
  // in case no callback was attached, we forward directly
  auto cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

  auto it = cb_map_.find(static_cast<uint8_t>(cb_id));
  if (it != cb_map_.end()) {
    auto callback = it->second;
    try {
      cb_success = callback(State(previous_state));
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR("Caught exception in callback for transition %d", it->first);
      RCUTILS_LOG_ERROR("Original error: %s", e.what());
      cb_success = node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  return cb_success;
}

void
ChangeStateHandlerImpl::execute_async_callback(
  unsigned int cb_id,
  const State & previous_state)
{
  auto it = async_cb_map_.find(static_cast<uint8_t>(cb_id));
  if (it != async_cb_map_.end()) {
    auto callback = it->second;
    try {
      callback(State(previous_state), shared_from_this());
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR("Caught exception in callback for transition %d", it->first);
      RCUTILS_LOG_ERROR("Original error: %s", e.what());
      continue_change_state(
        node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR);
    }
  } else {
    // Note that is_async_callback should be called before execute_async_callback
    // therefor this should never run
    // However, to be consistent with execute_callback:
    // in case no callback was attached, we forward directly
    continue_change_state(
      node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  }
}

const char *
ChangeStateHandlerImpl::get_label_for_return_code(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  auto cb_id = static_cast<uint8_t>(cb_return_code);
  if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS) {
    return rcl_lifecycle_transition_success_label;
  } else if (cb_id == lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE) {
    return rcl_lifecycle_transition_failure_label;
  }
  return rcl_lifecycle_transition_error_label;
}

void
ChangeStateHandlerImpl::rcl_ret_error()
{
  finalize_change_state(false);
}

ChangeStateHandlerImpl::~ChangeStateHandlerImpl()
{
  header_.reset();
  change_state_srv_hdl_.reset();
  node_base_interface_.reset();
}

}  // namespace rclcpp_lifecycle
