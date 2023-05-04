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

namespace rclcpp_lifecycle
{
ChangeStateHandlerImpl::ChangeStateHandlerImpl(
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
  post_user_transition_function_cb,
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
  post_on_error_cb,
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
  finalize_change_state_cb)
: post_user_transition_function_cb_(post_user_transition_function_cb),
  post_on_error_cb_(post_on_error_cb),
  finalize_change_state_cb_(finalize_change_state_cb),
  stage_(ChangeStateStage::READY)
{
}

void
ChangeStateHandlerImpl::continue_change_state(
  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
{
  assert(
    stage_ == ChangeStateStage::PRE_USER_TRANSITION_FUNCTION ||
    stage_ == ChangeStateStage::POST_USER_TRANSITION_FUNCTION ||
    stage_ == ChangeStateStage::FINALIZING);

  cb_return_code_ = cb_return_code;
  if (stage_ == ChangeStateStage::PRE_USER_TRANSITION_FUNCTION) {
    stage_ = ChangeStateStage::POST_USER_TRANSITION_FUNCTION;
    post_user_transition_function_cb_(cb_return_code);
  } else if (stage_ == ChangeStateStage::POST_USER_TRANSITION_FUNCTION) {
    stage_ = ChangeStateStage::POST_ON_ERROR;
    post_on_error_cb_(cb_return_code);
  } else if (stage_ == ChangeStateStage::FINALIZING) {
    finalize_change_state_cb_(cb_return_code);
  }
}

bool
ChangeStateHandlerImpl::is_processing_change_state_req()
{
  return !is_ready() && !has_staged_srv_req();
}

bool
ChangeStateHandlerImpl::is_ready()
{
  return stage_ == ChangeStateStage::READY;
}

bool
ChangeStateHandlerImpl::has_staged_srv_req()
{
  return stage_ == ChangeStateStage::STAGED_SRV_REQ;
}

void
ChangeStateHandlerImpl::start_change_state()
{
  assert(stage_ == ChangeStateStage::READY || stage_ == ChangeStateStage::STAGED_SRV_REQ);
  stage_ = ChangeStateStage::PRE_USER_TRANSITION_FUNCTION;
  rcl_ret_ = RCL_RET_OK;
}

void
ChangeStateHandlerImpl::set_change_state_srv_hdl(
  const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl)
{
  assert(change_state_srv_hdl_ == nullptr);
  change_state_srv_hdl_ = change_state_srv_hdl;
}

void
ChangeStateHandlerImpl::set_rmw_request_id_header(
  const std::shared_ptr<rmw_request_id_t> header)
{
  assert(stage_ == ChangeStateStage::READY);
  stage_ = ChangeStateStage::STAGED_SRV_REQ;
  header_ = header;
}

void
ChangeStateHandlerImpl::no_error_from_user_transition_function()
{
  assert(stage_ == ChangeStateStage::POST_USER_TRANSITION_FUNCTION);
  stage_ = ChangeStateStage::FINALIZING;
}

void
ChangeStateHandlerImpl::rcl_ret_error()
{
  finalize_change_state(false);
}

void
ChangeStateHandlerImpl::finalize_change_state(bool success)
{
  if (is_srv_request()) {
    ChangeStateSrv::Response resp;
    resp.success = success;
    change_state_srv_hdl_->send_response(*header_, resp);
    header_.reset();
  }

  // TODO(karsten1987): Lifecycle msgs have to be extended to keep both returns
  // 1. return is the actual transition
  // 2. return is whether an error occurred or not
  rcl_ret_ = success ? RCL_RET_OK : RCL_RET_ERROR;
  stage_ = ChangeStateStage::READY;
}

bool
ChangeStateHandlerImpl::is_srv_request()
{
  return header_ != nullptr;
}

ChangeStateHandlerImpl::~ChangeStateHandlerImpl()
{
  header_.reset();
  change_state_srv_hdl_.reset();
}

}  // namespace rclcpp_lifecycle
