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

#ifndef RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_IMPL_HPP_
#define RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_IMPL_HPP_

#include "rclcpp_lifecycle/change_state_handler.hpp"

#include <functional>
#include "rclcpp/service.hpp"
#include "rmw/types.h"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"


namespace rclcpp_lifecycle
{
class ChangeStateHandlerImpl : public ChangeStateHandler
{
public:
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
  ChangeStateHandlerImpl(
      std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
          post_user_transition_function_cb,
      std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
          post_on_error_cb,
      std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
          finalize_change_state_cb);

  void continue_change_state(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) override;

  bool is_ready();
  bool has_staged_srv_req();
  bool is_processing_change_state_req();
  void start_change_state();
  void set_change_state_srv_hdl(
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl);
  void set_rmw_request_id_header(
    const std::shared_ptr<rmw_request_id_t> header); 
  void no_error_from_user_transition_function();
  void rcl_ret_error();
  void finalize_change_state(bool success);
  bool is_srv_request();

  virtual ~ChangeStateHandlerImpl();

  State pre_transition_primary_state_;
  uint8_t transition_id_;

  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code_;
  rcl_ret_t rcl_ret_;

private:
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
      post_user_transition_function_cb_;
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
      post_on_error_cb_;
  std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
      finalize_change_state_cb_;
  
  std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_; 
  std::shared_ptr<rmw_request_id_t> header_;

  /*
  READY                         -> {STAGE_SRV_REQ, PRE_USER_TRANSITION_FUNCTION}
  STAGE_SRV_REQ                 -> {PRE_USER_TRANSITION_FUNCTION}
  PRE_USER_TRANSITION_FUNCTION  -> {POST_USER_TRANSITION_FUNCTION}  // change_state
  POST_USER_TRANSITION_FUNCTION -> {POST_ON_ERROR, FINALIZING}      // post_user_transition_function_cb_
  POST_ON_ERROR                 -> {FINALIZING}                     // handle_post_on_error_cb_
  FINALIZING                    -> {READY}                          // finalize_change_state_cb_
  ***ANY***                     -> {FINALIZING}                     // i.e., early exit of change_state
  */
  enum ChangeStateStage
  {
    READY,
    STAGED_SRV_REQ,
    PRE_USER_TRANSITION_FUNCTION,
    POST_USER_TRANSITION_FUNCTION,
    POST_ON_ERROR,
    FINALIZING
  };

  ChangeStateStage stage_;
};
} // namespace rclcpp_lifecycle

#endif // RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_IMPL_HPP_
