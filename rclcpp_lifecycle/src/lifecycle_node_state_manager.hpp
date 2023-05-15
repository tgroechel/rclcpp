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

#ifndef LIFECYCLE_NODE_STATE_MANAGER_HPP_
#define LIFECYCLE_NODE_STATE_MANAGER_HPP_

#include <atomic>
#include <functional>
#include <map>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rmw/types.h"

#include "lifecycle_state_services_manager.hpp"


namespace rclcpp_lifecycle
{
class LifecycleNodeStateManager
  : public std::enable_shared_from_this<LifecycleNodeStateManager>
{
public:
  using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;

  void init(
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    bool enable_communication_interface);

  bool
  register_callback(
    std::uint8_t lifecycle_transition,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb);

  void process_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

  const State & get_current_state() const;

  std::vector<State> get_available_states() const;

  std::vector<Transition> get_available_transitions() const;

  std::vector<Transition> get_transition_graph() const;

  rcl_ret_t change_state(
    uint8_t transition_id,
    node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code);

  rcl_ret_t change_state(
    uint8_t transition_id,
    const std::shared_ptr<rmw_request_id_t> header = nullptr);

  bool is_transitioning() const;

  /**
   * @brief Gets the transition id prioritizing request->transition.label over
   *        request->transition.id if the label is set
   *        Throws exception if state_machine_ is not initialized
   * @return the transition id, returns -1 if the transition does not exist
  */
  int get_transition_id_from_request(const ChangeStateSrv::Request::SharedPtr req);

  const rcl_lifecycle_transition_t * get_transition_by_label(const char * label) const;

  virtual ~LifecycleNodeStateManager();

private:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface_;
  std::unique_ptr<LifecycleStateServicesManager> state_services_manager_hdl_;

  mutable std::recursive_mutex state_machine_mutex_;
  rcl_lifecycle_state_machine_t state_machine_;
  State current_state_;
  std::map<
    std::uint8_t,
    std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)>> cb_map_;

  std::atomic<bool> is_transitioning_{false};
  std::shared_ptr<rmw_request_id_t> header_;
  State pre_transition_primary_state_;
  uint8_t transition_id_;

  node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code_;
  rcl_ret_t rcl_ret_;

  void process_user_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);
  void process_on_error_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn error_cb_code);
  void finalize_change_state(bool success);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  execute_callback(unsigned int cb_id, const State & previous_state) const;

  const char *
  get_label_for_return_code(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

  void rcl_ret_error();

  void update_current_state_();
  uint8_t get_current_state_id() const;
  bool in_non_error_transition_state(uint8_t) const;
  bool in_error_transition_state(uint8_t) const;
};
}  // namespace rclcpp_lifecycle
#endif  // LIFECYCLE_NODE_STATE_MANAGER_HPP_
