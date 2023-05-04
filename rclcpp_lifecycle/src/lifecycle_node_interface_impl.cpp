// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition_event.h"  // for getting the c-typesupport
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rcl/error_handling.h"
#include "rcl/node.h"

#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_lifecycle/transition_map.h"

#include "rcutils/logging_macros.h"

#include "rmw/types.h"

#include "lifecycle_node_interface_impl.hpp"

namespace rclcpp_lifecycle
{

LifecycleNode::LifecycleNodeInterfaceImpl::LifecycleNodeInterfaceImpl(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface)
: node_base_interface_(node_base_interface),
  node_services_interface_(node_services_interface)
{
}

LifecycleNode::LifecycleNodeInterfaceImpl::~LifecycleNodeInterfaceImpl()
{
  rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
  rcl_ret_t ret;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
    ret = rcl_lifecycle_state_machine_fini(&state_machine_, node_handle);
  }
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_FATAL_NAMED(
      "rclcpp_lifecycle",
      "failed to destroy rcl_state_machine");
  }
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::init(bool enable_communication_interface)
{
  rcl_node_t * node_handle = node_base_interface_->get_rcl_node_handle();
  const rcl_node_options_t * node_options =
    rcl_node_get_options(node_base_interface_->get_rcl_node_handle());
  auto state_machine_options = rcl_lifecycle_get_default_state_machine_options();
  state_machine_options.enable_com_interface = enable_communication_interface;
  state_machine_options.allocator = node_options->allocator;

  // The call to initialize the state machine takes
  // currently five different typesupports for all publishers/services
  // created within the RCL_LIFECYCLE structure.
  // The publisher takes a C-Typesupport since the publishing (i.e. creating
  // the message) is done fully in RCL.
  // Services are handled in C++, so that it needs a C++ typesupport structure.
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  state_machine_ = rcl_lifecycle_get_zero_initialized_state_machine();
  rcl_ret_t ret = rcl_lifecycle_state_machine_init(
    &state_machine_,
    node_handle,
    ROSIDL_GET_MSG_TYPE_SUPPORT(lifecycle_msgs, msg, TransitionEvent),
    rosidl_typesupport_cpp::get_service_type_support_handle<ChangeStateSrv>(),
    rosidl_typesupport_cpp::get_service_type_support_handle<GetStateSrv>(),
    rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableStatesSrv>(),
    rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
    rosidl_typesupport_cpp::get_service_type_support_handle<GetAvailableTransitionsSrv>(),
    &state_machine_options);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
            std::string("Couldn't initialize state machine for node ") +
            node_base_interface_->get_name());
  }
  change_state_hdl = std::make_shared<ChangeStateHandlerImpl>(
    std::ref(state_machine_mutex_),
    std::ref(state_machine_),
    std::ref(current_state_),
    node_base_interface_);

  current_state_ = State(state_machine_.current_state);

  if (enable_communication_interface) {
    { // change_state
      auto cb = std::bind(
        &LifecycleNode::LifecycleNodeInterfaceImpl::on_change_state, this,
        std::placeholders::_1, std::placeholders::_2);
      rclcpp::AnyServiceCallback<ChangeStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_change_state_ = std::make_shared<rclcpp::Service<ChangeStateSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_change_state,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_change_state_),
        nullptr);
      change_state_hdl->set_change_state_srv_hdl(srv_change_state_);
    }

    { // get_state
      auto cb = std::bind(
        &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_state, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetStateSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_state_ = std::make_shared<rclcpp::Service<GetStateSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_state,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_state_),
        nullptr);
    }

    { // get_available_states
      auto cb = std::bind(
        &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_states, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableStatesSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_states_ = std::make_shared<rclcpp::Service<GetAvailableStatesSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_available_states,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_states_),
        nullptr);
    }

    { // get_available_transitions
      auto cb = std::bind(
        &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_transitions, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_available_transitions_ =
        std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_available_transitions,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_available_transitions_),
        nullptr);
    }

    { // get_transition_graph
      auto cb = std::bind(
        &LifecycleNode::LifecycleNodeInterfaceImpl::on_get_transition_graph, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      rclcpp::AnyServiceCallback<GetAvailableTransitionsSrv> any_cb;
      any_cb.set(std::move(cb));

      srv_get_transition_graph_ =
        std::make_shared<rclcpp::Service<GetAvailableTransitionsSrv>>(
        node_base_interface_->get_shared_rcl_node_handle(),
        &state_machine_.com_interface.srv_get_transition_graph,
        any_cb);
      node_services_interface_->add_service(
        std::dynamic_pointer_cast<rclcpp::ServiceBase>(srv_get_transition_graph_),
        nullptr);
    }
  }
}

bool
LifecycleNode::LifecycleNodeInterfaceImpl::register_callback(
  std::uint8_t lifecycle_transition,
  std::function<node_interfaces::LifecycleNodeInterface::CallbackReturn(const State &)> & cb)
{
  return change_state_hdl->register_callback(lifecycle_transition, cb);
}

bool
LifecycleNode::LifecycleNodeInterfaceImpl::register_async_callback(
  std::uint8_t lifecycle_transition,
  std::function<void(const State &, std::shared_ptr<ChangeStateHandler>)> & cb)
{
  return change_state_hdl->register_async_callback(lifecycle_transition, cb);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_change_state(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<ChangeStateSrv::Request> req)
{
  std::uint8_t transition_id;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

    if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
      throw std::runtime_error("Can't get state. State machine is not initialized.");
    }
    // Use transition.label over transition.id if transition.label exits
    // label has higher precedence to the id due to ROS 2 defaulting integers to 0
    // e.g.: srv call of {transition: {label: configure}}
    //       transition.id    = 0           -> would be equiv to "create"
    //       transition.label = "configure" -> id is 1, use this
    if (req->transition.label.size() != 0) {
      auto rcl_transition = rcl_lifecycle_get_transition_by_label(
        state_machine_.current_state, req->transition.label.c_str());
      if (rcl_transition == nullptr) {
        // send fail response printing out the requested label
        RCUTILS_LOG_ERROR(
          "ChangeState srv request failed: Transition label '%s'"
          " is not available in the current state '%s'",
          req->transition.label.c_str(), state_machine_.current_state->label);
        ChangeStateSrv::Response resp;
        resp.success = false;
        srv_change_state_->send_response(*header, resp);
        return;
      }
      transition_id = static_cast<std::uint8_t>(rcl_transition->id);
    } else {
      transition_id = req->transition.id;
    }
  }

  change_state_hdl->change_state(transition_id, header);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_get_state(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetStateSrv::Request> req,
  std::shared_ptr<GetStateSrv::Response> resp) const
{
  (void)header;
  (void)req;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error(
            "Can't get state. State machine is not initialized.");
  }
  resp->current_state.id = static_cast<uint8_t>(state_machine_.current_state->id);
  resp->current_state.label = state_machine_.current_state->label;
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_states(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableStatesSrv::Request> req,
  std::shared_ptr<GetAvailableStatesSrv::Response> resp) const
{
  (void)header;
  (void)req;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error(
            "Can't get available states. State machine is not initialized.");
  }

  resp->available_states.resize(state_machine_.transition_map.states_size);
  for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
    resp->available_states[i].id =
      static_cast<uint8_t>(state_machine_.transition_map.states[i].id);
    resp->available_states[i].label =
      static_cast<std::string>(state_machine_.transition_map.states[i].label);
  }
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_get_available_transitions(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
  std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const
{
  (void)header;
  (void)req;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error(
            "Can't get available transitions. State machine is not initialized.");
  }

  resp->available_transitions.resize(state_machine_.current_state->valid_transition_size);
  for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
    lifecycle_msgs::msg::TransitionDescription & trans_desc = resp->available_transitions[i];

    auto rcl_transition = state_machine_.current_state->valid_transitions[i];
    trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
    trans_desc.transition.label = rcl_transition.label;
    trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
    trans_desc.start_state.label = rcl_transition.start->label;
    trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
    trans_desc.goal_state.label = rcl_transition.goal->label;
  }
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_get_transition_graph(
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<GetAvailableTransitionsSrv::Request> req,
  std::shared_ptr<GetAvailableTransitionsSrv::Response> resp) const
{
  (void)header;
  (void)req;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  if (rcl_lifecycle_state_machine_is_initialized(&state_machine_) != RCL_RET_OK) {
    throw std::runtime_error(
            "Can't get available transitions. State machine is not initialized.");
  }

  resp->available_transitions.resize(state_machine_.transition_map.transitions_size);
  for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
    lifecycle_msgs::msg::TransitionDescription & trans_desc = resp->available_transitions[i];

    auto rcl_transition = state_machine_.transition_map.transitions[i];
    trans_desc.transition.id = static_cast<uint8_t>(rcl_transition.id);
    trans_desc.transition.label = rcl_transition.label;
    trans_desc.start_state.id = static_cast<uint8_t>(rcl_transition.start->id);
    trans_desc.start_state.label = rcl_transition.start->label;
    trans_desc.goal_state.id = static_cast<uint8_t>(rcl_transition.goal->id);
    trans_desc.goal_state.label = rcl_transition.goal->label;
  }
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::get_current_state() const
{
  return current_state_;
}

std::vector<State>
LifecycleNode::LifecycleNodeInterfaceImpl::get_available_states() const
{
  std::vector<State> states;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  states.reserve(state_machine_.transition_map.states_size);

  for (unsigned int i = 0; i < state_machine_.transition_map.states_size; ++i) {
    states.emplace_back(&state_machine_.transition_map.states[i]);
  }
  return states;
}

std::vector<Transition>
LifecycleNode::LifecycleNodeInterfaceImpl::get_available_transitions() const
{
  std::vector<Transition> transitions;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  transitions.reserve(state_machine_.current_state->valid_transition_size);

  for (unsigned int i = 0; i < state_machine_.current_state->valid_transition_size; ++i) {
    transitions.emplace_back(&state_machine_.current_state->valid_transitions[i]);
  }
  return transitions;
}

std::vector<Transition>
LifecycleNode::LifecycleNodeInterfaceImpl::get_transition_graph() const
{
  std::vector<Transition> transitions;
  std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);
  transitions.reserve(state_machine_.transition_map.transitions_size);

  for (unsigned int i = 0; i < state_machine_.transition_map.transitions_size; ++i) {
    transitions.emplace_back(&state_machine_.transition_map.transitions[i]);
  }
  return transitions;
}

rcl_ret_t
LifecycleNode::LifecycleNodeInterfaceImpl::change_state(
  std::uint8_t transition_id,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  change_state_hdl->change_state(transition_id);
  // Set the reference cb_return_code to the last held one
  // to hold consistency with prior implementation
  cb_return_code = change_state_hdl->cb_return_code_;
  return change_state_hdl->rcl_ret_;
}

const State & LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  const char * transition_label)
{
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;
  return trigger_transition(transition_label, error);
}

const State & LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  const char * transition_label,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  const rcl_lifecycle_transition_t * transition;
  {
    std::lock_guard<std::recursive_mutex> lock(state_machine_mutex_);

    transition =
      rcl_lifecycle_get_transition_by_label(state_machine_.current_state, transition_label);
  }
  if (transition) {
    change_state(static_cast<uint8_t>(transition->id), cb_return_code);
  }
  return get_current_state();
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(uint8_t transition_id)
{
  node_interfaces::LifecycleNodeInterface::CallbackReturn error;
  change_state(transition_id, error);
  (void) error;
  return get_current_state();
}

const State &
LifecycleNode::LifecycleNodeInterfaceImpl::trigger_transition(
  uint8_t transition_id,
  node_interfaces::LifecycleNodeInterface::CallbackReturn & cb_return_code)
{
  change_state(transition_id, cb_return_code);
  return get_current_state();
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::add_managed_entity(
  std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity)
{
  weak_managed_entities_.push_back(managed_entity);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::add_timer_handle(
  std::shared_ptr<rclcpp::TimerBase> timer)
{
  weak_timers_.push_back(timer);
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_activate() const
{
  for (const auto & weak_entity : weak_managed_entities_) {
    auto entity = weak_entity.lock();
    if (entity) {
      entity->on_activate();
    }
  }
}

void
LifecycleNode::LifecycleNodeInterfaceImpl::on_deactivate() const
{
  for (const auto & weak_entity : weak_managed_entities_) {
    auto entity = weak_entity.lock();
    if (entity) {
      entity->on_deactivate();
    }
  }
}

}  // namespace rclcpp_lifecycle
