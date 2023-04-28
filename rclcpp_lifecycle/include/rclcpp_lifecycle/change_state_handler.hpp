#pragma once // TODO @tgroechel: check if they use pragma once or ifndef

// TODO @tgroechel: clean up includes here
#include <memory>
#include <functional>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/service.hpp"
#include "rmw/types.h"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace rclcpp_lifecycle
{
/*
*  Used for async user defined transition callbacks
*/
// TODO @tgroechel: comments for functions same as rclcpp style
class ChangeStateHandler
{
public:
    using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
    ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_udtf_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_on_error_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            finalize_change_state_cb);

    void continue_change_state(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

    // Calling outside of lifecycle_node_interface_impl is undefined behavior
    // Avoids needing to forward declare lifecycle_node_interface_impl + friend
    // TODO @tgroechel: is there a cleaner way to do this with a friend class?
    //                  another option is to base class in LifeCycle and then subclass in Impl called ChangeStateHandlerImpl -> this is probably the best idea imo, I'll do this last after I get the full functionality
    namespace lifecycle_node_interface_impl_private
    {
    bool _is_ready();
    bool _has_staged_srv_req()
    void _start_change_state();
    void _set_change_state_srv_hdl(const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl);
    void _set_rmw_request_id_header(const std::shared_ptr<rmw_request_id_t> header);    
    void _no_error_from_udtf();
    void _rcl_ret_error();
    void _finalize_change_state(bool success);
    bool _is_srv_request();
    }

private:
    // TODO @tgroechel: probably rename these, removing the cb and just doing 
    //                  "change_state_post_user_transition_callback" 
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_udtf_cb_;
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_on_error_cb_;
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        finalize_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_; 
    const std::shared_ptr<rmw_request_id_t> header_;

   /*
   READY            -> {STAGE_SRV_REQ, PRE_UDTF}
   STAGE_SRV_REQ    -> {PRE_UDTF}
   PRE_UDTF         -> {POST_UDTF}                      // change_state
   POST_UDTF        -> {POST_ON_ERROR, FINALIZING}      // post_udtf_cb_
   POST_ON_ERROR    -> {FINALIZING}                     // handle_post_on_error_cb_
   FINALIZING       -> {READY}                          // finalize_change_state_cb_
   ***ANY***        -> {FINALIZING}                     // i.e., early exit of change_state
   */
   enum ChangeStateStage // TODO @tgroechel: should have gone with a behavior tree design, this would be a reach goal though / a decent amount of scaffolding I think
   {
        READY,
        STAGED_SRV_REQ, // this is used as a passthrough for change_state when coming from srv
        PRE_UDTF, // TODO @tgroechel: once I rename the callback functions, these should also be renamed to PRE_USER_TRANSITION_CALLBACK/PRE_USER_TRANSITION_FUNCTION
        POST_UDTF,
        POST_ON_ERROR,
        FINALIZING
   };

   ChangeStateStage stage_;
};
} // namespace rclcpp_lifecycle