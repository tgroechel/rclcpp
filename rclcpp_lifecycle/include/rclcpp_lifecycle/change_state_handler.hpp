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
            post_error_handling_cb);

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
    void _rcl_ret_error();
    void _finalize_change_state(bool success);
    bool _is_srv_request();
    }

private:
    // TODO @tgroechel: probably rename these, removing the cb and just doing 
    //                  "change_state_post_user_transition_callback" and
    //                  "change_state_post_error_handling"
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_udtf_cb_;
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_error_handling_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_; 
    const std::shared_ptr<rmw_request_id_t> header_;

    /*
    These are the theoretical states of a `change_state` request
    Note that a change_state request can come either from a `ChangeState` srv call processed by `on_change_state`
    Or it can come from internal change_state call using `trigger` functions (often used in tests)

    0. ready to receive a change_state request (srv || trigger)
    1. before primary UDTF (i.e., non-error)
    2. post primary UDTF, before error checking
    3. post error checking
    4. returned / responded -> equivalent to 0. ready

    CHECKS:
    - on_change_state: check if header set, reject immediately if so
    - change_state: (run like a standard coroutine)
        - check if state is non-zero, reject immediately if so
        - set status to 1
    */
   enum ChangeStateStage
   {
        READY,
        STAGED_SRV_REQ, // this is used as a passthrough for change_state when coming from srv
        PRE_UDTF, // TODO @tgroechel: once I rename the callback functions, these should also be renamed to PRE_USER_TRANSITION_CALLBACK
        POST_UDTF,
        POST_ERROR_HANDLING
   };

   ChangeStateStage stage_;
};
} // namespace rclcpp_lifecycle