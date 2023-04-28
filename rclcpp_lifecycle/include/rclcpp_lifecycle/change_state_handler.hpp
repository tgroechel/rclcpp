#pragma once // TODO @tgroechel: check if they use pragma once or ifndef

// TODO @tgroechel: clean up includes here
#include <memory>
#include <vector>
#include <functional>
#include <atomic>
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
// TODO @tgroechel: this is likely going to handle much more given we need to deal with internal trigger
//                  give this the capability to understand if it is doing an internal transition or not
class ChangeStateHandler
{
public:
    using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
    ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            complete_change_state_cb);

    void continue_change_state(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

    // Calling outside of lifecycle_node_interface_impl is undefined behavior
    // Avoids needing to forward declare lifecycle_node_interface_impl + friend
    // TODO @tgroechel: is there a cleaner way to do this with a friend class?
    //                  another option is to base class in LifeCycle and then subclass in Impl called ChangeStateHandlerImpl -> this is probably the best idea imo, I'll do this last after I get the full functionality
    namespace lifecycle_node_interface_impl_private
    {
    void _set_change_state_srv_hdl(const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl);
    void _set_rmw_request_id_header(const std::shared_ptr<rmw_request_id_t> header);    
    void _rcl_ret_error();
    void _finalize_change_state(bool success);
    bool _is_srv_request();
    }

private:
    // TODO @tgroechel: very likely we have the ability to shift this or to do something different for `on_error`
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        complete_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_; 
    const std::shared_ptr<rmw_request_id_t> header_;
    std::atomic<bool> in_transition_{false}; // TODO @tgroechel: this can be figured out via the state_machine so possibly just reflect/use that within impl

    /*
    These are the theoretical states of a `change_state` request
    Note that a CS request can come either from a `ChangeState` srv call processed by `on_change_state`
    Or it can come from internal CS calls using `trigger` functions (often used in tests)

    0. ready to receive a change_state request (srv || trigger)
    1. before primary UDTF (i.e., non-error)
    2. post primary UDTF, before error checking
    3. post error checking
    4. returned / responded

    CHECKS:
    - on_change_state: check if header set, reject immediately if so
    - change_state: (run like a standard coroutine)
        - check if state is non-zero, reject immediately if so
        - set status to 1
    */
};
} // namespace rclcpp_lifecycle