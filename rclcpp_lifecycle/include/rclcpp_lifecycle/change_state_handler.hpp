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
// TODO @tgroechel: rename this to ChangeStateHandler as this will handle all the change state requests
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
};
} // namespace rclcpp_lifecycle