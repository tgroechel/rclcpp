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
class AsyncChangeStateHandler : public std::enable_shared_from_this<AsyncChangeStateHandler>
{
public:
    using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
    AsyncChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn,
                        std::shared_ptr<AsyncChangeStateHandler>)>
            complete_change_state_cb,
        const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl,
        const std::shared_ptr<rmw_request_id_t> header);

    void continue_change_state(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

    void rcl_ret_error();

    // Calling outside of lifecycle_node_interface_impl is undefined behavior
    // Avoids needing to forward declare lifecycle_node_interface_impl + friend
    namespace lifecycle_node_interface_impl_private
    {
    void _finalize_change_state(bool success);
    bool _has_valid_header_and_handle();
    }

private:
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn,
                        std::shared_ptr<AsyncChangeStateHandler>)>
        complete_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl_;
    const std::shared_ptr<rmw_request_id_t> header_;
    std::atomic<bool> in_transition_{false}; // TODO @tgroechel: this can be figured out via the state_machine so possibly just reflect/use that within impl
};
} // namespace rclcpp_lifecycle