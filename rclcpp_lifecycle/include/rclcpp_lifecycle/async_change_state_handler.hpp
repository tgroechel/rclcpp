#pragma once // TODO @tgroechel: check if they use pragma once or ifndef

// TODO @tgroechel: clean up includes here
#include <memory>
#include <vector>
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

    void complete_change_state(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

    void rcl_ret_error();

    void send_response(bool success); // TODO @tgroechel: hide this so only lifecycle impl can call?

private:
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn,
                        std::shared_ptr<AsyncChangeStateHandler>)>
        complete_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl_;
    const std::shared_ptr<rmw_request_id_t> header_;
};
} // namespace rclcpp_lifecycle