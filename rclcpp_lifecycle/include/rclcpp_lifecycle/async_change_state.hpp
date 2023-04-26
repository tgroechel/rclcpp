#pragma once // TODO @tgroechel: check if they use pragma once or ifndef

// TODO @tgroechel: clean up includes here
#include <memory>
#include <vector>
#include <functional>
#include "rclcpp_lifecycle/life_cycle_node_interface.hpp" // CallbackReturn

namespace rclcpp_lifecycle
{
/*
*  Used for async user defined transtion callbacks
*/
class AsyncChangeState
{
    AsyncChangeState(
        std::function<void(CallbackReturn,
                            std::shared_ptr<rclcpp::Service<ChangeStateSrv>>,
                            std::shared_ptr<rmw_request_id_t>)>
            complete_change_state_cb,
        const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl,
        const std::shared_ptr<rmw_request_id_t> header);

    void complete_change_state(CallbackReturn cb_return_code);

    void rcl_ret_error();

    void send_response(bool success);

private:
    std::function<void(CallbackReturn,
                        std::shared_ptr<rclcpp::Service<ChangeStateSrv>>,
                        std::shared_ptr<rmw_request_id_t>)>
        complete_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl_;
    const std::shared_ptr<rmw_request_id_t> header_;
};
} // namespace rclcpp_lifecycle