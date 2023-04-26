#include "async_change_state.hpp"

/*
 Used for async user defined transtion callbacks
*/
namespace lifecycle_node
{

AsyncChangeState::AsyncChangeState(
    std::function<void(CallbackReturn,
                        std::shared_ptr<rclcpp::Service<ChangeStateSrv>>, 
                        std::shared_ptr<rmw_request_id_t>)>
                    complete_change_state_cb,
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl,
    const std::shared_ptr<rmw_request_id_t> header)
    : complete_change_state_cb_(complete_change_state_cb),
        change_state_hdl_(change_state_hdl),
        header_(header)
{
}

void
AsyncChangeState::complete_change_state(
    rcl_ret_t cb_return_code)
{
    complete_change_state_cb_(cb_return_code);
}

void 
AsyncChangeState::rcl_ret_error()
{
    ChangeStateSrv::Response resp;
    resp.success = false;
    change_state_hdl->send_response(resp, *header_);
}

} // namespace lifecycle_node
