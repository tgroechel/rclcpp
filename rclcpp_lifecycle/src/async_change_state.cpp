#include "rclcpp_lifecycle/async_change_service.hpp"


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
    send_response(false);
}

void
AsyncChangeState::send_response(
    bool success)
{
    ChangeStateSrv::Response resp;
    resp.success = success;
    change_state_hdl->send_response(resp, *header_);

} // namespace lifecycle_node
