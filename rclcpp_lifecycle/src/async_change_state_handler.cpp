#include "rclcpp_lifecycle/async_change_state_handler.hpp"

namespace rclcpp_lifecycle
{

    AsyncChangeStateHandler::AsyncChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn,
                           std::shared_ptr<AsyncChangeStateHandler>)>
            complete_change_state_cb,
        const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_hdl,
        const std::shared_ptr<rmw_request_id_t> header)
        : complete_change_state_cb_(complete_change_state_cb),
          change_state_hdl_(change_state_hdl),
          header_(header)
    {
    }

    void
    AsyncChangeStateHandler::continue_change_state(
        node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
    {
        complete_change_state_cb_(cb_return_code, shared_from_this()); // TODO @tgroechel: not sure using 'this' makes sense, probably a better way
    }

    void
    AsyncChangeStateHandler::rcl_ret_error()
    {
        _finalize_change_state(false);
    }

    void
    AsyncChangeStateHandler::lifecycle_node_interface_impl_private::_finalize_change_state(
        bool success)
    {
        // TODO @tgroechel: what should we do in the case where it is "invalid"
        if(!_had_valid_header_and_handle())
        {
            return;
        }

        ChangeStateSrv::Response resp;
        resp.success = success;
        change_state_hdl_->send_response(*header_, resp);
        header_.reset();
        change_state_hdl_.reset();
    }

    bool
    AsyncChangeStateHandler::lifecycle_node_interface_impl_private::_has_valid_header_and_handle()
    {
        return header_ && change_state_hdl_;
    }

} // namespace rclcpp_lifecycle