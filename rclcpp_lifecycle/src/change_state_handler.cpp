#include "rclcpp_lifecycle/async_change_state_handler.hpp"

namespace rclcpp_lifecycle
{

    ChangeStateHandler::ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn,
                           std::shared_ptr<ChangeStateHandler>)>
            complete_change_state_cb)
        : complete_change_state_cb_(complete_change_state_cb)
    {
    }

    void
    ChangeStateHandler::continue_change_state(
        node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
    {
        complete_change_state_cb_(cb_return_code, shared_from_this()); // TODO @tgroechel: not sure using 'this' makes sense, probably a better way
    }

    void
    ChangeStateHandler::lifecycle_node_interface_impl_private::_set_change_state_srv_hdl(
        const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl)
    {
        change_state_srv_hdl_ = change_state_srv_hdl;    
    }

    void 
    ChangeStateHandler::lifecycle_node_interface_impl_private::_set_rmw_request_id_header(
        const std::shared_ptr<rmw_request_id_t> header)
    {
        header_ = header;
    }

    void
    ChangeStateHandler::lifecycle_node_interface_impl_private::_rcl_ret_error()
    {
        _finalize_change_state(false);
    }

    void
    ChangeStateHandler::lifecycle_node_interface_impl_private::_finalize_change_state(
        bool success)
    {
        if(_is_srv_request())
        {
            ChangeStateSrv::Response resp;
            resp.success = success;
            change_state_srv_hdl_->send_response(*header_, resp);
            header_.reset();
        }
        // TODO @tgroechel: what to do for non-server based finalizing
    }

    bool
    ChangeStateHandler::lifecycle_node_interface_impl_private::_is_srv_request()
    {
        return header_ != nullptr;
    }

} // namespace rclcpp_lifecycle