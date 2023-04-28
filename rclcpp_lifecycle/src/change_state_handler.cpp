#include "rclcpp_lifecycle/async_change_state_handler.hpp"

namespace rclcpp_lifecycle
{

    ChangeStateHandler::ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_udtf_cb),
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_error_handling_cb
        : post_udtf_cb_(post_udtf_cb), 
          post_error_handling_cb_(post_error_handling_cb), 
          stage_(ChangeStateStage::READY)
    {
    }

    void
    ChangeStateHandler::continue_change_state(
        node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code)
    {
        if(stage_ == ChangeStateStage::PRE_UDTF)
        {
            stage_ = ChangeStateStage::POST_UDTF;
            post_udtf_cb_(cb_return_code);
        }
        else if(stage_ == ChangeStateStage::POST_UDTF)
        {
            stage_ = ChangeStateStage::POST_ERROR_HANDLING;
            post_error_handling_cb_(cb_return_code);
        }
        // TODO @tgroechel: what to do in case of failure here? Could assert or log warning/error or throw exception?
        //                  Two times I think this could happen off the top of my head:
        //                  1. user defined transition callback calls this twice
        //                  2. error within implementation of lifecycle_node_interface_impl
    }

    bool
    ChangeStateHandler::lifecycle_node_interface_impl_private::_is_ready()
    {
        return stage_ == ChangeStateStage::READY;
    }

    bool
    ChangeStateHandler::lifecycle_node_interface_impl_private::_has_staged_srv_req()
    {
        return stage_ == ChangeStateHandler::STAGED_SRV_REQ;
    }

    void
    ChangeStateHandler::lifecycle_node_interface_impl_private::_start_change_state()
    {
        // TODO @tgroechel: this should probably assert == READY || == STAGED_SRV_REQ
        stage_ = ChangeStateStage::PRE_UDTF;
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
        stage_ = ChangeStateStage::STAGED_SRV_REQ;
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

        stage_ = ChangeStateStage::READY;
    }

    bool
    ChangeStateHandler::lifecycle_node_interface_impl_private::_is_srv_request()
    {
        return header_ != nullptr;
    }

} // namespace rclcpp_lifecycle