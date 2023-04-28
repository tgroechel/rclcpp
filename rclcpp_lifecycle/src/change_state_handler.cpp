// TODO @tgroechel: liscense
#include "rclcpp_lifecycle/async_change_state_handler.hpp"
#include <cassert> // TODO @tgroechel: does rclcpp like or dislike asserts?

namespace rclcpp_lifecycle
{

    ChangeStateHandler::ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_udtf_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_on_error_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            finalize_change_state_cb)
        : post_udtf_cb_(post_udtf_cb),
          post_on_error_cb_(post_on_error_cb),
          finalize_change_state_cb_(finalize_change_state_cb),
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
            stage_ = ChangeStateStage::POST_ON_ERROR;
            post_on_error_cb_(cb_return_code);
        }
        else if(stage_ == ChangeStateStage::FINALIZING)
        {
            finalize_change_state_cb_(cb_return_code);
        }

        // TODO @tgroechel: what to do in case of failure here? Could assert or log warning/error or throw exception? Silently do nothing?
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
        assert(stage_ == ChangeStateStage::READY || stage_ == ChangeStateStage::STAGED_SRV_REQ);
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
        assert(stage_ == ChangeStateStage::READY);
        stage_ = ChangeStateStage::STAGED_SRV_REQ;
        header_ = header;
    }

    void 
    ChangeStateHandler::lifecycle_node_interface_impl_private::_no_error_from_udtf(
        const std::shared_ptr<rmw_request_id_t> header)
    {
        assert(stage_ == ChangeStateStage::POST_UDTF);
        stage_ = ChangeStateStage::FINALIZING;
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
        // TODO @tgroechel: what to do for non-server based finalizing, this goes along with traditional internal triggers, possibly hold some return value for them throughout

        stage_ = ChangeStateStage::READY;
    }

    bool
    ChangeStateHandler::lifecycle_node_interface_impl_private::_is_srv_request()
    {
        return header_ != nullptr;
    }

} // namespace rclcpp_lifecycle