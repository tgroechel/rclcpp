// TODO @tgroechel: liscense
#pragma once // TODO @tgroechel: check if they use pragma once or ifndef


// TODO @tgroechel: clean up includes here
#include <memory>
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
// TODO @tgroechel: comments for functions same as rclcpp style
// TODO @tgroechel: should I rename UDTF and relayted to user transitoin function spelled out? probably
// TODO @tgroechel: split this and move it into lifecycle and interface_impl in order to avoid lifecycle_node_interface_impl_private
class ChangeStateHandler
{
public:
    using ChangeStateSrv = lifecycle_msgs::srv::ChangeState;
    ChangeStateHandler(
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_udtf_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            post_on_error_cb,
        std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
            finalize_change_state_cb);

    void continue_change_state(node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code);

    namespace lifecycle_node_interface_impl_private
    {
    bool _is_ready();
    bool _has_staged_srv_req()
    void _start_change_state();
    void _set_change_state_srv_hdl(const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl);
    void _set_rmw_request_id_header(const std::shared_ptr<rmw_request_id_t> header);    
    void _no_error_from_udtf();
    void _rcl_ret_error();
    void _finalize_change_state(bool success);
    bool _is_srv_request();
    }

private:
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_udtf_cb_;
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        post_on_error_cb_;
    std::function<void(node_interfaces::LifecycleNodeInterface::CallbackReturn)>
        finalize_change_state_cb_;
    const std::shared_ptr<rclcpp::Service<ChangeStateSrv>> change_state_srv_hdl_; 
    const std::shared_ptr<rmw_request_id_t> header_;

   /*
   READY            -> {STAGE_SRV_REQ, PRE_UDTF}
   STAGE_SRV_REQ    -> {PRE_UDTF}
   PRE_UDTF         -> {POST_UDTF}                      // change_state
   POST_UDTF        -> {POST_ON_ERROR, FINALIZING}      // post_udtf_cb_
   POST_ON_ERROR    -> {FINALIZING}                     // handle_post_on_error_cb_
   FINALIZING       -> {READY}                          // finalize_change_state_cb_
   ***ANY***        -> {FINALIZING}                     // i.e., early exit of change_state
   */
   enum ChangeStateStage
   {
        READY,
        STAGED_SRV_REQ,
        PRE_UDTF,
        POST_UDTF,
        POST_ON_ERROR,
        FINALIZING
   };

   ChangeStateStage stage_;
};
} // namespace rclcpp_lifecycle