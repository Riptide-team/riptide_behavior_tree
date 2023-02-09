#include <string>

#include "riptide_behavior_tree/plugins/service/unload_controller.hpp"

#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <controller_manager_msgs/controller_manager_msgs/srv/unload_controller.hpp>


namespace riptide_behavior_tree {

    UnloadControllerService::UnloadControllerService(const std::string &name, const BT::NodeConfiguration & conf) :
        BT::StatefulActionNode(name, conf),
        controller_name_(""),
        controller_manager_("/controller_manager"),
        received_response_(false),
        unloaded_controller_(false)
    {
        getInput("controller_name", controller_name_);
        getInput("controller_manager", controller_manager_);
    }

    BT::NodeStatus UnloadControllerService::onStart() {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        unload_controller_service_ = node_->create_client<controller_manager_msgs::srv::UnloadController>(
            controller_manager_,
            rmw_qos_profile_services_default,
            callback_group_);

        // Sending the request
        auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
        request->name = controller_name_;
        auto result = unload_controller_service_->async_send_request(request, std::bind(&UnloadControllerService::requestCallback, this, std::placeholders::_1));

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus UnloadControllerService::onRunning() {
        callback_group_executor_.spin_some();
        if (received_response_) {
            if (unloaded_controller_) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void UnloadControllerService::requestCallback(rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedFuture future) {
        received_response_ = true;
        auto result = future.get();
        if (result->ok) {
            unloaded_controller_ = true;
        }
    }
}  // namespace riptide_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<riptide_behavior_tree::UnloadControllerService>("UnloadController");
}