#include <string>
#include <memory>

#include "riptide_behavior_tree/plugins/action/depth_action.hpp"


namespace riptide_behavior_tree {

    DepthAction::DepthAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf)
        : nav2_behavior_tree::BtActionNode<riptide_msgs::action::Depth>(xml_tag_name, action_name, conf)
    {
        float depth;
        getInput("depth", depth);
        if (depth <= 0) {
            RCLCPP_WARN(
            node_->get_logger(), "Depth is negative or zero "
            "(%f). Setting to positive.", depth);
            depth *= -1;
        }

        goal_.depth = depth;
    }

    void DepthAction::on_tick() {
        increment_recovery_count();
    }

}  // namespace riptide_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder =
        [](const std::string & name, const BT::NodeConfiguration & config) {
            return std::make_unique<riptide_behavior_tree::DepthAction>(name, "depth", config);
        };

    factory.registerBuilder<riptide_behavior_tree::DepthAction>("DepthAction", builder);
}