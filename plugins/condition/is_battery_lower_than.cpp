#include <string>

#include "riptide_behavior_tree/plugins/condition/is_battery_lower_than.hpp"

namespace riptide_behavior_tree {

    IsBatteryLowerThanCondition::IsBatteryLowerThanCondition(const std::string & condition_name, const BT::NodeConfiguration & conf) :
        BT::ConditionNode(condition_name, conf),
        battery_topic_("~/battery_status"),
        min_battery_(0.0),
        is_battery_low_(false) 
    {
        getInput("min_battery", min_battery_);
        getInput("battery_topic", battery_topic_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsBatteryLowerThanCondition::batteryCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsBatteryLowerThanCondition::tick() {
        callback_group_executor_.spin_some();
        if (is_battery_low_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IsBatteryLowerThanCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg) {
        is_battery_low_ = msg->voltage <= min_battery_;
    }
}  // namespace riptide_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<riptide_behavior_tree::IsBatteryLowerThanCondition>("IsBatteryLowerThan");
}