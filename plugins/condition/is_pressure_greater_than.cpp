#include <string>

#include "riptide_behavior_tree/plugins/condition/is_pressure_greater_than.hpp"

namespace riptide_behavior_tree {

    IsPressureGreaterThan::IsPressureGreaterThan(const std::string & condition_name, const BT::NodeConfiguration & conf) :
        BT::ConditionNode(condition_name, conf),
        pressure_topic_("~/pressure_status"),
        min_pressure_(20000),
        is_pressure_high_(false)
    {
        getInput("min_pressure", min_pressure_);
        getInput("pressure_topic", pressure_topic_);
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        pressure_sub_ = node_->create_subscription<riptide_msgs::msg::Pressure>(
            pressure_topic_,
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IsPressureGreaterThan::pressureCallback, this, std::placeholders::_1),
            sub_option);
    }

    BT::NodeStatus IsPressureGreaterThan::tick() {
        callback_group_executor_.spin_some();
        if (is_pressure_high_) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IsPressureGreaterThan::pressureCallback(riptide_msgs::msg::Pressure::SharedPtr msg) {
        is_pressure_high_ = msg->pressure >= min_pressure_;
    }
}  // namespace riptide_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<riptide_behavior_tree::IsPressureGreaterThan>("IsPressureGreaterThan");
}