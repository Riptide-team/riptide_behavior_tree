#include "riptide_behavior_tree/plugins/decorator/inverter_node.hpp"

namespace riptide_behavior_tree {
    InverterNode::InverterNode(const std::string& name) : BT::DecoratorNode(name, {}) {
        setRegistrationID("Inverter");
    }

    BT::NodeStatus InverterNode::tick() {
        const BT::NodeStatus child_status = child_node_->executeTick();

        switch (child_status) {
            case BT::NodeStatus::SUCCESS: {
                resetChild();
                return BT::NodeStatus::FAILURE;
            }

            case BT::NodeStatus::FAILURE: {
                resetChild();
                return BT::NodeStatus::SUCCESS;
            }

            case BT::NodeStatus::RUNNING: {
                return BT::NodeStatus::RUNNING;
            }

            case BT::NodeStatus::IDLE: {
                throw BT::LogicError("[", name(), "]: A children should not return IDLE");
            }
        }
        return status();
    }
} // namespace riptide_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<riptide_behavior_tree::InverterNode>("Inverter");
}