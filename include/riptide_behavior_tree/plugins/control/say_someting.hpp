#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace riptide_behavior_tree {

    class SaySomething : public BT::SyncActionNode {
    public:
        SaySomething(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}

        // You must override the virtual function tick()
        BT::NodeStatus tick() override;

        // It is mandatory to define this static method.
        static BT::PortsList providedPorts() {
            return{ BT::InputPort<std::string>("message") };
        }
    };
} // riptide_behavior_tree