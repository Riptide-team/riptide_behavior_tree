#include "riptide_behavior_tree/plugins/control/say_someting.hpp"

#include <iostream>

namespace riptide_behavior_tree {
    BT::NodeStatus SaySomething::tick() {
        auto msg = getInput<std::string>("message");
        if (!msg) {
            throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
        }

        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // riptide_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<riptide_behavior_tree::SaySomething>("SaySomething");
}