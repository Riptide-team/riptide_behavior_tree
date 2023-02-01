#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace riptide_behavior_tree {
    /**
     * @brief The InverterNode returns SUCCESS if child fails
     * of FAILURE is child succeeds.
     * RUNNING status is propagated
     */
    class InverterNode : public BT::DecoratorNode {
        public:
            InverterNode(const std::string& name);

            virtual ~InverterNode() override = default;

        private:
            virtual BT::NodeStatus tick() override;
    };
} // namespace riptide_behavior_tree