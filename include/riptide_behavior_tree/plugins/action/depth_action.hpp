#pragma once

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "riptide_msgs/action/depth.hpp"


namespace riptide_behavior_tree {

    class DepthAction : public nav2_behavior_tree::BtActionNode<riptide_msgs::action::Depth> {
        public:
            /**
             * @brief A constructor for riptide_behavior_tree::DepthAction
             * @param xml_tag_name Name for the XML tag for this node
             * @param action_name Action name this node creates a client for
             * @param conf BT node configuration
             */
            DepthAction(
                const std::string & xml_tag_name,
                const std::string & action_name,
                const BT::NodeConfiguration & conf);

            void on_tick() override;

            /**
             * @brief Creates list of BT ports
             * @return BT::PortsList Containing basic ports along with node-specific ports
             */
            static BT::PortsList providedPorts() {
                return providedBasicPorts( {
                    BT::InputPort<int>("depth", 0, "Requested depth")
                });
            }
    };

}  // namespace riptide_behavior_tree