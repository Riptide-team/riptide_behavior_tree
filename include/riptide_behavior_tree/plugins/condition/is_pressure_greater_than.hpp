#pragma once

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "riptide_msgs/msg/pressure.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace riptide_behavior_tree {

    /**
    * @brief A BT::ConditionNode that listens to a battery topic and
    * returns SUCCESS when battery is low and FAILURE otherwise
    */
    class IsPressureGreaterThan : public BT::ConditionNode {
        public:
            /**
            * @brief A constructor for riptide_behavior_tree::IsPressureGreaterThan
            * @param condition_name Name for the XML tag for this node
            * @param conf BT node configuration
            */
            IsPressureGreaterThan(const std::string & condition_name, const BT::NodeConfiguration & conf);

            IsPressureGreaterThan() = delete;

            /**
            * @brief The main override required by a BT action
            * @return BT::NodeStatus Status of tick execution
            */
            BT::NodeStatus tick() override;

            /**
            * @brief Creates list of BT ports
            * @return BT::PortsList Containing node-specific ports
            */
            static BT::PortsList providedPorts() {
                return {
                    BT::InputPort<double>("min_pressure", "Minimum pressure"),
                    BT::InputPort<std::string>("pressure_topic", std::string("~/pressure_status"), "Pressure topic"),
                };
            }

        private:
            /**
             * @brief Callback function for pressure topic
             * @param msg Shared pointer to riptide_msgs::msg::Pressure message
             */
            void pressureCallback(riptide_msgs::msg::Pressure::SharedPtr msg);

            rclcpp::Node::SharedPtr node_;
            rclcpp::CallbackGroup::SharedPtr callback_group_;
            rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
            rclcpp::Subscription<riptide_msgs::msg::Pressure>::SharedPtr pressure_sub_;
            std::string pressure_topic_;
            double min_pressure_;
            bool is_pressure_high_;
    };

}  // namespace riptide_behavior_tree