#pragma once

#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

namespace riptide_behavior_tree {

    /**
    * @brief A BT::ConditionNode that listens to a battery topic and
    * returns SUCCESS when battery is low and FAILURE otherwise
    */
    class IsBatteryLowerThanCondition : public BT::ConditionNode {
        public:
            /**
            * @brief A constructor for riptide_behavior_tree::IsBatteryLowCondition
            * @param condition_name Name for the XML tag for this node
            * @param conf BT node configuration
            */
            IsBatteryLowerThanCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

            IsBatteryLowerThanCondition() = delete;

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
                    BT::InputPort<double>("min_battery", "Minimum battery voltage"),
                    BT::InputPort<std::string>("battery_topic", std::string("~/battery_status"), "Battery topic"),
                };
            }

        private:
            /**
             * @brief Callback function for battery topic
             * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
             */
            void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

            rclcpp::Node::SharedPtr node_;
            rclcpp::CallbackGroup::SharedPtr callback_group_;
            rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
            std::string battery_topic_;
            double min_battery_;
            bool is_battery_low_;
    };

}  // namespace riptide_behavior_tree