#pragma once

#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager_msgs/controller_manager_msgs/srv/load_controller.hpp>
#include <behaviortree_cpp_v3/action_node.h>


namespace riptide_behavior_tree {

    /**
    * @brief A BT::ActionNodeBase that call the service to load 
    * a ROS2Control controller
    */
    class LoadControllerService : public BT::StatefulActionNode {
        public:
            /**
            * @brief A constructor for riptide_behavior_tree::LoadControllerService
            * @param name Name for the XML tag for this node
            * @param conf BT node configuration
            */
            LoadControllerService(const std::string & name, const BT::NodeConfiguration & conf);

            /**
            * @brief Action executed on start
            * @return BT::NodeStatus Status of tick execution
            */
            BT::NodeStatus onStart() override;

            /**
            * @brief Action executed on start
            * @return BT::NodeStatus Status of tick execution
            */
            BT::NodeStatus onRunning() override;

            /**
            * @brief Action executed on halted
            * @return BT::NodeStatus Status of tick execution
            */
            void onHalted() override {};

            /**
            * @brief Creates list of BT ports
            * @return BT::PortsList Containing node-specific ports
            */
            static BT::PortsList providedPorts() {
                return {
                    BT::InputPort<std::string>("controller_name", "Controller name to load"),
                    BT::InputPort<std::string>("controller_manager", std::string("controller_manager"), "Controller manager node"),
                };
            }

        private:
            /**
             * @brief Callback function for battery topic
             * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
             */
            void requestCallback(rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedFuture future);

            rclcpp::Node::SharedPtr node_;
            rclcpp::CallbackGroup::SharedPtr callback_group_;
            rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
            rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
            std::string controller_name_;
            std::string controller_manager_;
            bool received_response_;
            bool loaded_controller_;
    };

}  // namespace riptide_behavior_tree