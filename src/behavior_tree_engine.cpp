#include "riptide_behavior_tree/behavior_tree_engine.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace riptide_behavior_tree {
    BehaviorTreeEngine::BehaviorTreeEngine() : Node("bt_engine") {
        configure_parameters();
        load_plugins();
        load_tree();
        if (run_groot_monitoring_) {
            add_groot_monitoring();
        }
        run();
    }

    void BehaviorTreeEngine::configure_parameters() {
        bt_file_path_ = this->declare_parameter("bt_file_path", "tree.xml");
        loop_timeout_ = std::chrono::milliseconds(this->declare_parameter("loop_timeout", 100));
        bt_loop_duration_ = std::chrono::milliseconds(this->declare_parameter("bt_loop_duration", 50));
        plugins_ = this->declare_parameter("plugins", std::vector<std::string>());
        // Groot
        run_groot_monitoring_ = this->declare_parameter("run_groot_monitoring", true);
        publisher_port_ = this->declare_parameter("publisher_port", 1666);
        server_port_ = this->declare_parameter("server_port", 1667);
        max_msg_per_second_ = this->declare_parameter("max_msg_per_second", 25);
    }

    void BehaviorTreeEngine::load_tree() {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<rclcpp::Node::SharedPtr>("node", std::make_shared<rclcpp::Node>("bt_node"));
        blackboard->set<std::chrono::milliseconds>("server_timeout", loop_timeout_);  // NOLINT 
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        //  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);
        RCLCPP_INFO(this->get_logger(), "Loading tree from file: %s", bt_file_path_.c_str());
        tree_ = std::make_shared<BT::Tree>(factory_.createTreeFromFile(bt_file_path_, blackboard));
        RCLCPP_INFO(this->get_logger(), "Success!");
    }

    void BehaviorTreeEngine::run() {
        rclcpp::WallRate loop_rate(loop_timeout_);
        RCLCPP_INFO_STREAM(
            this->get_logger(), "Running tree at frequency " <<
            1.0 / loop_timeout_.count() * 1e3 << "Hz");
        while (rclcpp::ok()) {
            tree_->tickRoot();
            loop_rate.sleep();
        }
    }

    void BehaviorTreeEngine::add_groot_monitoring() {
        RCLCPP_INFO_STREAM(
            this->get_logger(), "Groot monitoring enabled with server port [" <<
            server_port_ << "] and publisher port [" << publisher_port_ << "]");
        groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
            *tree_, max_msg_per_second_, publisher_port_, server_port_);
    }

    void BehaviorTreeEngine::load_plugins() {
        for (const auto & p : plugins_) {
            RCLCPP_INFO(this->get_logger(), "Loading plugin: %s", (BT::SharedLibrary::getOSName(p)).c_str());
            factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
        }
    }
}  // namespace riptide_behavior_tree


void sigint_handler(__attribute__((unused)) int signal_num) { // Silences compiler warnings
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);
    rclcpp::spin(std::make_shared<riptide_behavior_tree::BehaviorTreeEngine>());
    rclcpp::shutdown();
    return 0;
}