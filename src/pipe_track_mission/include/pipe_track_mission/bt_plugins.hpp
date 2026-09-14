#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <behaviortree_cpp/behavior_tree.h>

// --- SHARED DATA ---
// This struct acts as a bridge between ROS subscriptions and the BT nodes
struct AuvContext {
    bool is_exploration_empty = false;
    bool is_turning_back = false;
    int turn_back_counter = 0;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
    rclcpp::Logger logger = rclcpp::get_logger("bt_node");
};

// --- CUSTOM BT NODES ---
class IsExplorationEmpty : public BT::ConditionNode {
public:
    IsExplorationEmpty(const std::string& name, std::shared_ptr<AuvContext> ctx)
        : BT::ConditionNode(name, {}), ctx_(ctx) {}
        
    BT::NodeStatus tick() override {
        return ctx_->is_exploration_empty ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
private:
    std::shared_ptr<AuvContext> ctx_;
};

class IsConstantlyTurningBack : public BT::ConditionNode {
public:
    IsConstantlyTurningBack(const std::string& name, std::shared_ptr<AuvContext> ctx)
        : BT::ConditionNode(name, {}), ctx_(ctx) {}
        
    BT::NodeStatus tick() override {
        // If we receive the turn back signal for roughly 3 seconds (assuming 10Hz tick)
        if (ctx_->is_turning_back) {
            ctx_->turn_back_counter++;
        } else {
            ctx_->turn_back_counter = 0; // Reset if signal drops
        }
        
        if (ctx_->turn_back_counter > 30) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
private:
    std::shared_ptr<AuvContext> ctx_;
};

class StopAUV : public BT::SyncActionNode {
public:
    StopAUV(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<AuvContext> ctx)
        : BT::SyncActionNode(name, config), ctx_(ctx) {}
        
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("reason") };
    }

    BT::NodeStatus tick() override {
        std::string reason;
        getInput("reason", reason);
        RCLCPP_ERROR(ctx_->logger, "🛑 STOPPING AUV: %s", reason.c_str());

        // Trigger the finished_execution flag in your WaypointControlNode
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        ctx_->stop_pub->publish(stop_msg);

        return BT::NodeStatus::SUCCESS;
    }
private:
    std::shared_ptr<AuvContext> ctx_;
};

class KeepRunning : public BT::SyncActionNode {
public:
    KeepRunning(const std::string& name) : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::RUNNING; }
};