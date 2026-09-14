#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include "pipe_track_mission/bt_plugins.hpp" 

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace std::chrono_literals;

// --- MAIN ROS NODE ---
class BehaviourNode : public rclcpp::Node {
public:
    BehaviourNode() : Node("behaviour_node") {
        ctx_ = std::make_shared<AuvContext>();
        ctx_->logger = this->get_logger();
        
        // Declare parameter to choose which XML file to load
        this->declare_parameter<std::string>("tree_xml_file", "reactive_mission.xml");
        
        // Subscriptions to your planning nodes
        sub_exploration_ = this->create_subscription<std_msgs::msg::Bool>(
            "/planning/exploration_empty", 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) { ctx_->is_exploration_empty = msg->data; });
            
        sub_turn_back_ = this->create_subscription<std_msgs::msg::Bool>(
            "/planning/turn_back", 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) { ctx_->is_turning_back = msg->data; });

        // Publisher to stop the waypoint node
        ctx_->stop_pub = this->create_publisher<std_msgs::msg::Bool>("/movement/finished_execution", 10);

        // Register BT Nodes
        BT::BehaviorTreeFactory factory;
        factory.registerBuilder<IsExplorationEmpty>("IsExplorationEmpty", 
            [this](const std::string& name, const BT::NodeConfig&) { return std::make_unique<IsExplorationEmpty>(name, ctx_); });
        
        factory.registerBuilder<IsConstantlyTurningBack>("IsConstantlyTurningBack", 
            [this](const std::string& name, const BT::NodeConfig&) { return std::make_unique<IsConstantlyTurningBack>(name, ctx_); });
            
        factory.registerBuilder<StopAUV>("StopAUV", 
            [this](const std::string& name, const BT::NodeConfig& config) { return std::make_unique<StopAUV>(name, config, ctx_); });
            
        // factory.registerNodeType<KeepRunning>("KeepRunning");

        // Load the tree
        std::string xml_file = this->get_parameter("tree_xml_file").as_string();

        if (xml_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "XML file parameter is empty!");
            throw std::runtime_error("Empty XML file path");
        }

        // If it's not already an absolute path, build the absolute path automatically
        if (!std::filesystem::path(xml_file).is_absolute()) {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("pipe_track_mission");
            xml_file = pkg_share + "/config/" + xml_file;
        }

        tree_ = factory.createTreeFromFile(xml_file);
        BT::Groot2Publisher publisher(tree_);
        RCLCPP_INFO(this->get_logger(), "Behavior Tree loaded from: %s", xml_file.c_str());

        // Tick the tree at 10Hz
        timer_ = this->create_wall_timer(100ms, [this]() { tree_.tickExactlyOnce(); });
    }

private:
    std::shared_ptr<AuvContext> ctx_;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_exploration_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_turn_back_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviourNode>());
    rclcpp::shutdown();
    return 0;
}