
#include "atlas_bringup/hello_world_action.hpp"

namespace atlas_bringup
{
    HelloWorldAction::HelloWorldAction(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::ActionNodeBase(name, config)
    {
    }

    HelloWorldAction::~HelloWorldAction()
    {
    }

    BT::NodeStatus HelloWorldAction::tick()
    {
        RCLCPP_INFO(rclcpp::get_logger("HelloWorldAction"), "[tick] Hello, world!");
        return BT::NodeStatus::SUCCESS;
    }

    void HelloWorldAction::halt()
    {
        RCLCPP_INFO(rclcpp::get_logger("HelloWorldAction"), "[halt] Hello, world!");
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<atlas_bringup::HelloWorldAction>(name, config);
    };
    factory.registerBuilder<atlas_bringup::HelloWorldAction>("HelloWorldAction", builder);
}
