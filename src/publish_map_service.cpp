
#include "atlas_bringup/publish_map_service.hpp"

namespace atlas_bringup
{
    PublishMapService::PublishMapService(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BtServiceNode(name, config)
    {
    }

    PublishMapService::~PublishMapService()
    {
    }

    BT::NodeStatus PublishMapService::tick()
    {
        RCLCPP_INFO(rclcpp::get_logger("HelloWorldAction"), "[tick] Hello, world!");
        return BT::NodeStatus::SUCCESS;
    }

    void PublishMapService::halt()
    {
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<atlas_bringup::PublishMapService>(name, config);
    };
    factory.registerBuilder<atlas_bringup::PublishMapService>("PublishMap", builder);
}
