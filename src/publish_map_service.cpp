
#include "atlas_bringup/publish_map_service.hpp"

namespace atlas_bringup
{
    PublishMapService::PublishMapService(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BtServiceNode(name, config)
    {
    }

    void PublishMapService::on_tick()
    {
        getInput("resolution", request_->resolution);
        getInput("skip_first_cloud", request_->skip_first_cloud);
        RCLCPP_INFO(rclcpp::get_logger("PublishMapService"),
            "[on_tick] resolution: %f, skip_first_cloud %d", request_->resolution, request_->skip_first_cloud);
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<atlas_bringup::PublishMapService>("PublishMap");
}
