#pragma once

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "mrg_slam_msgs/srv/publish_map.hpp"

using namespace nav2_behavior_tree;

namespace atlas_bringup
{
    class PublishMapService : public BtServiceNode<mrg_slam_msgs::srv::PublishMap>
    {
    public:
        PublishMapService(
            const std::string & service_node_name,
            const BT::NodeConfiguration & conf);

        ~PublishMapService();

        BT::NodeStatus tick() override;
        void halt() override;

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<double>("resolution", 0.1, "Resolution")
            });
        }
    };
}