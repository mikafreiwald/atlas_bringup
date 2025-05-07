#pragma once

#include "nav2_behavior_tree/bt_action_node.hpp"

namespace atlas_bringup
{
    class HelloWorldAction : public BT::ActionNodeBase
    {
    public:
        HelloWorldAction(
            const std::string &name,
            const BT::NodeConfiguration &config);
        ~HelloWorldAction();

        BT::NodeStatus tick() override;
        void halt() override;

        static BT::PortsList providedPorts()
        {
            return {};
        }
    };
}