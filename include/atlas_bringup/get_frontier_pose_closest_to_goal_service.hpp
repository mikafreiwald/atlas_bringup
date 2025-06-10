#pragma once

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "geometry_msgs/geometry_msgs/msg/pose_stamped.hpp"
#include "nav3d_msgs/srv/get_frontier_pose_closest_to_goal.hpp"

using namespace nav2_behavior_tree;

namespace atlas_bringup
{
    class GetFrontierPoseClosestToGoalService : public BtServiceNode<nav3d_msgs::srv::GetFrontierPoseClosestToGoal>
    {
    public:
        GetFrontierPoseClosestToGoalService(
            const std::string & service_node_name,
            const BT::NodeConfiguration & conf);

        ~GetFrontierPoseClosestToGoalService() = default;

        void on_tick() override;

        BT::NodeStatus on_completion(std::shared_ptr<nav3d_msgs::srv::GetFrontierPoseClosestToGoal::Response> response) override;

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Target to move closer to along the frontier cells"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("frontier_pose", "closest frontier cell"),
            });
        }
    };
}