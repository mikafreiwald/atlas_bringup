
#include "atlas_bringup/get_frontier_pose_closest_to_goal_service.hpp"

namespace atlas_bringup
{
    GetFrontierPoseClosestToGoalService::GetFrontierPoseClosestToGoalService(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BtServiceNode(name, config)
    {
    }

    void GetFrontierPoseClosestToGoalService::on_tick()
    {
        getInput("goal_pose", request_->goal_pose);
    }

    BT::NodeStatus GetFrontierPoseClosestToGoalService::on_completion(
        std::shared_ptr<nav3d_msgs::srv::GetFrontierPoseClosestToGoal::Response> response
    ) {
        
        if (response->status == nav3d_msgs::srv::GetFrontierPoseClosestToGoal::Response::STATUS_SUCCESS) {
            setOutput("frontier_pose", response->frontier_pose);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<atlas_bringup::GetFrontierPoseClosestToGoalService>("GetFrontierPoseClosestToGoal");
}
