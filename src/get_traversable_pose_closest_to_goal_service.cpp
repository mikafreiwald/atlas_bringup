
#include "atlas_bringup/get_traversable_pose_closest_to_goal_service.hpp"

namespace atlas_bringup
{
    GetTraversablePoseClosestToGoalService::GetTraversablePoseClosestToGoalService(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BtServiceNode(name, config)
    {
    }

    void GetTraversablePoseClosestToGoalService::on_tick()
    {
        getInput("goal_pose", request_->goal_pose);
    }

    BT::NodeStatus GetTraversablePoseClosestToGoalService::on_completion(
        std::shared_ptr<nav3d_msgs::srv::GetTraversablePoseClosestToGoal::Response> response
    ) {
        
        if (response->status == nav3d_msgs::srv::GetTraversablePoseClosestToGoal::Response::STATUS_SUCCESS) {
            setOutput("traversable_pose", response->traversable_pose);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<atlas_bringup::GetTraversablePoseClosestToGoalService>("GetTraversablePoseClosestToGoal");
}
