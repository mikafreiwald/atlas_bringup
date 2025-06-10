#pragma once

#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"

namespace atlas_bringup
{

class IsGoalOutsideMap : public nav2_behavior_tree::AreErrorCodesPresent
{
    using ActionResult = nav2_msgs::action::ComputePathToPose::Result;
    using ThroughActionResult = nav2_msgs::action::ComputePathThroughPoses::Result;

public:
    IsGoalOutsideMap(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    IsGoalOutsideMap() = delete;
};

}
