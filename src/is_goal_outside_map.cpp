
#include "atlas_bringup/is_goal_outside_map.hpp"
#include <memory>

namespace atlas_bringup
{

IsGoalOutsideMap::IsGoalOutsideMap(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : AreErrorCodesPresent(condition_name, conf)
{
    error_codes_to_check_ = {
        ActionResult::GOAL_OUTSIDE_MAP,
        ThroughActionResult::GOAL_OUTSIDE_MAP
    };
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<atlas_bringup::IsGoalOutsideMap>(
    "IsGoalOutsideMap");
}
