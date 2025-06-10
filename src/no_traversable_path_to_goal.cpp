
#include "atlas_bringup/no_traversable_path_to_goal.hpp"
#include <memory>

namespace atlas_bringup
{

NoTraversablePathToGoal::NoTraversablePathToGoal(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : AreErrorCodesPresent(condition_name, conf)
{
    error_codes_to_check_ = {
        ActionResult::NO_VALID_PATH,
        ThroughActionResult::NO_VALID_PATH
    };
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<atlas_bringup::NoTraversablePathToGoal>(
    "NoTraversablePathToGoal");
}
