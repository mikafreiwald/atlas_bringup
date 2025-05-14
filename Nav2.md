
# Nav2 Components

[Nav2 Plugins](https://docs.nav2.org/plugins/index.html#costmap-layers)

## Costmap

- Node type: Costmap2DRos
- contains LayeredCostmap
    - has 2 Layer Types: Plugin, Filter
    - combines Layer into 1 Costmap2D (holds unsigned char[])

### Interface

**Layer**
- updates master costmap

**CostmapLayer**: Layer, Costmap2D
- updates master costmap
- has internal costmap to maintain state

### Nav2 Layer Plugins

- StaticLayer
- VoxelLayer
- InflationLayer
- DenoiseLayer

## Planner Server

Creates a plan from global costmap.

Nodes:
- planner_server
- global_costmap

### Interface

**GlobalPlanner**
- creates path from start to goal
- has access to global costmap

### Nav2 Planner Plugins

- NavfnPlanner
- SmacPlanner

## Controller Server

Follow a global path with local costmap, progress checker and goal checker.

Nodes:
- controller_server
- local_costmap

### Interface

**Controller**
- repeatedly called to compute velocity command
- has access to local costmap, current path and goal checker
- can be canceled

**ProgressChecker**
- checks if robot has moved compared to previous pose

**GoalChecker**
- determines if the goal should be considered reached

### Nav2 Controller Plugins

- MPPI Controller
    - containes multiple critcs for navigation

## Map Server

Published a map from a file

## Behavior Tree Navigator

Receives goal and executes task as described in xml BT to reach goal.
Creates an action server for each possbile task (plugin).

Nodes:
- bt_navigator

### Interface

**BehaviorTreeNavigator<ActionT>**
- react to events: goal received, preempted, goal completed
- ActionT is the action type for the created action server

### Nav2 Behavior Tree Navigator Plugins

- Navigate To Pose Navigator
- Navigate Through Poses Navigator

## Behavior Server

Contains behaviors used to recover from failure conditions.
Create an action server for each defined behavior (plugin).

Nodes:
- behavior_server

### Interface

**TimedBehavior<ActionT>** : Behavior
- needs BT node if used inside BT xml file
- called cyclically while it returns RUNNING
- has access to local and global collision checker

### Nav2 Behaviors

- Clear Costmap
- Spin
- Back Up
- Wait
- Drive On Heading
- Assisted Teleop