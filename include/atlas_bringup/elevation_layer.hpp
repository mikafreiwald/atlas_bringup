#pragma once

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/pass_through.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

namespace atlas_bringup
{
    class ElevationLayer : public nav2_costmap_2d::Layer
    {
    public:
        ElevationLayer();
        ~ElevationLayer() override;

        void deactivate() override;
        void activate() override;

        void onInitialize() override;
        void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override;
        void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override;
        void reset() override;
        bool isClearable() override;

    private:
        void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        rclcpp::Logger logger_{rclcpp::get_logger("ElevationLayer")};
        std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid, rclcpp_lifecycle::LifecycleNode>> grid_sub_;
        std::shared_ptr<message_filters::PassThrough<nav_msgs::msg::OccupancyGrid>> filter_;

        std::mutex grid_mutex_;
        nav2_costmap_2d::Costmap2D costmap_;
    };
}