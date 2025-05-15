
#include "atlas_bringup/elevation_layer.hpp"

namespace atlas_bringup
{
    ElevationLayer::ElevationLayer()
    {
    }
    ElevationLayer::~ElevationLayer()
    {
        auto node = node_.lock();
        filter_.reset();
    }

    void ElevationLayer::onInitialize()
    {
        // Initialization code here
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
        }

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        const auto custom_qos_profile = rclcpp::SensorDataQoS();

        grid_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::OccupancyGrid,
                                                                 rclcpp_lifecycle::LifecycleNode>>(
            node, "/atlas/local_occupancy_grid_map", custom_qos_profile.get_rmw_qos_profile(), sub_opt);

        grid_sub_->unsubscribe();

        filter_ = std::make_shared<message_filters::PassThrough<nav_msgs::msg::OccupancyGrid>>(*grid_sub_);
        filter_->registerCallback(&ElevationLayer::gridCallback, this);
    }

    void ElevationLayer::activate()
    {
        RCLCPP_INFO(logger_, "ElevationLayer activated");
        if (grid_sub_)
        {
            grid_sub_->subscribe();
        }
    }
    void ElevationLayer::deactivate()
    {
        RCLCPP_INFO(logger_, "ElevationLayer deactivated");
        if (grid_sub_)
        {
            grid_sub_->unsubscribe();
        }
    }

    void ElevationLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y)
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (costmap_.getCharMap() == nullptr)
        {
            return;
        }
        costmap_.mapToWorld(0, 0, *min_x, *min_y);
        costmap_.mapToWorld(costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY(), *max_x, *max_y);
        // RCLCPP_INFO(logger_, "Updated bounds: min_x: %f, min_y: %f, max_x: %f, max_y: %f",
        //             *min_x, *min_y, *max_x, *max_y);
    }
    void ElevationLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (costmap_.getCharMap() == nullptr)
        {
            return;
        }

        for (int i = min_i; i < max_i; ++i)
        {
            for (int j = min_j; j < max_j; ++j)
            {
                double wx = 0.0, wy = 0.0;
                master_grid.mapToWorld(i, j, wx, wy);
                unsigned int mx = 0, my = 0;
                if (costmap_.worldToMap(wx, wy, mx, my))
                {
                    unsigned char cost = costmap_.getCost(mx, my);
                    master_grid.setCost(i, j, cost);
                }
                else
                {
                    master_grid.setCost(i, j, nav2_costmap_2d::NO_INFORMATION);
                }
            }
        }
        // RCLCPP_INFO(logger_, "Updated costs in the range: [%d, %d] to [%d, %d]", min_i, min_j, max_i, max_j);
    }
    void ElevationLayer::reset()
    {
        RCLCPP_INFO(logger_, "ElevationLayer reset");
    }
    bool ElevationLayer::isClearable()
    {
        return false; // Example return value
    }

    void ElevationLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // RCLCPP_INFO(logger_, "Received elevation grid");

        std::lock_guard<std::mutex> lock(grid_mutex_);
        // TODO: reuse costmap
        costmap_ = nav2_costmap_2d::Costmap2D(*msg);
    }
} // namespace atlas_bringup

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(atlas_bringup::ElevationLayer, nav2_costmap_2d::Layer)
