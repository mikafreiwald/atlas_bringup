
#include "atlas_bringup/elevation_layer.hpp"

namespace atlas_bringup
{
    ElevationLayer::ElevationLayer()
    {
    }
    ElevationLayer::~ElevationLayer()
    {
    }
    void ElevationLayer::onInitialize()
    {
        // Initialization code here
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
        }

        grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/atlas/local_occupancy_grid_map", 10,
            std::bind(&ElevationLayer::gridCallback, this, std::placeholders::_1));
    }
    void ElevationLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double *min_x, double *min_y, double *max_x, double *max_y)
    {
        // Update bounds code here
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (grid_.data.empty())
        {
            return;
        }
        *min_x = grid_.info.origin.position.x;
        *min_y = grid_.info.origin.position.y;
        *max_x = grid_.info.origin.position.x + grid_.info.width * grid_.info.resolution;
        *max_y = grid_.info.origin.position.y + grid_.info.height * grid_.info.resolution;
        RCLCPP_INFO(logger_, "Updated bounds: min_x: %f, min_y: %f, max_x: %f, max_y: %f",
                    *min_x, *min_y, *max_x, *max_y);
    }
    void ElevationLayer::updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        // Update costs code here
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (grid_.data.empty())
        {
            return;
        }
        for (int i = min_i; i <= max_i; ++i)
        {
            for (int j = min_j; j <= max_j; ++j)
            {
                int8_t cost = grid_.data[i * grid_.info.width + j];
                master_grid.setCost(i, j, cost);
            }
        }
        RCLCPP_INFO(logger_, "Updated costs in the range: [%d, %d] to [%d, %d]", min_i, min_j, max_i, max_j);
    }
    void ElevationLayer::reset()
    {
        // Reset code here
    }
    bool ElevationLayer::isClearable()
    {
        return false; // Example return value
    }

    void ElevationLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Handle the received message
        RCLCPP_INFO(logger_, "Received elevation grid");

        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_.data.clear();
        grid_.header = msg->header;
        grid_.info = msg->info;
        grid_.data.insert(grid_.data.end(), msg->data.begin(), msg->data.end());
    }
} // namespace atlas_bringup

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(atlas_bringup::ElevationLayer, nav2_costmap_2d::Layer)
