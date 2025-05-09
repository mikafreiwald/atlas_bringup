#pragma once

#include "nav2_costmap_2d/costmap_layer.hpp"

namespace atlas_bringup
{
    class ElevationLayer : public nav2_costmap_2d::Layer
    {
    public:
        ElevationLayer();
        ~ElevationLayer() override;

        void onInitialize() override;
        void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override;
        void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override;
        void reset() override;
        bool isClearable() override;
    };
}