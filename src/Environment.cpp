/*
 *  This file is part of qpOASES.
 *
 *  Environment: Grid map and obstacle management for multi-agent planning
 *
 *  Implementation
 */

#include <qpOASES/Environment.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>

BEGIN_NAMESPACE_QPOASES

Environment::Environment(real_t x_min, real_t x_max, real_t y_min, real_t y_max, real_t grid_resolution)
    : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), grid_resolution_(grid_resolution)
{
    // Validate bounds
    if (x_max <= x_min || y_max <= y_min) {
        printf("WARNING: Invalid environment bounds! x_max=%f, x_min=%f, y_max=%f, y_min=%f\n",
               x_max, x_min, y_max, y_min);
    }
    
    if (grid_resolution <= 0.0) {
        printf("WARNING: Invalid grid resolution %f, using default 0.1\n", grid_resolution);
        grid_resolution_ = 0.1;
    }
}

Environment::~Environment()
{
    // Nothing to clean up (using std::vector)
}

int Environment::addObstacle(const RectangularObstacle& obstacle)
{
    obstacles_.push_back(obstacle);
    return (int)obstacles_.size() - 1;
}

bool Environment::isCollisionFree(real_t x, real_t y, real_t r_agent) const
{
    // Check all obstacles
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        if (isInsideObstacle(x, y, obstacles_[i], r_agent)) {
            return false;  // Collision detected
        }
    }
    
    return true;  // No collision
}

bool Environment::isInBounds(real_t x, real_t y) const
{
    return (x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_);
}

void Environment::getBounds(real_t& x_min, real_t& x_max, real_t& y_min, real_t& y_max) const
{
    x_min = x_min_;
    x_max = x_max_;
    y_min = y_min_;
    y_max = y_max_;
}

bool Environment::isInsideObstacle(real_t x, real_t y, const RectangularObstacle& obstacle, real_t r_agent) const
{
    // Transform point to obstacle's local frame
    real_t dx = x - obstacle.x_center;
    real_t dy = y - obstacle.y_center;
    
    real_t cos_theta = cos(-obstacle.theta);
    real_t sin_theta = sin(-obstacle.theta);
    
    real_t x_local = cos_theta * dx - sin_theta * dy;
    real_t y_local = sin_theta * dx + cos_theta * dy;
    
    // Check if inside obstacle (with safety margin)
    return (fabs(x_local) < obstacle.width / 2.0 + r_agent) &&
           (fabs(y_local) < obstacle.height / 2.0 + r_agent);
}

void Environment::printASCII(const Point2D* agent_positions, int n_agents) const
{
    // ASCII visualization parameters
    const int width = 60;   // Characters wide
    const int height = 30;  // Characters tall
    
    printf("\n=== Environment (ASCII) ===\n");
    printf("Bounds: [%.1f, %.1f] x [%.1f, %.1f]\n", x_min_, x_max_, y_min_, y_max_);
    printf("Obstacles: %d\n\n", (int)obstacles_.size());
    
    // Create grid
    char grid[height][width];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            grid[i][j] = '.';  // Empty space
        }
    }
    
    // Draw obstacles
    for (size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        const RectangularObstacle& obs = obstacles_[obs_idx];
        
        // Sample obstacle boundary
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                // Convert grid coordinates to world coordinates
                real_t x = x_min_ + (x_max_ - x_min_) * j / (width - 1);
                real_t y = y_max_ - (y_max_ - y_min_) * i / (height - 1);  // Flip y
                
                if (isInsideObstacle(x, y, obs, 0.0)) {
                    grid[i][j] = '#';  // Obstacle
                }
            }
        }
    }
    
    // Draw agents
    if (agent_positions && n_agents > 0) {
        for (int a = 0; a < n_agents; ++a) {
            real_t x = agent_positions[a].x;
            real_t y = agent_positions[a].y;
            
            // Convert to grid coordinates
            int j = (int)((x - x_min_) / (x_max_ - x_min_) * (width - 1));
            int i = (int)((y_max_ - y) / (y_max_ - y_min_) * (height - 1));
            
            if (i >= 0 && i < height && j >= 0 && j < width) {
                grid[i][j] = '0' + a;  // Agent number
            }
        }
    }
    
    // Print grid
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            printf("%c", grid[i][j]);
        }
        printf("\n");
    }
    
    printf("\nLegend: . = free space, # = obstacle, 0-9 = agents\n");
}

returnValue Environment::saveToFile(const char* filename) const
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        printf("ERROR: Failed to open file %s for writing\n", filename);
        return RET_UNKNOWN_BUG;
    }
    
    // Write header
    file << "# Environment Configuration\n";
    file << "# Bounds: [" << x_min_ << ", " << x_max_ << "] x [" << y_min_ << ", " << y_max_ << "]\n";
    file << "# Grid resolution: " << grid_resolution_ << "\n";
    file << "# Obstacles: " << obstacles_.size() << "\n\n";
    
    // Write obstacles
    file << "# Obstacle format: x_center, y_center, width, height, theta\n";
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        const RectangularObstacle& obs = obstacles_[i];
        file << obs.x_center << ", " << obs.y_center << ", "
             << obs.width << ", " << obs.height << ", " << obs.theta << "\n";
    }
    
    file.close();
    return SUCCESSFUL_RETURN;
}

END_NAMESPACE_QPOASES
