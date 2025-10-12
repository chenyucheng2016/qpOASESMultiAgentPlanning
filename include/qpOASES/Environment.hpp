/*
 *  This file is part of qpOASES.
 *
 *  Environment: Grid map and obstacle management for multi-agent planning
 *
 *  Author: Multi-Agent MPC Team
 *  Date: October 2025
 */

#ifndef QPOASES_ENVIRONMENT_HPP
#define QPOASES_ENVIRONMENT_HPP

#include <qpOASES/TurboADMM.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

/**
 * @brief 2D point structure
 */
struct Point2D {
    real_t x;
    real_t y;
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(real_t x_, real_t y_) : x(x_), y(y_) {}
};

/**
 * @brief Environment class for multi-agent planning
 * 
 * Manages:
 * - Workspace bounds (grid map)
 * - Static rectangular obstacles
 * - Collision checking
 * - Visualization
 */
class Environment {
public:
    /**
     * @brief Constructor
     * @param x_min Minimum x coordinate
     * @param x_max Maximum x coordinate
     * @param y_min Minimum y coordinate
     * @param y_max Maximum y coordinate
     * @param grid_resolution Grid cell size (default: 0.1m)
     */
    Environment(real_t x_min, real_t x_max, real_t y_min, real_t y_max, real_t grid_resolution = 0.1);
    
    /**
     * @brief Destructor
     */
    ~Environment();
    
    /**
     * @brief Add a rectangular obstacle to the environment
     * @param obstacle Obstacle to add
     * @return Obstacle ID (index in obstacle list)
     */
    int addObstacle(const RectangularObstacle& obstacle);
    
    /**
     * @brief Check if a point is collision-free
     * @param x X coordinate
     * @param y Y coordinate
     * @param r_agent Agent radius (safety margin)
     * @return true if collision-free, false otherwise
     */
    bool isCollisionFree(real_t x, real_t y, real_t r_agent = 0.5) const;
    
    /**
     * @brief Check if a point is within workspace bounds
     * @param x X coordinate
     * @param y Y coordinate
     * @return true if within bounds, false otherwise
     */
    bool isInBounds(real_t x, real_t y) const;
    
    /**
     * @brief Get all obstacles
     * @return Vector of obstacles
     */
    const std::vector<RectangularObstacle>& getObstacles() const { return obstacles_; }
    
    /**
     * @brief Get workspace bounds
     */
    void getBounds(real_t& x_min, real_t& x_max, real_t& y_min, real_t& y_max) const;
    
    /**
     * @brief Print ASCII visualization of environment
     * @param agent_positions Optional agent positions to display
     * @param n_agents Number of agents
     */
    void printASCII(const Point2D* agent_positions = 0, int n_agents = 0) const;
    
    /**
     * @brief Save environment to file (CSV format)
     * @param filename Output filename
     */
    returnValue saveToFile(const char* filename) const;
    
    /**
     * @brief Get grid resolution
     */
    real_t getGridResolution() const { return grid_resolution_; }
    
private:
    // Workspace bounds
    real_t x_min_;
    real_t x_max_;
    real_t y_min_;
    real_t y_max_;
    
    // Grid resolution
    real_t grid_resolution_;
    
    // Obstacles
    std::vector<RectangularObstacle> obstacles_;
    
    /**
     * @brief Helper: Check if point is inside a rectangular obstacle
     * @param x X coordinate
     * @param y Y coordinate
     * @param obstacle Obstacle to check
     * @param r_agent Agent radius
     * @return true if inside obstacle (collision), false otherwise
     */
    bool isInsideObstacle(real_t x, real_t y, const RectangularObstacle& obstacle, real_t r_agent) const;
};

END_NAMESPACE_QPOASES

#endif  // QPOASES_ENVIRONMENT_HPP
