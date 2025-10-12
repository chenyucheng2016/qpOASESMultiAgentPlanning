/*
 *  This file is part of qpOASES.
 *
 *  RRTStarPlanner: RRT* path planning using OMPL (or custom implementation)
 *
 *  Author: Multi-Agent MPC Team
 *  Date: October 2025
 */

#ifndef QPOASES_RRTSTARPLANNER_HPP
#define QPOASES_RRTSTARPLANNER_HPP

#include <qpOASES/Environment.hpp>
#include <qpOASES/RRTPath.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

/**
 * @brief Simple RRT* implementation for 2D path planning
 * 
 * This is a lightweight RRT* implementation that doesn't require OMPL.
 * It uses our Environment class for collision checking.
 * 
 * Features:
 * - RRT* algorithm with rewiring
 * - 2D state space (x, y)
 * - Collision checking via Environment
 * - Path smoothing
 */
class RRTStarPlanner {
public:
    /**
     * @brief Constructor
     * @param env Environment reference
     * @param r_agent Agent radius for collision checking
     */
    RRTStarPlanner(const Environment& env, real_t r_agent = 0.5);
    
    /**
     * @brief Destructor
     */
    ~RRTStarPlanner();
    
    /**
     * @brief Plan a path from start to goal
     * @param start Start position
     * @param goal Goal position
     * @param max_iterations Maximum RRT* iterations (default: 5000)
     * @param goal_bias Probability of sampling goal (default: 0.1)
     * @return Path as vector of points (empty if no path found)
     */
    std::vector<Point2D> plan(Point2D start, Point2D goal, 
                              int max_iterations = 5000, 
                              real_t goal_bias = 0.1);
    
    /**
     * @brief Plan a path with temporal information
     * @param start Start position
     * @param goal Goal position
     * @param max_velocity Maximum velocity (m/s, default: 5.0)
     * @param max_acceleration Maximum acceleration (m/s², default: 10.0)
     * @param max_iterations Maximum RRT* iterations (default: 5000)
     * @param goal_bias Probability of sampling goal (default: 0.1)
     * @return RRTPath with temporal information (empty if no path found)
     */
    RRTPath planWithTiming(Point2D start, Point2D goal,
                           real_t max_velocity = 5.0,
                           real_t max_acceleration = 10.0,
                           int max_iterations = 5000,
                           real_t goal_bias = 0.1);
    
    /**
     * @brief Set step size for RRT* expansion
     * @param step_size Step size in meters (default: 0.5)
     */
    void setStepSize(real_t step_size) { step_size_ = step_size; }
    
    /**
     * @brief Set rewiring radius for RRT*
     * @param rewire_radius Rewiring radius in meters (default: 2.0)
     */
    void setRewireRadius(real_t rewire_radius) { rewire_radius_ = rewire_radius; }
    
    /**
     * @brief Set goal tolerance
     * @param goal_tolerance Distance to goal for success (default: 0.5)
     */
    void setGoalTolerance(real_t goal_tolerance) { goal_tolerance_ = goal_tolerance; }
    
    /**
     * @brief Get number of nodes in tree
     */
    int getNumNodes() const { return (int)nodes_.size(); }
    
    /**
     * @brief Print planning statistics
     */
    void printStatistics() const;
    
private:
    /**
     * @brief RRT* tree node
     */
    struct Node {
        Point2D pos;
        int parent_id;
        real_t cost;  // Cost from start
        
        Node(Point2D p, int parent, real_t c) 
            : pos(p), parent_id(parent), cost(c) {}
    };
    
    // Environment
    const Environment& env_;
    real_t r_agent_;
    
    // RRT* parameters
    real_t step_size_;
    real_t rewire_radius_;
    real_t goal_tolerance_;
    
    // Tree
    std::vector<Node> nodes_;
    
    // Statistics
    int iterations_used_;
    bool path_found_;
    
    /**
     * @brief Sample a random point in the environment
     * @param goal Goal position (for goal biasing)
     * @param goal_bias Probability of sampling goal
     * @return Sampled point
     */
    Point2D samplePoint(Point2D goal, real_t goal_bias);
    
    /**
     * @brief Find nearest node to a point
     * @param point Query point
     * @return Index of nearest node
     */
    int findNearest(Point2D point);
    
    /**
     * @brief Steer from one point toward another
     * @param from Start point
     * @param to Target point
     * @return Steered point (at most step_size away from 'from')
     */
    Point2D steer(Point2D from, Point2D to);
    
    /**
     * @brief Check if path between two points is collision-free
     * @param from Start point
     * @param to End point
     * @return true if collision-free, false otherwise
     */
    bool isPathCollisionFree(Point2D from, Point2D to);
    
    /**
     * @brief Find nodes within radius of a point
     * @param point Query point
     * @param radius Search radius
     * @return Indices of nodes within radius
     */
    std::vector<int> findNear(Point2D point, real_t radius);
    
    /**
     * @brief Compute distance between two points
     */
    real_t distance(Point2D a, Point2D b) const;
    
    /**
     * @brief Extract path from tree
     * @param goal_node_id Index of goal node
     * @return Path from start to goal
     */
    std::vector<Point2D> extractPath(int goal_node_id);
    
    /**
     * @brief Smooth path using shortcutting
     * @param path Input path
     * @return Smoothed path
     */
    std::vector<Point2D> smoothPath(const std::vector<Point2D>& path);
};

END_NAMESPACE_QPOASES

#endif  // QPOASES_RRTSTARPLANNER_HPP
