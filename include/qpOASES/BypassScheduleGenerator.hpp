/*
 *  This file is part of qpOASES.
 *
 *  BypassScheduleGenerator: Generate time-varying obstacle bypass schedules from RRT* paths
 *
 *  Author: Multi-Agent MPC Team
 *  Date: October 2025
 */

#ifndef QPOASES_BYPASSSCHEDULEGENERATOR_HPP
#define QPOASES_BYPASSSCHEDULEGENERATOR_HPP

#include <qpOASES/RRTPath.hpp>
#include <qpOASES/TurboADMM.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

/**
 * @brief Bypass Schedule Generator
 * 
 * Analyzes RRT* paths and generates time-varying obstacle bypass schedules
 * for use with MPC/ADMM. For each MPC stage k, determines which side of
 * each obstacle the agent should bypass.
 * 
 * Algorithm:
 * 1. For each stage k=0..N:
 *    - Interpolate position from RRT* path at time t = k*dt
 *    - Transform position to obstacle's local frame
 *    - Determine bypass side (LEFT, RIGHT, ABOVE, BELOW)
 *    - Assign to schedule[k]
 * 
 * The generated schedule is used by addTimeVaryingObstacleConstraint()
 * to create convex constraints at each MPC stage.
 */
class BypassScheduleGenerator {
public:
    /**
     * @brief Constructor
     */
    BypassScheduleGenerator();
    
    /**
     * @brief Destructor
     */
    ~BypassScheduleGenerator();
    
    /**
     * @brief Generate bypass schedule for a single obstacle
     * @param rrt_path RRT* path with temporal information
     * @param obstacle Rectangular obstacle to bypass
     * @param N MPC horizon length
     * @param dt MPC time step (seconds)
     * @param schedule_out Output array of bypass sides (size N+1)
     * @return true if schedule generated successfully, false otherwise
     */
    bool generateSchedule(const RRTPath& rrt_path,
                          const RectangularObstacle& obstacle,
                          int N,
                          real_t dt,
                          BypassSide* schedule_out);
    
    /**
     * @brief Generate bypass schedules for multiple obstacles
     * @param rrt_path RRT* path with temporal information
     * @param obstacles Vector of obstacles
     * @param N MPC horizon length
     * @param dt MPC time step (seconds)
     * @param schedules_out Output vector of schedules (one per obstacle)
     * @return true if all schedules generated successfully, false otherwise
     */
    bool generateSchedules(const RRTPath& rrt_path,
                           const std::vector<RectangularObstacle>& obstacles,
                           int N,
                           real_t dt,
                           std::vector<BypassSide*>& schedules_out);
    
    /**
     * @brief Set safety margin for bypass side determination
     * @param margin Safety margin (meters, default: 0.5)
     */
    void setSafetyMargin(real_t margin) { safety_margin_ = margin; }
    
    /**
     * @brief Get safety margin
     */
    real_t getSafetyMargin() const { return safety_margin_; }
    
    /**
     * @brief Print schedule for debugging
     * @param schedule Bypass schedule array (size N+1)
     * @param N Horizon length
     * @param obstacle_id Obstacle ID for labeling
     */
    static void printSchedule(const BypassSide* schedule, int N, int obstacle_id = 0);
    
private:
    real_t safety_margin_;  ///< Safety margin for bypass determination
    
    /**
     * @brief Determine bypass side at a specific position
     * @param position Agent position (x, y)
     * @param obstacle Rectangular obstacle
     * @return Bypass side (LEFT, RIGHT, ABOVE, BELOW)
     */
    BypassSide determineBypassSide(Point2D position,
                                    const RectangularObstacle& obstacle) const;
    
    /**
     * @brief Transform point to obstacle's local frame
     * @param point Point in global frame
     * @param obstacle Rectangular obstacle
     * @return Point in obstacle's local frame
     */
    Point2D transformToLocal(Point2D point,
                             const RectangularObstacle& obstacle) const;
    
    /**
     * @brief Check if position is inside obstacle (with margin)
     * @param position Agent position
     * @param obstacle Rectangular obstacle
     * @param margin Safety margin
     * @return true if inside (collision), false otherwise
     */
    bool isInsideObstacle(Point2D position,
                          const RectangularObstacle& obstacle,
                          real_t margin) const;
};

END_NAMESPACE_QPOASES

#endif  // QPOASES_BYPASSSCHEDULEGENERATOR_HPP
