/*
 *  This file is part of qpOASES.
 *
 *  RRTPath: Enhanced path structure with temporal information
 *
 *  Author: Multi-Agent MPC Team
 *  Date: October 2025
 */

#ifndef QPOASES_RRTPATH_HPP
#define QPOASES_RRTPATH_HPP

#include <qpOASES/Environment.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

/**
 * @brief Waypoint with temporal information
 */
struct TimedWaypoint {
    Point2D position;     // Spatial position (x, y)
    real_t time;          // Time stamp (seconds)
    real_t velocity;      // Velocity at this waypoint (m/s)
    
    TimedWaypoint() : position(), time(0.0), velocity(0.0) {}
    TimedWaypoint(Point2D pos, real_t t, real_t v) 
        : position(pos), time(t), velocity(v) {}
};

/**
 * @brief RRT* path with temporal information
 * 
 * Features:
 * - Spatial waypoints (x, y)
 * - Time stamps for each waypoint
 * - Velocity profile
 * - Path length and duration
 * - Safety margin verification
 */
class RRTPath {
public:
    /**
     * @brief Constructor
     */
    RRTPath();
    
    /**
     * @brief Constructor from spatial waypoints
     * @param waypoints Spatial waypoints (x, y)
     * @param max_velocity Maximum velocity (m/s)
     * @param max_acceleration Maximum acceleration (m/s²)
     */
    RRTPath(const std::vector<Point2D>& waypoints, 
            real_t max_velocity = 5.0, 
            real_t max_acceleration = 10.0);
    
    /**
     * @brief Add temporal information to spatial waypoints
     * @param waypoints Spatial waypoints
     * @param max_velocity Maximum velocity (m/s)
     * @param max_acceleration Maximum acceleration (m/s²)
     */
    void setWaypoints(const std::vector<Point2D>& waypoints,
                      real_t max_velocity = 5.0,
                      real_t max_acceleration = 10.0);
    
    /**
     * @brief Get timed waypoints
     */
    const std::vector<TimedWaypoint>& getTimedWaypoints() const { return timed_waypoints_; }
    
    /**
     * @brief Get spatial waypoints only
     */
    std::vector<Point2D> getSpatialWaypoints() const;
    
    /**
     * @brief Interpolate position at a specific time
     * @param t Time (seconds)
     * @return Interpolated position
     */
    Point2D interpolatePosition(real_t t) const;
    
    /**
     * @brief Interpolate velocity at a specific time
     * @param t Time (seconds)
     * @return Interpolated velocity (m/s)
     */
    real_t interpolateVelocity(real_t t) const;
    
    /**
     * @brief Get total path length (meters)
     */
    real_t getPathLength() const { return path_length_; }
    
    /**
     * @brief Get total duration (seconds)
     */
    real_t getDuration() const { return duration_; }
    
    /**
     * @brief Get number of waypoints
     */
    int getNumWaypoints() const { return (int)timed_waypoints_.size(); }
    
    /**
     * @brief Verify safety margin along path
     * @param env Environment for collision checking
     * @param r_agent Agent radius (safety margin)
     * @param dt Time step for checking (default: 0.1s)
     * @return true if path maintains safety margin, false otherwise
     */
    bool verifySafetyMargin(const Environment& env, real_t r_agent, real_t dt = 0.1) const;
    
    /**
     * @brief Print path summary
     */
    void printSummary() const;
    
    /**
     * @brief Resample path at regular time intervals
     * @param dt Time step for resampling (seconds)
     * @return Resampled path with more waypoints
     */
    RRTPath resample(real_t dt) const;
    
private:
    std::vector<TimedWaypoint> timed_waypoints_;
    real_t path_length_;
    real_t duration_;
    
    /**
     * @brief Compute velocity profile using trapezoidal profile
     * @param waypoints Spatial waypoints
     * @param max_velocity Maximum velocity (m/s)
     * @param max_acceleration Maximum acceleration (m/s²)
     */
    void computeVelocityProfile(const std::vector<Point2D>& waypoints,
                                real_t max_velocity,
                                real_t max_acceleration);
    
    /**
     * @brief Find segment containing time t
     * @param t Time (seconds)
     * @return Segment index (or -1 if out of range)
     */
    int findSegment(real_t t) const;
};

END_NAMESPACE_QPOASES

#endif  // QPOASES_RRTPATH_HPP
