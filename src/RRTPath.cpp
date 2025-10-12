/*
 *  This file is part of qpOASES.
 *
 *  RRTPath: Enhanced path structure with temporal information
 *
 *  Implementation
 */

#include <qpOASES/RRTPath.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>

BEGIN_NAMESPACE_QPOASES

RRTPath::RRTPath()
    : path_length_(0.0), duration_(0.0)
{
}

RRTPath::RRTPath(const std::vector<Point2D>& waypoints, 
                 real_t max_velocity, 
                 real_t max_acceleration)
    : path_length_(0.0), duration_(0.0)
{
    setWaypoints(waypoints, max_velocity, max_acceleration);
}

void RRTPath::setWaypoints(const std::vector<Point2D>& waypoints,
                           real_t max_velocity,
                           real_t max_acceleration)
{
    if (waypoints.empty()) {
        timed_waypoints_.clear();
        path_length_ = 0.0;
        duration_ = 0.0;
        return;
    }
    
    // Compute velocity profile
    computeVelocityProfile(waypoints, max_velocity, max_acceleration);
}

void RRTPath::computeVelocityProfile(const std::vector<Point2D>& waypoints,
                                     real_t max_velocity,
                                     real_t max_acceleration)
{
    timed_waypoints_.clear();
    
    if (waypoints.empty()) {
        return;
    }
    
    if (waypoints.size() == 1) {
        // Single waypoint - stationary
        timed_waypoints_.push_back(TimedWaypoint(waypoints[0], 0.0, 0.0));
        path_length_ = 0.0;
        duration_ = 0.0;
        return;
    }
    
    // Compute segment lengths
    std::vector<real_t> segment_lengths;
    path_length_ = 0.0;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        real_t dx = waypoints[i+1].x - waypoints[i].x;
        real_t dy = waypoints[i+1].y - waypoints[i].y;
        real_t length = sqrt(dx * dx + dy * dy);
        segment_lengths.push_back(length);
        path_length_ += length;
    }
    
    // Trapezoidal velocity profile
    // Phase 1: Acceleration from 0 to v_max
    // Phase 2: Cruise at v_max (if path is long enough)
    // Phase 3: Deceleration from v_max to 0
    
    // Time to accelerate from 0 to v_max
    real_t t_accel = max_velocity / max_acceleration;
    
    // Distance covered during acceleration
    real_t d_accel = 0.5 * max_acceleration * t_accel * t_accel;
    
    // Distance covered during deceleration (same as acceleration)
    real_t d_decel = d_accel;
    
    // Check if we can reach max velocity
    if (d_accel + d_decel > path_length_) {
        // Path is too short - triangular profile (no cruise phase)
        // Accelerate to v_peak, then immediately decelerate
        real_t v_peak = sqrt(max_acceleration * path_length_);
        real_t t_accel_short = v_peak / max_acceleration;
        real_t t_decel_short = t_accel_short;
        
        // Assign velocities along path
        real_t current_distance = 0.0;
        real_t current_time = 0.0;
        
        // First waypoint (v=0)
        timed_waypoints_.push_back(TimedWaypoint(waypoints[0], 0.0, 0.0));
        
        for (size_t i = 0; i < segment_lengths.size(); ++i) {
            real_t segment_length = segment_lengths[i];
            real_t next_distance = current_distance + segment_length;
            
            // Determine velocity at end of segment
            real_t velocity_end;
            if (next_distance <= path_length_ / 2.0) {
                // Acceleration phase
                real_t d_from_start = next_distance;
                velocity_end = sqrt(2.0 * max_acceleration * d_from_start);
                velocity_end = std::min(velocity_end, v_peak);
            } else {
                // Deceleration phase
                real_t d_from_end = path_length_ - next_distance;
                velocity_end = sqrt(2.0 * max_acceleration * d_from_end);
                velocity_end = std::min(velocity_end, v_peak);
            }
            
            // Compute segment time based on kinematics
            real_t velocity_start = (i == 0) ? 0.0 : timed_waypoints_.back().velocity;
            
            // Time for this segment using kinematic equations
            real_t segment_time;
            real_t avg_velocity = (velocity_start + velocity_end) / 2.0;
            
            if (avg_velocity < 1e-6) {
                // Both velocities near zero - use acceleration from rest
                // t = sqrt(2*d/a)
                segment_time = sqrt(2.0 * segment_length / max_acceleration);
            } else {
                // Use average velocity: d = v_avg * t
                segment_time = segment_length / avg_velocity;
            }
            
            current_time += segment_time;
            
            timed_waypoints_.push_back(TimedWaypoint(waypoints[i+1], current_time, velocity_end));
            current_distance = next_distance;
        }
        
        // Last waypoint has zero velocity
        if (!timed_waypoints_.empty()) {
            timed_waypoints_.back().velocity = 0.0;
        }
        
        duration_ = current_time;
    } else {
        // Trapezoidal profile (with cruise phase)
        real_t d_cruise = path_length_ - d_accel - d_decel;
        real_t t_cruise = d_cruise / max_velocity;
        
        // Assign velocities along path
        real_t current_distance = 0.0;
        real_t current_time = 0.0;
        
        // First waypoint (v=0)
        timed_waypoints_.push_back(TimedWaypoint(waypoints[0], 0.0, 0.0));
        
        for (size_t i = 0; i < segment_lengths.size(); ++i) {
            real_t segment_length = segment_lengths[i];
            real_t next_distance = current_distance + segment_length;
            
            // Determine velocity at end of segment
            real_t velocity_end;
            if (next_distance <= d_accel) {
                // Acceleration phase
                velocity_end = sqrt(2.0 * max_acceleration * next_distance);
                velocity_end = std::min(velocity_end, max_velocity);
            } else if (next_distance <= d_accel + d_cruise) {
                // Cruise phase
                velocity_end = max_velocity;
            } else {
                // Deceleration phase
                real_t d_from_end = path_length_ - next_distance;
                velocity_end = sqrt(2.0 * max_acceleration * d_from_end);
                velocity_end = std::min(velocity_end, max_velocity);
            }
            
            // Compute segment time based on kinematics
            real_t velocity_start = (i == 0) ? 0.0 : timed_waypoints_.back().velocity;
            
            // Time for this segment using kinematic equations
            real_t segment_time;
            real_t avg_velocity = (velocity_start + velocity_end) / 2.0;
            
            if (avg_velocity < 1e-6) {
                // Both velocities near zero - use acceleration from rest
                // t = sqrt(2*d/a)
                segment_time = sqrt(2.0 * segment_length / max_acceleration);
            } else {
                // Use average velocity: d = v_avg * t
                segment_time = segment_length / avg_velocity;
            }
            
            current_time += segment_time;
            
            timed_waypoints_.push_back(TimedWaypoint(waypoints[i+1], current_time, velocity_end));
            current_distance = next_distance;
        }
        
        // Last waypoint has zero velocity
        if (!timed_waypoints_.empty()) {
            timed_waypoints_.back().velocity = 0.0;
        }
        
        duration_ = current_time;
    }
}

std::vector<Point2D> RRTPath::getSpatialWaypoints() const
{
    std::vector<Point2D> waypoints;
    for (size_t i = 0; i < timed_waypoints_.size(); ++i) {
        waypoints.push_back(timed_waypoints_[i].position);
    }
    return waypoints;
}

Point2D RRTPath::interpolatePosition(real_t t) const
{
    if (timed_waypoints_.empty()) {
        return Point2D(0.0, 0.0);
    }
    
    // Clamp to valid range
    if (t <= 0.0) {
        return timed_waypoints_[0].position;
    }
    if (t >= duration_) {
        return timed_waypoints_.back().position;
    }
    
    // Find segment
    int seg = findSegment(t);
    if (seg < 0 || seg >= (int)timed_waypoints_.size() - 1) {
        return timed_waypoints_.back().position;
    }
    
    // Linear interpolation
    const TimedWaypoint& wp1 = timed_waypoints_[seg];
    const TimedWaypoint& wp2 = timed_waypoints_[seg + 1];
    
    real_t dt = wp2.time - wp1.time;
    if (dt < 1e-6) {
        return wp1.position;
    }
    
    real_t ratio = (t - wp1.time) / dt;
    
    real_t x = wp1.position.x + ratio * (wp2.position.x - wp1.position.x);
    real_t y = wp1.position.y + ratio * (wp2.position.y - wp1.position.y);
    
    return Point2D(x, y);
}

real_t RRTPath::interpolateVelocity(real_t t) const
{
    if (timed_waypoints_.empty()) {
        return 0.0;
    }
    
    // Clamp to valid range
    if (t <= 0.0) {
        return timed_waypoints_[0].velocity;
    }
    if (t >= duration_) {
        return timed_waypoints_.back().velocity;
    }
    
    // Find segment
    int seg = findSegment(t);
    if (seg < 0 || seg >= (int)timed_waypoints_.size() - 1) {
        return timed_waypoints_.back().velocity;
    }
    
    // Linear interpolation
    const TimedWaypoint& wp1 = timed_waypoints_[seg];
    const TimedWaypoint& wp2 = timed_waypoints_[seg + 1];
    
    real_t dt = wp2.time - wp1.time;
    if (dt < 1e-6) {
        return wp1.velocity;
    }
    
    real_t ratio = (t - wp1.time) / dt;
    
    return wp1.velocity + ratio * (wp2.velocity - wp1.velocity);
}

int RRTPath::findSegment(real_t t) const
{
    for (int i = 0; i < (int)timed_waypoints_.size() - 1; ++i) {
        if (t >= timed_waypoints_[i].time && t <= timed_waypoints_[i+1].time) {
            return i;
        }
    }
    return -1;
}

bool RRTPath::verifySafetyMargin(const Environment& env, real_t r_agent, real_t dt) const
{
    if (timed_waypoints_.empty()) {
        return false;
    }
    
    // Check at regular time intervals
    int num_checks = (int)(duration_ / dt) + 1;
    
    for (int i = 0; i <= num_checks; ++i) {
        real_t t = i * dt;
        if (t > duration_) {
            t = duration_;
        }
        
        Point2D pos = interpolatePosition(t);
        
        if (!env.isInBounds(pos.x, pos.y)) {
            printf("WARNING: Path goes out of bounds at t=%.2f: (%.2f, %.2f)\n", 
                   t, pos.x, pos.y);
            return false;
        }
        
        if (!env.isCollisionFree(pos.x, pos.y, r_agent)) {
            printf("WARNING: Path violates safety margin at t=%.2f: (%.2f, %.2f)\n",
                   t, pos.x, pos.y);
            return false;
        }
    }
    
    return true;
}

void RRTPath::printSummary() const
{
    printf("\n=== RRT* Path Summary ===\n");
    printf("Waypoints: %d\n", (int)timed_waypoints_.size());
    printf("Path length: %.2f m\n", path_length_);
    printf("Duration: %.2f s\n", duration_);
    
    if (!timed_waypoints_.empty()) {
        printf("Average velocity: %.2f m/s\n", path_length_ / duration_);
        printf("\nWaypoint Details:\n");
        printf("  ID |    Time (s) |  Position (x, y)  | Velocity (m/s)\n");
        printf("-----+-------------+-------------------+---------------\n");
        
        for (size_t i = 0; i < timed_waypoints_.size(); ++i) {
            const TimedWaypoint& wp = timed_waypoints_[i];
            
            // Compute velocity via numerical differentiation
            real_t computed_vel = wp.velocity;
            if (i > 0) {
                const TimedWaypoint& wp_prev = timed_waypoints_[i-1];
                real_t dx = wp.position.x - wp_prev.position.x;
                real_t dy = wp.position.y - wp_prev.position.y;
                real_t dist = sqrt(dx * dx + dy * dy);
                real_t dt = wp.time - wp_prev.time;
                if (dt > 1e-6) {
                    computed_vel = dist / dt;
                }
            }
            
            printf("  %2d | %11.3f | (%7.2f, %7.2f) | %13.2f\n",
                   (int)i, wp.time, wp.position.x, wp.position.y, computed_vel);
        }
    }
}

RRTPath RRTPath::resample(real_t dt) const
{
    if (timed_waypoints_.empty() || duration_ < 1e-6) {
        return RRTPath();
    }
    
    // Create new path with resampled waypoints
    std::vector<Point2D> resampled_waypoints;
    std::vector<real_t> resampled_times;
    
    // Sample at regular time intervals
    int n_samples = (int)(duration_ / dt);
    
    for (int i = 0; i <= n_samples; ++i) {
        real_t t = i * dt;
        if (t > duration_) break;
        
        Point2D pos = interpolatePosition(t);
        resampled_waypoints.push_back(pos);
        resampled_times.push_back(t);
    }
    
    // Always add the final waypoint at exactly duration_ with zero velocity
    if (resampled_times.empty() || resampled_times.back() < duration_ - 1e-6) {
        Point2D final_pos = interpolatePosition(duration_);
        resampled_waypoints.push_back(final_pos);
        resampled_times.push_back(duration_);
    }
    
    // Create new RRTPath
    RRTPath resampled;
    resampled.timed_waypoints_.clear();
    resampled.path_length_ = path_length_;
    resampled.duration_ = duration_;
    
    // Copy waypoints with velocities computed via numerical differentiation
    for (size_t i = 0; i < resampled_waypoints.size(); ++i) {
        real_t t = resampled_times[i];
        real_t vel = 0.0;
        
        // Compute velocity via numerical differentiation
        if (i > 0 && i < resampled_waypoints.size() - 1) {
            // Use central difference for interior points
            real_t dx = resampled_waypoints[i].x - resampled_waypoints[i-1].x;
            real_t dy = resampled_waypoints[i].y - resampled_waypoints[i-1].y;
            real_t dist = sqrt(dx * dx + dy * dy);
            real_t dt_actual = resampled_times[i] - resampled_times[i-1];
            if (dt_actual > 1e-6) {
                vel = dist / dt_actual;
            }
        } else if (i == resampled_waypoints.size() - 1) {
            // Last waypoint: velocity is zero
            vel = 0.0;
        }
        // First waypoint: velocity is zero (already initialized)
        
        resampled.timed_waypoints_.push_back(
            TimedWaypoint(resampled_waypoints[i], t, vel)
        );
    }
    
    return resampled;
}

END_NAMESPACE_QPOASES
