/*
 *  This file is part of qpOASES.
 *
 *  BypassScheduleGenerator: Generate time-varying obstacle bypass schedules from RRT* paths
 *
 *  Implementation
 */

#include <qpOASES/BypassScheduleGenerator.hpp>
#include <cmath>
#include <cstdio>

BEGIN_NAMESPACE_QPOASES

BypassScheduleGenerator::BypassScheduleGenerator()
    : safety_margin_(0.5)
{
}

BypassScheduleGenerator::~BypassScheduleGenerator()
{
}

bool BypassScheduleGenerator::generateSchedule(const RRTPath& rrt_path,
                                                const RectangularObstacle& obstacle,
                                                int N,
                                                real_t dt,
                                                BypassSide* schedule_out)
{
    if (schedule_out == nullptr) {
        printf("ERROR: schedule_out is null\n");
        return false;
    }
    
    if (rrt_path.getNumWaypoints() == 0) {
        printf("ERROR: RRT* path is empty\n");
        return false;
    }
    
    if (N < 0) {
        printf("ERROR: Invalid horizon N=%d\n", N);
        return false;
    }
    
    // For each stage k=0..N
    for (int k = 0; k <= N; ++k) {
        // Compute time at this stage
        real_t t = k * dt;
        
        // Clamp to path duration
        if (t > rrt_path.getDuration()) {
            t = rrt_path.getDuration();
        }
        
        // Interpolate position from RRT* path
        Point2D position = rrt_path.interpolatePosition(t);
        
        // Determine bypass side at this position
        schedule_out[k] = determineBypassSide(position, obstacle);
    }
    
    return true;
}

bool BypassScheduleGenerator::generateSchedules(const RRTPath& rrt_path,
                                                 const std::vector<RectangularObstacle>& obstacles,
                                                 int N,
                                                 real_t dt,
                                                 std::vector<BypassSide*>& schedules_out)
{
    if (schedules_out.size() != obstacles.size()) {
        printf("ERROR: schedules_out size (%d) != obstacles size (%d)\n",
               (int)schedules_out.size(), (int)obstacles.size());
        return false;
    }
    
    // Generate schedule for each obstacle
    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (!generateSchedule(rrt_path, obstacles[i], N, dt, schedules_out[i])) {
            printf("ERROR: Failed to generate schedule for obstacle %d\n", (int)i);
            return false;
        }
    }
    
    return true;
}

BypassSide BypassScheduleGenerator::determineBypassSide(Point2D position,
                                                         const RectangularObstacle& obstacle) const
{
    // Transform position to obstacle's local frame
    Point2D local_pos = transformToLocal(position, obstacle);
    
    // Get obstacle dimensions (half-widths)
    real_t half_width = obstacle.width / 2.0;
    real_t half_height = obstacle.height / 2.0;
    
    // Add safety margin
    real_t margin = safety_margin_;
    
    // Determine which side the position is on
    // Use the dimension with the largest distance to determine primary side
    
    real_t dist_left = -local_pos.x - half_width;
    real_t dist_right = local_pos.x - half_width;
    real_t dist_below = -local_pos.y - half_height;
    real_t dist_above = local_pos.y - half_height;
    
    // Find the side with maximum clearance
    real_t max_clearance = dist_left;
    BypassSide side = BYPASS_LEFT;
    
    if (dist_right > max_clearance) {
        max_clearance = dist_right;
        side = BYPASS_RIGHT;
    }
    
    if (dist_below > max_clearance) {
        max_clearance = dist_below;
        side = BYPASS_BELOW;
    }
    
    if (dist_above > max_clearance) {
        max_clearance = dist_above;
        side = BYPASS_ABOVE;
    }
    
    return side;
}

Point2D BypassScheduleGenerator::transformToLocal(Point2D point,
                                                   const RectangularObstacle& obstacle) const
{
    // Translate to obstacle center
    real_t dx = point.x - obstacle.x_center;
    real_t dy = point.y - obstacle.y_center;
    
    // Rotate by -theta (inverse rotation)
    real_t cos_theta = cos(-obstacle.theta);
    real_t sin_theta = sin(-obstacle.theta);
    
    real_t local_x = cos_theta * dx - sin_theta * dy;
    real_t local_y = sin_theta * dx + cos_theta * dy;
    
    return Point2D(local_x, local_y);
}

bool BypassScheduleGenerator::isInsideObstacle(Point2D position,
                                                const RectangularObstacle& obstacle,
                                                real_t margin) const
{
    // Transform to local frame
    Point2D local_pos = transformToLocal(position, obstacle);
    
    // Check if inside (with margin)
    real_t half_width = obstacle.width / 2.0 + margin;
    real_t half_height = obstacle.height / 2.0 + margin;
    
    return (fabs(local_pos.x) < half_width && fabs(local_pos.y) < half_height);
}

void BypassScheduleGenerator::printSchedule(const BypassSide* schedule, int N, int obstacle_id)
{
    printf("\n=== Bypass Schedule (Obstacle %d) ===\n", obstacle_id);
    printf("Stage |  Time (s) | Bypass Side\n");
    printf("------+-----------+-------------\n");
    
    for (int k = 0; k <= N; ++k) {
        const char* side_str = "UNKNOWN";
        switch (schedule[k]) {
            case BYPASS_LEFT:  side_str = "LEFT";  break;
            case BYPASS_RIGHT: side_str = "RIGHT"; break;
            case BYPASS_ABOVE: side_str = "ABOVE"; break;
            case BYPASS_BELOW: side_str = "BELOW"; break;
        }
        
        printf("  %3d | %9.2f | %s\n", k, k * 0.2, side_str);  // Assuming dt=0.2
    }
}

END_NAMESPACE_QPOASES
