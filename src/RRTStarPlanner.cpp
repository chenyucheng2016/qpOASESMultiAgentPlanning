/*
 *  This file is part of qpOASES.
 *
 *  RRTStarPlanner: RRT* path planning implementation
 *
 *  Implementation
 */

#include <qpOASES/RRTStarPlanner.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstdio>
#include <algorithm>

BEGIN_NAMESPACE_QPOASES

RRTStarPlanner::RRTStarPlanner(const Environment& env, real_t r_agent)
    : env_(env), r_agent_(r_agent), 
      step_size_(0.5), rewire_radius_(2.0), goal_tolerance_(0.5),
      iterations_used_(0), path_found_(false)
{
    // Seed random number generator
    srand((unsigned int)time(0));
}

RRTStarPlanner::~RRTStarPlanner()
{
    // Nothing to clean up
}

std::vector<Point2D> RRTStarPlanner::plan(Point2D start, Point2D goal, 
                                          int max_iterations, real_t goal_bias)
{
    // Clear previous tree
    nodes_.clear();
    iterations_used_ = 0;
    path_found_ = false;
    
    // Check if start and goal are valid
    if (!env_.isInBounds(start.x, start.y) || !env_.isCollisionFree(start.x, start.y, r_agent_)) {
        printf("ERROR: Start position (%.2f, %.2f) is invalid!\n", start.x, start.y);
        return std::vector<Point2D>();
    }
    
    if (!env_.isInBounds(goal.x, goal.y) || !env_.isCollisionFree(goal.x, goal.y, r_agent_)) {
        printf("ERROR: Goal position (%.2f, %.2f) is invalid!\n", goal.x, goal.y);
        return std::vector<Point2D>();
    }
    
    // Add start node
    nodes_.push_back(Node(start, -1, 0.0));
    
    int goal_node_id = -1;
    real_t best_goal_cost = 1e10;
    
    // RRT* main loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        iterations_used_ = iter + 1;
        
        // Sample random point (with goal biasing)
        Point2D x_rand = samplePoint(goal, goal_bias);
        
        // Find nearest node
        int nearest_id = findNearest(x_rand);
        Point2D x_nearest = nodes_[nearest_id].pos;
        
        // Steer toward sample
        Point2D x_new = steer(x_nearest, x_rand);
        
        // Check if path is collision-free
        if (!isPathCollisionFree(x_nearest, x_new)) {
            continue;
        }
        
        // Find nearby nodes for rewiring
        std::vector<int> near_ids = findNear(x_new, rewire_radius_);
        
        // Choose best parent (minimum cost)
        int best_parent_id = nearest_id;
        real_t best_cost = nodes_[nearest_id].cost + distance(x_nearest, x_new);
        
        for (size_t i = 0; i < near_ids.size(); ++i) {
            int near_id = near_ids[i];
            Point2D x_near = nodes_[near_id].pos;
            
            real_t cost = nodes_[near_id].cost + distance(x_near, x_new);
            
            if (cost < best_cost && isPathCollisionFree(x_near, x_new)) {
                best_parent_id = near_id;
                best_cost = cost;
            }
        }
        
        // Add new node
        int new_node_id = (int)nodes_.size();
        nodes_.push_back(Node(x_new, best_parent_id, best_cost));
        
        // Rewire nearby nodes
        for (size_t i = 0; i < near_ids.size(); ++i) {
            int near_id = near_ids[i];
            Point2D x_near = nodes_[near_id].pos;
            
            real_t new_cost = best_cost + distance(x_new, x_near);
            
            if (new_cost < nodes_[near_id].cost && isPathCollisionFree(x_new, x_near)) {
                nodes_[near_id].parent_id = new_node_id;
                nodes_[near_id].cost = new_cost;
            }
        }
        
        // Check if we reached the goal
        real_t dist_to_goal = distance(x_new, goal);
        if (dist_to_goal < goal_tolerance_) {
            if (best_cost < best_goal_cost) {
                goal_node_id = new_node_id;
                best_goal_cost = best_cost;
                path_found_ = true;
            }
        }
        
        // Early termination if goal found and enough iterations
        if (path_found_ && iter > max_iterations / 2) {
            break;
        }
    }
    
    // Extract path if found
    if (goal_node_id >= 0) {
        std::vector<Point2D> path = extractPath(goal_node_id);
        
        // Add goal as final point
        path.push_back(goal);
        
        // Smooth path
        path = smoothPath(path);
        
        return path;
    }
    
    printf("WARNING: No path found after %d iterations\n", iterations_used_);
    return std::vector<Point2D>();
}

RRTPath RRTStarPlanner::planWithTiming(Point2D start, Point2D goal,
                                       real_t max_velocity,
                                       real_t max_acceleration,
                                       int max_iterations,
                                       real_t goal_bias)
{
    // First, find spatial path using standard RRT*
    std::vector<Point2D> spatial_path = plan(start, goal, max_iterations, goal_bias);
    
    // If no path found, return empty RRTPath
    if (spatial_path.empty()) {
        return RRTPath();
    }
    
    // Create RRTPath with temporal information
    RRTPath timed_path(spatial_path, max_velocity, max_acceleration);
    
    // Verify safety margin along the timed path
    if (!timed_path.verifySafetyMargin(env_, r_agent_, 0.1)) {
        printf("WARNING: Path violates safety margin!\n");
    }
    
    return timed_path;
}

Point2D RRTStarPlanner::samplePoint(Point2D goal, real_t goal_bias)
{
    // Goal biasing
    real_t r = (real_t)rand() / RAND_MAX;
    if (r < goal_bias) {
        return goal;
    }
    
    // Random sampling in environment bounds
    real_t x_min, x_max, y_min, y_max;
    env_.getBounds(x_min, x_max, y_min, y_max);
    
    real_t x = x_min + (x_max - x_min) * ((real_t)rand() / RAND_MAX);
    real_t y = y_min + (y_max - y_min) * ((real_t)rand() / RAND_MAX);
    
    return Point2D(x, y);
}

int RRTStarPlanner::findNearest(Point2D point)
{
    int nearest_id = 0;
    real_t min_dist = distance(nodes_[0].pos, point);
    
    for (size_t i = 1; i < nodes_.size(); ++i) {
        real_t dist = distance(nodes_[i].pos, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = (int)i;
        }
    }
    
    return nearest_id;
}

Point2D RRTStarPlanner::steer(Point2D from, Point2D to)
{
    real_t dist = distance(from, to);
    
    if (dist <= step_size_) {
        return to;
    }
    
    // Move step_size toward 'to'
    real_t ratio = step_size_ / dist;
    real_t x = from.x + ratio * (to.x - from.x);
    real_t y = from.y + ratio * (to.y - from.y);
    
    return Point2D(x, y);
}

bool RRTStarPlanner::isPathCollisionFree(Point2D from, Point2D to)
{
    // Check collision along path with fine resolution
    real_t dist = distance(from, to);
    int num_checks = (int)(dist / 0.1) + 1;  // Check every 0.1m
    
    for (int i = 0; i <= num_checks; ++i) {
        real_t ratio = (real_t)i / num_checks;
        real_t x = from.x + ratio * (to.x - from.x);
        real_t y = from.y + ratio * (to.y - from.y);
        
        if (!env_.isInBounds(x, y) || !env_.isCollisionFree(x, y, r_agent_)) {
            return false;
        }
    }
    
    return true;
}

std::vector<int> RRTStarPlanner::findNear(Point2D point, real_t radius)
{
    std::vector<int> near_ids;
    
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (distance(nodes_[i].pos, point) < radius) {
            near_ids.push_back((int)i);
        }
    }
    
    return near_ids;
}

real_t RRTStarPlanner::distance(Point2D a, Point2D b) const
{
    real_t dx = a.x - b.x;
    real_t dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

std::vector<Point2D> RRTStarPlanner::extractPath(int goal_node_id)
{
    std::vector<Point2D> path;
    
    int current_id = goal_node_id;
    while (current_id >= 0) {
        path.push_back(nodes_[current_id].pos);
        current_id = nodes_[current_id].parent_id;
    }
    
    // Reverse to get start -> goal order
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<Point2D> RRTStarPlanner::smoothPath(const std::vector<Point2D>& path)
{
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<Point2D> smoothed;
    smoothed.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        // Try to connect to farthest visible point
        size_t farthest = i + 1;
        
        for (size_t j = path.size() - 1; j > i + 1; --j) {
            if (isPathCollisionFree(path[i], path[j])) {
                farthest = j;
                break;
            }
        }
        
        smoothed.push_back(path[farthest]);
        i = farthest;
    }
    
    return smoothed;
}

void RRTStarPlanner::printStatistics() const
{
    printf("\n=== RRT* Planning Statistics ===\n");
    printf("Iterations used: %d\n", iterations_used_);
    printf("Nodes in tree: %d\n", (int)nodes_.size());
    printf("Path found: %s\n", path_found_ ? "YES" : "NO");
    
    if (path_found_) {
        // Find goal node (last node with lowest cost near goal)
        real_t min_cost = 1e10;
        for (size_t i = 0; i < nodes_.size(); ++i) {
            if (nodes_[i].cost < min_cost) {
                min_cost = nodes_[i].cost;
            }
        }
        printf("Path cost: %.2f\n", min_cost);
    }
}

END_NAMESPACE_QPOASES
