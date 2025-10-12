/*
 * Minimal visualization test - just show RRT* path
 */

#include <qpOASES/Environment.hpp>
#include <qpOASES/RRTStarPlanner.hpp>
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

USING_NAMESPACE_QPOASES

void visualizeRRTPath(const RRTPath& rrt_path,
                     const RectangularObstacle& obstacle,
                     Point2D start, Point2D goal)
{
    printf("\n");
    printf("================================================================================\n");
    printf("                         RRT* PATH VISUALIZATION\n");
    printf("================================================================================\n\n");
    
    const int width = 80;
    const int height = 40;
    
    real_t x_min = 0.0;
    real_t x_max = 30.0;
    real_t y_min = 0.0;
    real_t y_max = 20.0;
    
    // Create grid
    char grid[height][width];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            grid[i][j] = '.';
        }
    }
    
    // Draw obstacle
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            real_t x = x_min + (x_max - x_min) * j / (width - 1);
            real_t y = y_max - (y_max - y_min) * i / (height - 1);
            
            real_t dx = x - obstacle.x_center;
            real_t dy = y - obstacle.y_center;
            real_t cos_theta = cos(-obstacle.theta);
            real_t sin_theta = sin(-obstacle.theta);
            real_t local_x = cos_theta * dx - sin_theta * dy;
            real_t local_y = sin_theta * dx + cos_theta * dy;
            
            if (fabs(local_x) < obstacle.width / 2.0 && fabs(local_y) < obstacle.height / 2.0) {
                grid[i][j] = '#';
            }
        }
    }
    
    // Draw RRT* path
    const std::vector<TimedWaypoint>& waypoints = rrt_path.getTimedWaypoints();
    
    for (size_t k = 0; k < waypoints.size(); ++k) {
        real_t px = waypoints[k].position.x;
        real_t py = waypoints[k].position.y;
        
        int j = (int)((px - x_min) / (x_max - x_min) * (width - 1));
        int i = (int)((y_max - py) / (y_max - y_min) * (height - 1));
        
        if (i >= 0 && i < height && j >= 0 && j < width) {
            if (k == 0) {
                grid[i][j] = 'S';
            } else if (k == waypoints.size() - 1) {
                grid[i][j] = 'G';
            } else if (grid[i][j] != '#') {
                grid[i][j] = '*';
            }
        }
    }
    
    // Interpolate between waypoints
    for (size_t k = 0; k < waypoints.size() - 1; ++k) {
        real_t x1 = waypoints[k].position.x;
        real_t y1 = waypoints[k].position.y;
        real_t x2 = waypoints[k+1].position.x;
        real_t y2 = waypoints[k+1].position.y;
        
        int steps = 30;
        for (int s = 1; s < steps; ++s) {
            real_t alpha = (real_t)s / steps;
            real_t px = x1 + alpha * (x2 - x1);
            real_t py = y1 + alpha * (y2 - y1);
            
            int j = (int)((px - x_min) / (x_max - x_min) * (width - 1));
            int i = (int)((y_max - py) / (y_max - y_min) * (height - 1));
            
            if (i >= 0 && i < height && j >= 0 && j < width && grid[i][j] == '.') {
                grid[i][j] = '-';
            }
        }
    }
    
    // Print grid with axes
    printf("  ");
    for (int j = 0; j < width; ++j) {
        if (j % 10 == 0) printf("|");
        else printf("-");
    }
    printf("\n");
    
    for (int i = 0; i < height; ++i) {
        if (i % 5 == 0) printf("%2d", (int)(y_max - i * (y_max - y_min) / (height - 1)));
        else printf("  ");
        
        for (int j = 0; j < width; ++j) {
            printf("%c", grid[i][j]);
        }
        printf("\n");
    }
    
    printf("  ");
    for (int j = 0; j < width; ++j) {
        if (j % 10 == 0) printf("|");
        else printf("-");
    }
    printf("\n");
    printf("  0         10        20        30        40        50        60        70\n");
    
    printf("\n");
    printf("Legend:\n");
    printf("  .  = Free space\n");
    printf("  #  = Obstacle (tilted %.0f°)\n", obstacle.theta * 180.0 / M_PI);
    printf("  -  = RRT* path\n");
    printf("  *  = RRT* waypoint\n");
    printf("  S  = Start\n");
    printf("  G  = Goal\n");
    printf("\n");
    
    printf("Path Statistics:\n");
    printf("  Waypoints:     %zu\n", waypoints.size());
    printf("  Path length:   %.2f m\n", rrt_path.getPathLength());
    printf("  Duration:      %.2f s\n", rrt_path.getDuration());
    printf("  Avg velocity:  %.2f m/s\n", rrt_path.getPathLength() / rrt_path.getDuration());
    
    printf("\n");
    printf("================================================================================\n\n");
}

int main()
{
    printf("=== RRT* Path Visualization Test ===\n\n");
    
    // Setup environment
    printf("Creating environment...\n");
    Environment env(0.0, 30.0, 0.0, 20.0, 0.1);
    RectangularObstacle obstacle(15.0, 10.0, 8.0, 4.0, M_PI / 6.0);  // 30° tilt
    env.addObstacle(obstacle);
    printf("  Obstacle: center=(15.0, 10.0), size=8.0x4.0, angle=30°\n\n");
    
    // Setup start and goal
    Point2D start(2.0, 5.0);
    Point2D goal(28.0, 15.0);
    printf("Planning path from (%.1f, %.1f) to (%.1f, %.1f)...\n", 
           start.x, start.y, goal.x, goal.y);
    
    // Run RRT*
    RRTStarPlanner planner(env, 0.5);
    planner.setStepSize(0.5);
    planner.setRewireRadius(3.0);
    
    RRTPath rrt_path = planner.planWithTiming(start, goal, 5.0, 10.0, 5000);
    
    if (rrt_path.getNumWaypoints() == 0) {
        printf("ERROR: RRT* failed to find path!\n");
        return 1;
    }
    
    printf("  SUCCESS: Path found!\n");
    
    // Visualize
    visualizeRRTPath(rrt_path, obstacle, start, goal);
    
    printf("=== Test Complete ===\n");
    return 0;
}
