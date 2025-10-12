/*
 * Test: Bypass Schedule Generator
 * 
 * Tests the generation of time-varying obstacle bypass schedules from RRT* paths
 */

#include <qpOASES/BypassScheduleGenerator.hpp>
#include <qpOASES/RRTStarPlanner.hpp>
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

USING_NAMESPACE_QPOASES

void visualizeScheduleOnPath(const RRTPath& path, 
                             const RectangularObstacle& obstacle,
                             const BypassSide* schedule,
                             int N,
                             real_t dt)
{
    printf("\n=== Path Visualization with Bypass Schedule ===\n");
    
    // ASCII grid
    const int width = 60;
    const int height = 30;
    
    // Get path bounds
    real_t x_min = obstacle.x_center - obstacle.width;
    real_t x_max = obstacle.x_center + obstacle.width;
    real_t y_min = obstacle.y_center - obstacle.height;
    real_t y_max = obstacle.y_center + obstacle.height;
    
    // Expand to include path
    for (int k = 0; k <= N; ++k) {
        Point2D pos = path.interpolatePosition(k * dt);
        x_min = std::min(x_min, pos.x - 2.0);
        x_max = std::max(x_max, pos.x + 2.0);
        y_min = std::min(y_min, pos.y - 2.0);
        y_max = std::max(y_max, pos.y + 2.0);
    }
    
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
            
            // Check if inside obstacle
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
    
    // Draw path with bypass side indicators
    for (int k = 0; k <= N; ++k) {
        Point2D pos = path.interpolatePosition(k * dt);
        
        int j = (int)((pos.x - x_min) / (x_max - x_min) * (width - 1));
        int i = (int)((y_max - pos.y) / (y_max - y_min) * (height - 1));
        
        if (i >= 0 && i < height && j >= 0 && j < width) {
            if (k == 0) {
                grid[i][j] = 'S';  // Start
            } else if (k == N) {
                grid[i][j] = 'G';  // Goal
            } else {
                // Use different characters for different bypass sides
                switch (schedule[k]) {
                    case BYPASS_LEFT:  grid[i][j] = 'L'; break;
                    case BYPASS_RIGHT: grid[i][j] = 'R'; break;
                    case BYPASS_ABOVE: grid[i][j] = 'A'; break;
                    case BYPASS_BELOW: grid[i][j] = 'B'; break;
                }
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
    
    printf("\nLegend:\n");
    printf("  . = free space\n");
    printf("  # = obstacle\n");
    printf("  S = start, G = goal\n");
    printf("  L = bypass LEFT, R = bypass RIGHT\n");
    printf("  A = bypass ABOVE, B = bypass BELOW\n");
}

int main()
{
    printf("=== Test: Bypass Schedule Generator ===\n\n");
    
    // ========================================
    // Test 1: Single obstacle, path goes around right
    // ========================================
    
    printf("--- Test 1: Path Goes Around Right Side ---\n");
    
    Environment env1(0.0, 30.0, 0.0, 20.0, 0.1);
    RectangularObstacle obs1(15.0, 10.0, 6.0, 4.0, 0.0);
    env1.addObstacle(obs1);
    
    RRTStarPlanner planner1(env1, 0.5);
    planner1.setStepSize(0.5);
    planner1.setRewireRadius(3.0);
    
    Point2D start1(2.0, 10.0);
    Point2D goal1(28.0, 10.0);
    
    printf("Environment: [0, 30] x [0, 20]\n");
    printf("Obstacle: center=(%.1f, %.1f), size=%.1f×%.1f, angle=%.2f\n",
           obs1.x_center, obs1.y_center, obs1.width, obs1.height, obs1.theta);
    printf("Start: (%.1f, %.1f), Goal: (%.1f, %.1f)\n\n",
           start1.x, start1.y, goal1.x, goal1.y);
    
    RRTPath path1 = planner1.planWithTiming(start1, goal1, 5.0, 10.0, 3000);
    
    if (path1.getNumWaypoints() == 0) {
        printf("FAIL: No path found!\n");
        return 1;
    }
    
    printf("Path found: %d waypoints, %.2f m, %.2f s\n",
           path1.getNumWaypoints(), path1.getPathLength(), path1.getDuration());
    
    // Generate bypass schedule
    int N1 = 10;
    real_t dt1 = 0.2;
    BypassSide schedule1[11];  // N+1
    
    BypassScheduleGenerator generator1;
    generator1.setSafetyMargin(0.5);
    
    bool success1 = generator1.generateSchedule(path1, obs1, N1, dt1, schedule1);
    
    if (!success1) {
        printf("FAIL: Failed to generate schedule!\n");
        return 1;
    }
    
    printf("SUCCESS: Schedule generated!\n");
    BypassScheduleGenerator::printSchedule(schedule1, N1, 1);
    
    // Verify schedule consistency
    bool consistent1 = true;
    BypassSide dominant_side1 = schedule1[N1/2];  // Check middle of path
    int side_changes1 = 0;
    
    for (int k = 1; k <= N1; ++k) {
        if (schedule1[k] != schedule1[k-1]) {
            side_changes1++;
        }
    }
    
    printf("\nSchedule Analysis:\n");
    printf("  Dominant side: %s\n", 
           dominant_side1 == BYPASS_LEFT ? "LEFT" :
           dominant_side1 == BYPASS_RIGHT ? "RIGHT" :
           dominant_side1 == BYPASS_ABOVE ? "ABOVE" : "BELOW");
    printf("  Side changes: %d\n", side_changes1);
    printf("  Consistency: %s\n", side_changes1 <= 2 ? "GOOD" : "WARNING: Many changes");
    
    visualizeScheduleOnPath(path1, obs1, schedule1, N1, dt1);
    
    // ========================================
    // Test 2: Obstacle with rotation
    // ========================================
    
    printf("\n--- Test 2: Rotated Obstacle ---\n");
    
    Environment env2(0.0, 30.0, 0.0, 20.0, 0.1);
    RectangularObstacle obs2(15.0, 10.0, 8.0, 3.0, M_PI / 4.0);  // 45 degrees
    env2.addObstacle(obs2);
    
    RRTStarPlanner planner2(env2, 0.5);
    planner2.setStepSize(0.5);
    planner2.setRewireRadius(3.0);
    
    Point2D start2(2.0, 10.0);
    Point2D goal2(28.0, 10.0);
    
    printf("Obstacle: center=(%.1f, %.1f), size=%.1f×%.1f, angle=%.2f rad (45°)\n",
           obs2.x_center, obs2.y_center, obs2.width, obs2.height, obs2.theta);
    
    RRTPath path2 = planner2.planWithTiming(start2, goal2, 5.0, 10.0, 3000);
    
    if (path2.getNumWaypoints() == 0) {
        printf("FAIL: No path found!\n");
        return 1;
    }
    
    printf("Path found: %d waypoints, %.2f m, %.2f s\n",
           path2.getNumWaypoints(), path2.getPathLength(), path2.getDuration());
    
    int N2 = 10;
    real_t dt2 = 0.2;
    BypassSide schedule2[11];
    
    BypassScheduleGenerator generator2;
    bool success2 = generator2.generateSchedule(path2, obs2, N2, dt2, schedule2);
    
    if (!success2) {
        printf("FAIL: Failed to generate schedule!\n");
        return 1;
    }
    
    printf("SUCCESS: Schedule generated!\n");
    BypassScheduleGenerator::printSchedule(schedule2, N2, 2);
    
    visualizeScheduleOnPath(path2, obs2, schedule2, N2, dt2);
    
    // ========================================
    // Test 3: Multiple obstacles
    // ========================================
    
    printf("\n--- Test 3: Multiple Obstacles ---\n");
    
    Environment env3(0.0, 30.0, 0.0, 20.0, 0.1);
    
    RectangularObstacle obs3_1(10.0, 10.0, 4.0, 2.0, 0.0);
    RectangularObstacle obs3_2(20.0, 10.0, 3.0, 3.0, 0.0);
    
    env3.addObstacle(obs3_1);
    env3.addObstacle(obs3_2);
    
    std::vector<RectangularObstacle> obstacles3;
    obstacles3.push_back(obs3_1);
    obstacles3.push_back(obs3_2);
    
    RRTStarPlanner planner3(env3, 0.5);
    planner3.setStepSize(0.5);
    planner3.setRewireRadius(3.0);
    
    Point2D start3(2.0, 10.0);
    Point2D goal3(28.0, 10.0);
    
    printf("Obstacles: 2\n");
    printf("  Obstacle 1: center=(%.1f, %.1f), size=%.1f×%.1f\n",
           obs3_1.x_center, obs3_1.y_center, obs3_1.width, obs3_1.height);
    printf("  Obstacle 2: center=(%.1f, %.1f), size=%.1f×%.1f\n",
           obs3_2.x_center, obs3_2.y_center, obs3_2.width, obs3_2.height);
    
    RRTPath path3 = planner3.planWithTiming(start3, goal3, 5.0, 10.0, 5000);
    
    if (path3.getNumWaypoints() == 0) {
        printf("FAIL: No path found!\n");
        return 1;
    }
    
    printf("Path found: %d waypoints, %.2f m, %.2f s\n",
           path3.getNumWaypoints(), path3.getPathLength(), path3.getDuration());
    
    int N3 = 15;
    real_t dt3 = 0.2;
    BypassSide schedule3_1[16];
    BypassSide schedule3_2[16];
    
    std::vector<BypassSide*> schedules3;
    schedules3.push_back(schedule3_1);
    schedules3.push_back(schedule3_2);
    
    BypassScheduleGenerator generator3;
    bool success3 = generator3.generateSchedules(path3, obstacles3, N3, dt3, schedules3);
    
    if (!success3) {
        printf("FAIL: Failed to generate schedules!\n");
        return 1;
    }
    
    printf("SUCCESS: Schedules generated for all obstacles!\n");
    
    BypassScheduleGenerator::printSchedule(schedule3_1, N3, 1);
    BypassScheduleGenerator::printSchedule(schedule3_2, N3, 2);
    
    // ========================================
    // Summary
    // ========================================
    
    printf("\n=== Test Summary ===\n");
    printf("Test 1 (Single obstacle):    %s\n", success1 ? "PASS ✓" : "FAIL ✗");
    printf("Test 2 (Rotated obstacle):   %s\n", success2 ? "PASS ✓" : "FAIL ✗");
    printf("Test 3 (Multiple obstacles): %s\n", success3 ? "PASS ✓" : "FAIL ✗");
    
    printf("\n=== All Tests Complete ===\n");
    
    return (success1 && success2 && success3) ? 0 : 1;
}
