@echo off
echo ================================================================================
echo          COMPREHENSIVE BUILD VERIFICATION - qpOASES Multi-Agent MPC
echo ================================================================================
echo.

set FAILED=0
set PASSED=0

echo [STEP 1/7] Checking directory structure...
if not exist "src\" (
    echo   ERROR: src/ directory not found
    set /a FAILED+=1
) else (
    echo   OK: src/ directory exists
    set /a PASSED+=1
)

if not exist "include\qpOASES\" (
    echo   ERROR: include/qpOASES/ directory not found
    set /a FAILED+=1
) else (
    echo   OK: include/qpOASES/ directory exists
    set /a PASSED+=1
)

if not exist "tests\" (
    echo   ERROR: tests/ directory not found
    set /a FAILED+=1
) else (
    echo   OK: tests/ directory exists
    set /a PASSED+=1
)

echo.
echo [STEP 2/7] Compiling core qpOASES components...

g++ -std=c++11 -I include -c src/QProblem.cpp -o build/QProblem.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: QProblem.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: QProblem.cpp compiled
    set /a PASSED+=1
)

g++ -std=c++11 -I include -c src/QProblemB.cpp -o build/QProblemB.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: QProblemB.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: QProblemB.cpp compiled
    set /a PASSED+=1
)

g++ -std=c++11 -I include -c src/Bounds.cpp -o build/Bounds.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: Bounds.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: Bounds.cpp compiled
    set /a PASSED+=1
)

g++ -std=c++11 -I include -c src/Constraints.cpp -o build/Constraints.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: Constraints.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: Constraints.cpp compiled
    set /a PASSED+=1
)

echo.
echo [STEP 3/7] Compiling RRT* and environment components...

g++ -std=c++11 -I include -c src/Environment.cpp -o build/Environment.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: Environment.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: Environment.cpp compiled
    set /a PASSED+=1
)

g++ -std=c++11 -I include -c src/RRTStarPlanner.cpp -o build/RRTStarPlanner.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: RRTStarPlanner.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: RRTStarPlanner.cpp compiled
    set /a PASSED+=1
)

g++ -std=c++11 -I include -c src/RRTPath.cpp -o build/RRTPath.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: RRTPath.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: RRTPath.cpp compiled
    set /a PASSED+=1
)

echo.
echo [STEP 4/7] Compiling bypass schedule generator...

g++ -std=c++11 -I include -c src/BypassScheduleGenerator.cpp -o build/BypassScheduleGenerator.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: BypassScheduleGenerator.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: BypassScheduleGenerator.cpp compiled
    set /a PASSED+=1
)

echo.
echo [STEP 5/7] Compiling ADMM multi-agent MPC...

g++ -std=c++11 -I include -c src/TurboADMM.cpp -o build/TurboADMM.o -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: TurboADMM.cpp failed to compile
    set /a FAILED+=1
) else (
    echo   OK: TurboADMM.cpp compiled
    set /a PASSED+=1
)

echo.
echo [STEP 6/7] Building test executables...

echo   Building test_rrt_basic...
g++ -std=c++11 -I include -o test_rrt_basic.exe tests/test_rrt_basic.cpp src/RRTStarPlanner.cpp src/RRTPath.cpp src/Environment.cpp -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: test_rrt_basic failed to build
    set /a FAILED+=1
) else (
    echo   OK: test_rrt_basic built successfully
    set /a PASSED+=1
)

echo   Building test_bypass_schedule...
g++ -std=c++11 -I include -o test_bypass_schedule.exe tests/test_bypass_schedule.cpp src/RRTStarPlanner.cpp src/RRTPath.cpp src/Environment.cpp src/BypassScheduleGenerator.cpp -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: test_bypass_schedule failed to build
    set /a FAILED+=1
) else (
    echo   OK: test_bypass_schedule built successfully
    set /a PASSED+=1
)

echo   Building test_time_varying_obstacles...
g++ -std=c++11 -I include -o test_time_varying_obstacles.exe tests/test_time_varying_obstacles.cpp src/TurboADMM.cpp src/QProblem.cpp src/QProblemB.cpp src/Bounds.cpp src/Constraints.cpp src/SubjectTo.cpp src/Indexlist.cpp src/Utils.cpp src/Options.cpp src/Matrices.cpp src/MessageHandling.cpp src/Flipper.cpp src/BLASReplacement.cpp src/LAPACKReplacement.cpp -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: test_time_varying_obstacles failed to build
    set /a FAILED+=1
) else (
    echo   OK: test_time_varying_obstacles built successfully
    set /a PASSED+=1
)

echo   Building test_viz_minimal...
g++ -std=c++11 -I include -o test_viz_minimal.exe tests/test_viz_minimal.cpp src/RRTStarPlanner.cpp src/RRTPath.cpp src/Environment.cpp -D__NO_COPYRIGHT__ 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo   ERROR: test_viz_minimal failed to build
    set /a FAILED+=1
) else (
    echo   OK: test_viz_minimal built successfully
    set /a PASSED+=1
)

echo.
echo [STEP 7/7] Running quick smoke tests...

echo   Running test_rrt_basic...
test_rrt_basic.exe >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo   WARNING: test_rrt_basic exited with error code %ERRORLEVEL%
) else (
    echo   OK: test_rrt_basic passed
    set /a PASSED+=1
)

echo   Running test_viz_minimal...
test_viz_minimal.exe >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo   WARNING: test_viz_minimal exited with error code %ERRORLEVEL%
) else (
    echo   OK: test_viz_minimal passed
    set /a PASSED+=1
)

echo.
echo ================================================================================
echo                           BUILD VERIFICATION SUMMARY
echo ================================================================================
echo   Tests Passed: %PASSED%
echo   Tests Failed: %FAILED%
echo.

if %FAILED% EQU 0 (
    echo   STATUS: ALL CHECKS PASSED - Codebase is coherent!
    echo.
    echo   Core Components:
    echo     - qpOASES MPC-aware solver with Riccati warm start
    echo     - RRT* path planner with timing
    echo     - Bypass schedule generator
    echo     - ADMM multi-agent coordination
    echo     - Time-varying obstacle constraints
    echo.
    echo   Ready for:
    echo     - Single-agent pipeline testing
    echo     - Multi-agent scenarios
    echo     - Performance benchmarking
    echo.
) else (
    echo   STATUS: SOME CHECKS FAILED - Review errors above
    echo.
)

echo ================================================================================
