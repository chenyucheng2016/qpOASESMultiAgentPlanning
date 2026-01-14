#!/bin/bash

# Directories
TESTS_BIN="/mnt/c/Users/cheny/Documents/GitHub/qpOASESMultiAgentPlanning/tests/bin"
BUILD_BIN="/mnt/c/Users/cheny/Documents/GitHub/qpOASESMultiAgentPlanning/build/bin"

# Function to run a binary 30 times and compute mean and std of execution time
run_binary() {
    local binary_path="$1"
    local binary_name="$2"
    local times=()

    echo "Running $binary_name 30 times..."

    for i in {1..30}; do
        # Run the binary and capture output
        output=$( "$binary_path" 2>&1 )
        # Parse time in ms from the output (assuming format: "Solve time: XXX ms")
        time_ms=$( echo "$output" | grep "Solve time:" | sed 's/.*Solve time: \([0-9.]*\) ms.*/\1/' | head -40 )
        # If not found, try other formats or set to 0
        if [[ -z "$time_ms" ]]; then
            echo "Warning: Could not parse time for $binary_name run $i"
            time_ms="0"
        fi
        times+=("$time_ms")
    done

    # Compute mean and std using awk
    mean=$(printf '%s\n' "${times[@]}" | awk '{sum+=$1} END {print sum/NR}')
    std=$(printf '%s\n' "${times[@]}" | awk -v mean="$mean" '{sum+=($1-mean)^2} END {print sqrt(sum/(NR-1))}')

    # Format output
    printf "Binary: %s\n" "$binary_name"
    printf "Mean: %.2f ms\n" "$mean"
    printf "Std: %.2f ms\n" "$std"
    printf "\n"
}

echo "=== Benchmarking binaries in tests/bin ==="
for exe_path in "$TESTS_BIN"/*; do
    if [[ -f "$exe_path" && -x "$exe_path" ]]; then
        binary_name=$(basename "$exe_path")
        if [[ "$binary_name" == *"8agent"* ]]; then continue; fi
        run_binary "$exe_path" "$binary_name"
    fi
done

echo "=== Benchmarking binaries starting with 'test' in build/bin ==="
for exe_path in "$BUILD_BIN"/test*; do
    if [[ -f "$exe_path" && -x "$exe_path" ]]; then
        binary_name=$(basename "$exe_path")
        if [[ "$binary_name" == *"8agent"* ]]; then continue; fi
        if [[ "$binary_name" == *"_hpipm" && "$binary_name" != "test_2agent_hpipm" ]]; then continue; fi
        run_binary "$exe_path" "$binary_name"
    fi
done
