# Makefile for MPC-aware qpOASES Test Suite
#
# Builds test executables for Riccati LQR and O(N) TQ factorization validation

# Compiler settings
CXX = g++
CXXFLAGS = -O2 -Wall -std=c++11
INCLUDES = -I../include -I.
LDFLAGS = -L../bin -lqpOASES -lm

# Directories
BINDIR = bin
UTILDIR = utils

# Targets
TESTS = test_riccati_mpc test_realworld_mpc benchmark_comparison benchmark_horizon_sweep diagnostic_warmstart debug_mpc_path

# Default target
all: $(BINDIR) $(TESTS)

# Create bin directory
$(BINDIR):
	@mkdir -p $(BINDIR)

# Build synthetic tests
test_riccati_mpc: $(BINDIR) test_riccati_mpc.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_riccati_mpc.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Build real-world tests
test_realworld_mpc: $(BINDIR) test_realworld_mpc.cpp $(UTILDIR)/test_problems.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_realworld_mpc.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Build comparison benchmark
benchmark_comparison: $(BINDIR) benchmark_comparison.cpp $(UTILDIR)/test_problems.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ benchmark_comparison.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Build horizon sweep benchmark
benchmark_horizon_sweep: $(BINDIR) benchmark_horizon_sweep.cpp $(UTILDIR)/test_problems.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ benchmark_horizon_sweep.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Build diagnostic warmstart test
diagnostic_warmstart: $(BINDIR) diagnostic_warmstart.cpp $(UTILDIR)/test_problems.hpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ diagnostic_warmstart.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Build minimal debug test
debug_mpc_path: $(BINDIR) debug_mpc_path.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ debug_mpc_path.cpp $(LDFLAGS)
	@echo "Built: $(BINDIR)/$@"

# Run all tests
test: all
	@echo ""
	@echo "========================================="
	@echo "  Running Synthetic Tests"
	@echo "========================================="
	@./$(BINDIR)/test_riccati_mpc
	@echo ""
	@echo "========================================="
	@echo "  Running Real-World Tests"
	@echo "========================================="
	@./$(BINDIR)/test_realworld_mpc

# Run only synthetic tests
test-synthetic: test_riccati_mpc
	@./$(BINDIR)/test_riccati_mpc

# Run only real-world tests
test-realworld: test_realworld_mpc
	@./$(BINDIR)/test_realworld_mpc

# Run comparison benchmark
benchmark: benchmark_comparison
	@echo ""
	@echo "========================================="
	@echo "  Running Performance Comparison"
	@echo "========================================="
	@./$(BINDIR)/benchmark_comparison

# Run diagnostic warmstart test
diagnostic: diagnostic_warmstart
	@./$(BINDIR)/diagnostic_warmstart

# Run minimal debug test
debug: debug_mpc_path
	@./$(BINDIR)/debug_mpc_path

# Clean
clean:
	rm -rf $(BINDIR)
	@echo "Cleaned test binaries"

.PHONY: all test test-synthetic test-realworld clean
