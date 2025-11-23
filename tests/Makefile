# Makefile for MPC-aware qpOASES Test Suite
#
# Builds test executables for Riccati LQR and O(N) TQ factorization validation

# Compiler settings
CXX = g++
CXXFLAGS = -g -O2 -Wall -std=c++11 -fno-omit-frame-pointer
INCLUDES = -I../include -I.
LDFLAGS = -L../bin -lqpOASES -lm

# Directories
BINDIR = bin
UTILDIR = utils

# Targets
TESTS = test_riccati_mpc test_realworld_mpc benchmark_comparison benchmark_horizon_sweep diagnostic_warmstart debug_mpc_path test_2agent_rho25 test_2agent_rho25_osqp test_6agent

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

# Build 2-agent ADMM test with rho=25 (requires TurboADMM and all qpOASES sources)
SRCDIR = ../src
SOURCES = $(SRCDIR)/TurboADMM.cpp $(SRCDIR)/QProblem.cpp $(SRCDIR)/QProblemB.cpp \
          $(SRCDIR)/Bounds.cpp $(SRCDIR)/Constraints.cpp $(SRCDIR)/SubjectTo.cpp \
          $(SRCDIR)/Indexlist.cpp $(SRCDIR)/Utils.cpp $(SRCDIR)/Options.cpp \
          $(SRCDIR)/Matrices.cpp $(SRCDIR)/MessageHandling.cpp \
          $(SRCDIR)/Flipper.cpp $(SRCDIR)/BLASReplacement.cpp $(SRCDIR)/LAPACKReplacement.cpp

test_2agent_rho25: $(BINDIR) test_2agent_rho25.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_2agent_rho25.cpp $(SOURCES) -lm
	@echo "Built: $(BINDIR)/$@"

test_4agent_rho25: $(BINDIR) test_4agent_rho25.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_4agent_rho25.cpp $(SOURCES) -lm
	@echo "Built: $(BINDIR)/$@"

test_6agent: $(BINDIR) test_6agent.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_6agent.cpp $(SOURCES) -lm
	@echo "Built: $(BINDIR)/$@"

# Build 2-agent ADMM test with OSQP solver (requires Eigen)
test_2agent_rho25_osqp: $(BINDIR) test_2agent_rho25_osqp.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -I"C:/osqp/include" -I"C:/eigen" -o $(BINDIR)/$@ test_2agent_rho25_osqp.cpp $(SOURCES) -D__NO_COPYRIGHT__ -L"C:/osqp/lib" -losqp -lm
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
