# Makefile for MPC-aware qpOASES Test Suite
#
# Builds test executables for Riccati LQR and O(N) TQ factorization validation

# Compiler settings
CXX = g++
CXXFLAGS = -g -O2 -Wall -std=c++11 -fno-omit-frame-pointer -fopenmp -Wno-format-truncation -D__SUPPRESSANYOUTPUT__
INCLUDES = -I../include -I.
LDFLAGS = -L../bin -lqpOASES -lm -fopenmp

# Directories
BINDIR = bin
UTILDIR = utils

# Targets
TESTS = test_2agent_rho25 test_4agent_rho25 test_6agent test_8agent

# Default target
all: $(BINDIR) $(TESTS)

# Create bin directory
$(BINDIR):
	@mkdir -p $(BINDIR)

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

test_8agent: $(BINDIR) test_8agent.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(BINDIR)/$@ test_8agent.cpp $(SOURCES) -lm
	@echo "Built: $(BINDIR)/$@"
