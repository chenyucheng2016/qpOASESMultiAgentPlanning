# Consensus-Based Degenerate Collision Resolution

## Problem Statement

When multiple agents are in a **degenerate collision scenario** (e.g., head-on collision with both agents on the same line), the standard symmetric projection cannot create separation in the perpendicular direction. This results in:

- Agents staying on the same line (y-coordinates don't change)
- Collision violations (distance < d_safe)
- ADMM unable to converge

**Example:** Two agents at (4.65, 5.00) and (6.64, 5.00) with distance 1.988m < 2.0m safety.

---

## Design Goals

1. **Extensible:** Handle 2, 3, ..., N agents converging at a single point
2. **Distributed:** Each agent computes independently using only agent IDs (common knowledge)
3. **Deterministic:** Same agent IDs → same conflict resolution
4. **Geometric:** Clear spatial arrangement (circular/radial pattern)
5. **Smooth:** Gradual transition via ADMM iterations

---

## Architecture Overview

### 1. Degeneration Detection

**Criteria for degenerate collision:**
```
degenerate = (dist < d_safe) AND (collinearity < threshold) AND (approaching)

where:
  collinearity = |dy| / dist  (0 = perfectly collinear)
  approaching = (v_rel < 0)   (negative relative velocity)
```

**Multi-agent cluster detection:**
```
For each agent i at stage k:
  conflict_cluster[i] = {j : dist(i,j) < 1.5*d_safe AND approaching(i,j)}
  
If |conflict_cluster[i]| >= 2:
  → Multi-agent conflict detected
```

---

### 2. Consensus Protocol

**For N agents in conflict, establish deterministic arrangement:**

#### **Pairwise (N=2):**
```
Rule: Lower ID passes on RIGHT, higher ID passes on LEFT

if (agent_i < agent_j):
    agent_i → RIGHT (y decreases)
    agent_j → LEFT (y increases)
else:
    agent_i → LEFT (y increases)
    agent_j → RIGHT (y decreases)
```

#### **Multi-agent (N≥3):**
```
Rule: Arrange agents in circle around conflict centroid

1. Sort agents by ID: [a₀, a₁, ..., aₙ₋₁]
2. Assign angles: θᵢ = 2π * i / N
3. Compute radius: R = d_safe / (2 * sin(π/N))
4. Position: pᵢ = centroid + R * (cos(θᵢ), sin(θᵢ))
```

**Example (3 agents):**
```
Agent IDs: [0, 1, 2]
Angles: [0°, 120°, 240°]
Positions form equilateral triangle around centroid
```

---

### 3. Projection Modification

**Current (symmetric):**
```cpp
v_i = midpoint + (d_safe/2) * direction
v_j = midpoint - (d_safe/2) * direction
```

**Proposed (consensus-aware):**
```cpp
if (degenerate_collision):
    // Perpendicular separation
    perp_dir = rotate_90(direction)
    
    if (agent_i < agent_j):
        v_i = current_i - lateral_offset * perp_dir  // RIGHT
        v_j = current_j + lateral_offset * perp_dir  // LEFT
    else:
        v_i = current_i + lateral_offset * perp_dir  // LEFT
        v_j = current_j - lateral_offset * perp_dir  // RIGHT
else:
    // Normal symmetric projection
    v_i = midpoint + (d_safe/2) * direction
    v_j = midpoint - (d_safe/2) * direction
```

---

### 4. Geometric Properties

**For N agents in circular arrangement:**

| N | Angle Spacing | Min Radius (d_safe=2m) | Arrangement |
|---|---------------|------------------------|-------------|
| 2 | 180° | 1.0m | Line |
| 3 | 120° | 1.15m | Triangle |
| 4 | 90° | 1.41m | Square |
| 5 | 72° | 1.70m | Pentagon |
| 6 | 60° | 2.0m | Hexagon |

**Formula:** `R_min = d_safe / (2 * sin(π/N))`

---

## Implementation Phases

### **Phase 1: Simple Pairwise Consensus (N=2)**
- Detect collinear head-on collisions
- Apply ID-based left/right rule
- Test on current 2-agent scenario

### **Phase 2: Multi-Agent Clusters (N≥3)**
- Identify conflict clusters
- Compute circular arrangement
- Test on 3-agent and 4-agent scenarios

### **Phase 3: Fallback Strategies**
- Deadlock mode (all agents stop)
- Priority-based resolution
- Sequential passing

---

## Test Scenarios

### **Scenario 1: Head-On (2 agents)**
```
Agent 0: (0, 5) → (15, 5)
Agent 1: (15, 5) → (0, 5)
Expected: Agent 0 passes below, Agent 1 passes above
```

### **Scenario 2: Y-Junction (3 agents)**
```
Agent 0: (0, 5) → (10, 5)
Agent 1: (10, 0) → (10, 10)
Agent 2: (20, 5) → (10, 5)
Expected: Agents form triangle around (10, 5)
```

### **Scenario 3: Crossroads (4 agents)**
```
Agent 0: (5, 0) → (5, 10)   [North]
Agent 1: (10, 5) → (0, 5)   [West]
Agent 2: (5, 10) → (5, 0)   [South]
Agent 3: (0, 5) → (10, 5)   [East]
Expected: Agents form square around (5, 5)
```

---

## Key Advantages

1. **Deterministic:** Agent IDs provide common knowledge
2. **Distributed:** No communication needed
3. **Fair:** Symmetric rule (no agent has priority)
4. **Extensible:** Works for 2, 3, ..., N agents
5. **Smooth:** Gradual convergence via ADMM

---

## References

- Symmetric projection: Current implementation in `TurboADMM.cpp` lines 2203-2225
- ADMM convergence check: `TurboADMM.cpp` lines 1867-1941
- Position-only coupling: Memory[a60c8f8a] (Oct 9, 2025 achievements)
