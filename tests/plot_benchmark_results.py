#!/usr/bin/env python3
"""
Plot benchmark results: Vanilla vs MPC-Aware qpOASES
Reads benchmark_horizon_results.csv and generates performance plots
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read benchmark results
try:
    df = pd.read_csv('benchmark_horizon_results.csv')
except FileNotFoundError:
    print("Error: benchmark_horizon_results.csv not found!")
    print("Please run the horizon sweep benchmark first:")
    print("  cd tests")
    print("  make benchmark_horizon_sweep")
    print("  LD_LIBRARY_PATH=../bin:$LD_LIBRARY_PATH ./bin/benchmark_horizon_sweep")
    exit(1)

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# ============================================================================
# Plot 1: Time vs Horizon
# ============================================================================

ax1.plot(df['Horizon'], df['Vanilla_ms'], 'o-', linewidth=2, markersize=8, 
         label='Vanilla qpOASES', color='#e74c3c')
ax1.plot(df['Horizon'], df['MPC_ms'], 's-', linewidth=2, markersize=8,
         label='MPC-Aware qpOASES', color='#27ae60')

ax1.set_xlabel('Planning Horizon (N)', fontsize=12, fontweight='bold')
ax1.set_ylabel('Solve Time (ms)', fontsize=12, fontweight='bold')
ax1.set_title('Performance Comparison: Vanilla vs MPC-Aware qpOASES', 
              fontsize=14, fontweight='bold')
ax1.legend(fontsize=11, loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_yscale('log')  # Log scale to show both methods clearly

# Add speedup annotations
for i, row in df.iterrows():
    if i % 2 == 0:  # Annotate every other point to avoid crowding
        speedup = row['Speedup']
        ax1.annotate(f'{speedup:.1f}x', 
                    xy=(row['Horizon'], row['MPC_ms']),
                    xytext=(10, 10), textcoords='offset points',
                    fontsize=9, color='#27ae60', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', 
                             edgecolor='#27ae60', alpha=0.7))

# ============================================================================
# Plot 2: Speedup vs Horizon
# ============================================================================

colors = plt.cm.viridis(np.linspace(0.3, 0.9, len(df)))
bars = ax2.bar(range(len(df)), df['Speedup'], color=colors, 
               edgecolor='black', linewidth=1.5, alpha=0.8)

# Add value labels on bars
for i, (bar, speedup) in enumerate(zip(bars, df['Speedup'])):
    height = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2., height,
            f'{speedup:.1f}x',
            ha='center', va='bottom', fontsize=10, fontweight='bold')

ax2.set_xlabel('Planning Horizon (N)', fontsize=12, fontweight='bold')
ax2.set_ylabel('Speedup Factor', fontsize=12, fontweight='bold')
ax2.set_title('MPC-Aware Speedup vs Planning Horizon', 
              fontsize=14, fontweight='bold')
ax2.set_xticks(range(len(df)))
ax2.set_xticklabels(df['Horizon'])
ax2.grid(True, alpha=0.3, axis='y')
ax2.axhline(y=1, color='red', linestyle='--', linewidth=2, alpha=0.5, 
           label='No speedup')
ax2.legend(fontsize=10)

# Add horizontal lines for reference
ax2.axhline(y=10, color='orange', linestyle=':', linewidth=1.5, alpha=0.5)
ax2.text(len(df)-0.5, 10.5, '10x', fontsize=9, color='orange', fontweight='bold')

plt.tight_layout()
plt.savefig('benchmark_results.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved to: benchmark_results.png")

# ============================================================================
# Additional Plot: Iterations Comparison
# ============================================================================

fig2, ax3 = plt.subplots(figsize=(10, 6))

x = np.arange(len(df))
width = 0.35

bars1 = ax3.bar(x - width/2, df['Vanilla_Iterations'], width, 
               label='Vanilla qpOASES', color='#e74c3c', alpha=0.8,
               edgecolor='black', linewidth=1.5)
bars2 = ax3.bar(x + width/2, df['MPC_Iterations'], width,
               label='MPC-Aware qpOASES', color='#27ae60', alpha=0.8,
               edgecolor='black', linewidth=1.5)

ax3.set_xlabel('Planning Horizon (N)', fontsize=12, fontweight='bold')
ax3.set_ylabel('Number of Iterations', fontsize=12, fontweight='bold')
ax3.set_title('Iteration Count Comparison', fontsize=14, fontweight='bold')
ax3.set_xticks(x)
ax3.set_xticklabels(df['Horizon'])
ax3.legend(fontsize=11)
ax3.grid(True, alpha=0.3, axis='y')

# Add value labels
for bars in [bars1, bars2]:
    for bar in bars:
        height = bar.get_height()
        if height > 0:
            ax3.text(bar.get_x() + bar.get_width()/2., height,
                    f'{int(height)}',
                    ha='center', va='bottom', fontsize=9)

plt.tight_layout()
plt.savefig('benchmark_iterations.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved to: benchmark_iterations.png")

# ============================================================================
# Print Summary Statistics
# ============================================================================

print("\n" + "="*60)
print("BENCHMARK SUMMARY")
print("="*60)
print(f"\nAverage speedup: {df['Speedup'].mean():.1f}x")
print(f"Maximum speedup: {df['Speedup'].max():.1f}x (N={df.loc[df['Speedup'].idxmax(), 'Horizon']})")
print(f"Minimum speedup: {df['Speedup'].min():.1f}x (N={df.loc[df['Speedup'].idxmin(), 'Horizon']})")
print(f"\nMaximum solution error: {df['Solution_Error'].max():.2e}")
print(f"Average solution error: {df['Solution_Error'].mean():.2e}")

if df['Solution_Error'].max() < 1e-6:
    print("\n✅ All solutions verified: Errors below 1e-6 tolerance")
else:
    print(f"\n⚠️  Some solutions have errors above 1e-6")

print("\n" + "="*60)

plt.show()
